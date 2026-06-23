import Foundation
import KuyuCore
import simd

struct ArticulatedContactSolver: Sendable {
    private let body: KuyuBodyModel
    private let world: KuyuWorldModel
    private let bindings: [ArticulatedJointBinding]
    private let kinematics: ArticulatedKinematics
    private let bindingIndexByJointID: [String: Int]
    private let inverseInertiaByBindingIndex: [Double]
    private let ancestorJointIDsByLinkID: [String: [String]]
    private let orderedLinkIDs: [String]
    private let linkByID: [String: LinkDefinition]
    private let bodyMaterialsByID: [String: BodyMaterial]
    private let surfaces: [ContactSurface]

    init(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        bindings: [ArticulatedJointBinding],
        dynamics: [ArticulatedJointDynamics],
        topology: ArticulatedSnapshotTopology
    ) throws {
        guard world.contact.mode != .disabled else {
            throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.disabled")
        }
        guard world.solver.kind == .deterministicConstraint else {
            throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld(
                "contact.\(world.contact.mode.rawValue).solver.\(world.solver.kind.rawValue)"
            )
        }
        guard dynamics.count == bindings.count else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.dynamics-count")
        }
        for index in bindings.indices where bindings[index].joint.id != dynamics[index].joint.id {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.dynamics-order")
        }
        self.body = body
        self.world = world
        self.bindings = bindings
        self.kinematics = ArticulatedKinematics(topology: topology)
        self.bindingIndexByJointID = Dictionary(uniqueKeysWithValues: bindings.enumerated().map { ($0.element.joint.id, $0.offset) })
        self.inverseInertiaByBindingIndex = dynamics.map { 1.0 / max($0.effectiveInertia, 1e-12) }
        self.ancestorJointIDsByLinkID = Self.ancestorJointIDsByLinkID(topology: topology)
        self.orderedLinkIDs = topology.rootLinks.map(\.id) + topology.orderedJoints.map(\.childLinkID)
        self.linkByID = Dictionary(uniqueKeysWithValues: body.links.map { ($0.id, $0) })
        self.bodyMaterialsByID = Dictionary(uniqueKeysWithValues: body.materials.map { ($0.id, $0) })
        self.surfaces = try Self.contactSurfaces(from: world)
        try validateCollisions(body: body)
    }

    func contactTorques(
        state: ArticulatedState,
        deltaTime: Double
    ) throws -> ([Double], ArticulatedContactMetrics) {
        var torques = Array(repeating: 0.0, count: bindings.count)
        guard world.contact.mode == .penalty else {
            return (torques, .disabled)
        }
        let batch = try allContacts(state: state)
        let contacts = batch.contacts.filter { $0.penetration > 0 }
        guard !contacts.isEmpty else {
            return (torques, .disabled)
        }
        let stiffness = world.contact.stiffness ?? 0
        let damping = world.contact.damping ?? 0
        var metrics = ArticulatedContactMetrics(
            activeContacts: contacts.count,
            maxPenetration: contacts.map(\.penetration).max() ?? 0,
            maxNormalImpulse: 0,
            maxNormalForce: 0,
            solverIterations: 1
        )

        for contact in contacts {
            let closingVelocity = min(contact.normalVelocity, 0)
            var normalForce = stiffness * contact.penetration - damping * closingVelocity
            if contact.normalVelocity < 0 {
                normalForce += contact.material.restitution * (-contact.normalVelocity) * contact.linkMass / deltaTime
            }
            guard normalForce > 0 else { continue }
            let tangentialVelocity = contact.pointVelocity - contact.normal * simd_dot(contact.pointVelocity, contact.normal)
            let tangentialSpeed = simd_length(tangentialVelocity)
            var force = contact.normal * normalForce
            if tangentialSpeed > 1e-12 {
                let frictionLimit = contact.material.dynamicFriction * normalForce
                let stopForce = contact.linkMass * tangentialSpeed / deltaTime
                force -= (tangentialVelocity / tangentialSpeed) * min(frictionLimit, stopForce)
            }
            let entries = jacobianEntries(for: contact, evaluation: batch.evaluation)
            for entry in entries {
                torques[entry.index] += simd_dot(entry.linear, force)
            }
            metrics.maxNormalForce = max(metrics.maxNormalForce, normalForce)
            metrics.maxNormalImpulse = max(metrics.maxNormalImpulse, normalForce * deltaTime)
        }
        try ensureFinite(torques, field: "contact.torques")
        return (torques, metrics)
    }

    func project(
        state: ArticulatedState,
        deltaTime: Double
    ) throws -> ArticulatedStateStepResult {
        guard world.contact.mode == .constraint else {
            return ArticulatedStateStepResult(state: state, contactMetrics: .disabled)
        }
        var next = state
        let tolerance = max(world.solver.tolerance, 1e-9)
        let iterations = max(world.solver.iterations, 1)
        var metrics = ArticulatedContactMetrics.disabled
        var iterationsUsed = 0
        var observedContacts = 0

        for iteration in 0..<iterations {
            iterationsUsed = iteration + 1
            let batch = try allContacts(state: next)
            let contacts = batch.contacts.filter { $0.penetration > 0 }
            observedContacts = max(observedContacts, contacts.count)
            let maxPenetration = contacts.map(\.penetration).max() ?? 0
            if maxPenetration <= tolerance {
                metrics.maxPenetration = maxPenetration
                break
            }
            var deltas = Array(repeating: 0.0, count: next.position.count)
            for contact in contacts {
                let entries = jacobianEntries(for: contact, evaluation: batch.evaluation)
                let denominator = inverseInertiaWeightedNormalDenominator(entries: entries, normal: contact.normal)
                guard denominator > 1e-18 else { continue }
                let lambda = contact.penetration / denominator
                for entry in entries {
                    let normalJacobian = simd_dot(entry.linear, contact.normal)
                    let rawDelta = inverseInertiaByBindingIndex[entry.index] * normalJacobian * lambda
                    deltas[entry.index] += clamp(rawDelta, lower: -0.05, upper: 0.05)
                }
            }
            for index in next.position.indices {
                next.position[index] = try clampedPosition(
                    next.position[index] + deltas[index],
                    joint: bindings[index].joint,
                    field: "contact.position[\(index)]"
                )
            }
        }

        let finalBatch = try allContacts(state: next)
        let finalContacts = finalBatch.contacts.filter { $0.penetration > 0 }
        let residualPenetration = finalContacts.map(\.penetration).max() ?? 0
        if residualPenetration > tolerance {
            throw ArticulatedRigidBodySimulator.SimulationError.unresolvedContact(
                maxPenetration: residualPenetration,
                tolerance: tolerance
            )
        }

        var velocityImpulseMax = 0.0
        let velocityContacts = finalBatch.contacts.filter { $0.penetration >= -tolerance }
        observedContacts = max(observedContacts, velocityContacts.count)
        for contact in velocityContacts where contact.normalVelocity < 0 {
            let entries = jacobianEntries(for: contact, evaluation: finalBatch.evaluation)
            let denominator = inverseInertiaWeightedNormalDenominator(entries: entries, normal: contact.normal)
            guard denominator > 1e-18 else { continue }
            let impulse = -(1 + contact.material.restitution) * contact.normalVelocity / denominator
            for entry in entries {
                let normalJacobian = simd_dot(entry.linear, contact.normal)
                next.velocity[entry.index] = try clampedVelocity(
                    next.velocity[entry.index] + inverseInertiaByBindingIndex[entry.index] * normalJacobian * impulse,
                    joint: bindings[entry.index].joint,
                    field: "contact.velocity[\(entry.index)]"
                )
            }
            velocityImpulseMax = max(velocityImpulseMax, abs(impulse))
        }

        metrics.activeContacts = observedContacts
        metrics.maxPenetration = residualPenetration
        metrics.maxNormalImpulse = velocityImpulseMax
        metrics.solverIterations = iterationsUsed
        try ensureFinite(next.position, field: "contact.position")
        try ensureFinite(next.velocity, field: "contact.velocity")
        return ArticulatedStateStepResult(state: next, contactMetrics: metrics)
    }

    private func allContacts(state: ArticulatedState) throws -> ContactEvaluationBatch {
        let scalars = try scalarState(state)
        let evaluation = try kinematics.evaluate(scalars: scalars)
        var contacts: [ArticulatedContact] = []
        for linkID in orderedLinkIDs {
            guard let link = linkByID[linkID],
                  let linkState = evaluation.linksByID[linkID] else {
                continue
            }
            let material = try material(for: link)
            for collision in link.collisions.sorted(by: { $0.id < $1.id }) {
                let witness = try witness(for: collision, linkState: linkState)
                for surface in surfaces {
                    guard surface.contains(witness.point) else { continue }
                    let penetration = surface.planeZ - witness.point.z
                    contacts.append(ArticulatedContact(
                        linkID: linkID,
                        point: witness.point,
                        pointVelocity: witness.velocity,
                        normal: surface.normal,
                        penetration: penetration,
                        normalVelocity: simd_dot(witness.velocity, surface.normal),
                        material: material.combined(with: surface.material),
                        linkMass: max(link.mass, 1e-9)
                    ))
                }
            }
        }
        contacts.sort {
            if $0.linkID != $1.linkID { return $0.linkID < $1.linkID }
            if $0.point.x != $1.point.x { return $0.point.x < $1.point.x }
            if $0.point.y != $1.point.y { return $0.point.y < $1.point.y }
            return $0.point.z < $1.point.z
        }
        return ContactEvaluationBatch(contacts: contacts, evaluation: evaluation)
    }

    private func jacobianEntries(
        for contact: ArticulatedContact,
        evaluation: ArticulatedKinematicEvaluation
    ) -> [ContactJacobianEntry] {
        let ancestorJointIDs = ancestorJointIDsByLinkID[contact.linkID] ?? []
        var entries: [ContactJacobianEntry] = []
        entries.reserveCapacity(ancestorJointIDs.count)
        for jointID in ancestorJointIDs {
            guard let index = bindingIndexByJointID[jointID],
                  let jointState = evaluation.jointsByID[jointID] else {
                continue
            }
            let linear: SIMD3<Double>
            switch jointState.joint.kind {
            case .fixed:
                continue
            case .revolute, .continuous:
                linear = simd_cross(jointState.axisWorld, contact.point - jointState.originPosition)
            case .prismatic:
                linear = jointState.axisWorld
            }
            entries.append(ContactJacobianEntry(index: index, linear: linear))
        }
        return entries
    }

    private func inverseInertiaWeightedNormalDenominator(
        entries: [ContactJacobianEntry],
        normal: SIMD3<Double>
    ) -> Double {
        entries.reduce(0.0) { partial, entry in
            let normalJacobian = simd_dot(entry.linear, normal)
            return partial + normalJacobian * normalJacobian * inverseInertiaByBindingIndex[entry.index]
        }
    }

    private func scalarState(_ state: ArticulatedState) throws -> [String: Double] {
        var scalars: [String: Double] = [:]
        var positionsByJointID: [String: Double] = [:]
        var velocitiesByJointID: [String: Double] = [:]
        for index in state.position.indices {
            let jointID = bindings[index].joint.id
            scalars[jointID] = state.position[index]
            scalars["velocity_\(jointID)"] = state.velocity[index]
            positionsByJointID[jointID] = state.position[index]
            velocitiesByJointID[jointID] = state.velocity[index]
        }

        var unresolvedMimics = body.joints.filter { $0.mimic != nil }
        while !unresolvedMimics.isEmpty {
            var remaining: [JointDefinition] = []
            var resolvedCount = 0
            for joint in unresolvedMimics {
                guard let mimic = joint.mimic,
                      let masterPosition = positionsByJointID[mimic.jointID] else {
                    remaining.append(joint)
                    continue
                }
                let masterVelocity = velocitiesByJointID[mimic.jointID] ?? 0
                let position = masterPosition * mimic.multiplier + mimic.offset
                let velocity = masterVelocity * mimic.multiplier
                positionsByJointID[joint.id] = position
                velocitiesByJointID[joint.id] = velocity
                scalars[joint.id] = position
                scalars["velocity_\(joint.id)"] = velocity
                resolvedCount += 1
            }
            if resolvedCount == 0 {
                let ids = remaining.map(\.id).joined(separator: ",")
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("mimic.\(ids)")
            }
            unresolvedMimics = remaining
        }
        return scalars
    }

    private func witness(
        for geometry: GeometryInstance,
        linkState: ArticulatedLinkKinematicState
    ) throws -> CollisionWitness {
        let poseOrientation = orientation(fromRPY: geometry.pose.rpy)
        let worldOrientation = (linkState.orientation * poseOrientation).normalizedQuat
        let center = linkState.position + linkState.orientation.act(simdVector(geometry.pose.xyz))
        let point: SIMD3<Double>
        switch geometry.kind {
        case .sphere:
            guard let radius = geometry.radius else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.geometry.\(geometry.id).radius")
            }
            let scale = geometry.scale.map { max($0.x, max($0.y, $0.z)) } ?? 1
            point = center - SIMD3<Double>(0, 0, radius * scale)
        case .box:
            guard let size = geometry.size else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.geometry.\(geometry.id).size")
            }
            let scale = geometry.scale ?? KuyuVector3(x: 1, y: 1, z: 1)
            let half = SIMD3<Double>(size.x * scale.x, size.y * scale.y, size.z * scale.z) * 0.5
            point = boxVertices(halfExtents: half, orientation: worldOrientation, center: center).min { lhs, rhs in
                lhs.z < rhs.z
            } ?? center
        case .cylinder:
            guard let radius = geometry.radius, let length = geometry.length else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.geometry.\(geometry.id).cylinder")
            }
            let scale = geometry.scale ?? KuyuVector3(x: 1, y: 1, z: 1)
            let scaledRadius = radius * max(scale.x, scale.y)
            let halfLength = length * scale.z * 0.5
            point = cylinderWitnessPoint(
                radius: scaledRadius,
                halfLength: halfLength,
                orientation: worldOrientation,
                center: center
            )
        case .mesh:
            throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.geometry.mesh.\(geometry.id)")
        }
        let velocity = linkState.velocity + simd_cross(linkState.angularVelocity, point - linkState.position)
        return CollisionWitness(point: point, velocity: velocity)
    }

    private func material(for link: LinkDefinition) throws -> ContactMaterial {
        guard let materialID = link.materialID else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.material.\(link.id)")
        }
        guard let material = bodyMaterialsByID[materialID] else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.material.\(materialID)")
        }
        guard let staticFriction = material.staticFriction else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody(
                "contact.material.\(materialID).staticFriction"
            )
        }
        guard let dynamicFriction = material.dynamicFriction else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody(
                "contact.material.\(materialID).dynamicFriction"
            )
        }
        guard let restitution = material.restitution else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody(
                "contact.material.\(materialID).restitution"
            )
        }
        return ContactMaterial(
            staticFriction: staticFriction,
            dynamicFriction: dynamicFriction,
            restitution: restitution
        )
    }

    private func clampedPosition(_ value: Double, joint: JointDefinition, field: String) throws -> Double {
        guard value.isFinite else {
            throw ArticulatedRigidBodySimulator.SimulationError.nonFiniteState(field)
        }
        var result = value
        if let lower = joint.lowerLimit {
            result = max(result, lower)
        }
        if let upper = joint.upperLimit {
            result = min(result, upper)
        }
        return result
    }

    private func clampedVelocity(_ value: Double, joint: JointDefinition, field: String) throws -> Double {
        guard value.isFinite else {
            throw ArticulatedRigidBodySimulator.SimulationError.nonFiniteState(field)
        }
        guard let limit = joint.velocityLimit else { return value }
        return clamp(value, lower: -limit, upper: limit)
    }

    private func ensureFinite(_ values: [Double], field: String) throws {
        for (index, value) in values.enumerated() where !value.isFinite {
            throw ArticulatedRigidBodySimulator.SimulationError.nonFiniteState("\(field)[\(index)]")
        }
    }

    private func boxVertices(
        halfExtents: SIMD3<Double>,
        orientation: simd_quatd,
        center: SIMD3<Double>
    ) -> [SIMD3<Double>] {
        var vertices: [SIMD3<Double>] = []
        vertices.reserveCapacity(8)
        for x in [-halfExtents.x, halfExtents.x] {
            for y in [-halfExtents.y, halfExtents.y] {
                for z in [-halfExtents.z, halfExtents.z] {
                    vertices.append(center + orientation.act(SIMD3<Double>(x, y, z)))
                }
            }
        }
        return vertices
    }

    private func cylinderWitnessPoint(
        radius: Double,
        halfLength: Double,
        orientation: simd_quatd,
        center: SIMD3<Double>
    ) -> SIMD3<Double> {
        var candidates: [SIMD3<Double>] = []
        candidates.reserveCapacity(16)
        for z in [-halfLength, halfLength] {
            for index in 0..<8 {
                let angle = Double(index) * .pi * 0.25
                let local = SIMD3<Double>(cos(angle) * radius, sin(angle) * radius, z)
                candidates.append(center + orientation.act(local))
            }
        }
        return candidates.min { lhs, rhs in lhs.z < rhs.z } ?? center
    }

    private func orientation(fromRPY rpy: KuyuVector3) -> simd_quatd {
        let roll = simd_quatd(angle: rpy.x, axis: SIMD3<Double>(1, 0, 0))
        let pitch = simd_quatd(angle: rpy.y, axis: SIMD3<Double>(0, 1, 0))
        let yaw = simd_quatd(angle: rpy.z, axis: SIMD3<Double>(0, 0, 1))
        return (yaw * pitch * roll).normalizedQuat
    }

    private func simdVector(_ vector: KuyuVector3) -> SIMD3<Double> {
        SIMD3<Double>(vector.x, vector.y, vector.z)
    }

    private func clamp(_ value: Double, lower: Double, upper: Double) -> Double {
        min(max(value, lower), upper)
    }

    private static func ancestorJointIDsByLinkID(topology: ArticulatedSnapshotTopology) -> [String: [String]] {
        var result = Dictionary(uniqueKeysWithValues: topology.rootLinks.map { ($0.id, [String]()) })
        for joint in topology.orderedJoints {
            let parentAncestors = result[joint.parentLinkID] ?? []
            result[joint.childLinkID] = parentAncestors + [joint.id]
        }
        return result
    }

    private static func contactSurfaces(from world: KuyuWorldModel) throws -> [ContactSurface] {
        guard !world.surfaces.isEmpty else {
            throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.surfaces")
        }
        let materialsByID = Dictionary(uniqueKeysWithValues: world.materials.map { ($0.id, $0) })
        return try world.surfaces.sorted(by: { $0.id < $1.id }).map { surface in
            guard surface.frameID == "world" else {
                throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.surface.frame.\(surface.id)")
            }
            guard surface.geometry.kind == .box, let size = surface.geometry.size else {
                throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.surface.geometry.\(surface.id)")
            }
            guard abs(surface.pose.rpy.x) <= 1e-12,
                  abs(surface.pose.rpy.y) <= 1e-12,
                  abs(surface.pose.rpy.z) <= 1e-12 else {
                throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.surface.rotation.\(surface.id)")
            }
            guard abs(surface.geometry.pose.rpy.x) <= 1e-12,
                  abs(surface.geometry.pose.rpy.y) <= 1e-12,
                  abs(surface.geometry.pose.rpy.z) <= 1e-12 else {
                throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld(
                    "contact.surface.geometry.rotation.\(surface.id)"
                )
            }
            guard let material = materialsByID[surface.materialID] else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.surface.material.\(surface.materialID)")
            }
            let scale = surface.geometry.scale ?? KuyuVector3(x: 1, y: 1, z: 1)
            let scaledSize = SIMD3<Double>(
                size.x * scale.x,
                size.y * scale.y,
                size.z * scale.z
            )
            let center = SIMD3<Double>(
                surface.pose.xyz.x + surface.geometry.pose.xyz.x,
                surface.pose.xyz.y + surface.geometry.pose.xyz.y,
                surface.pose.xyz.z + surface.geometry.pose.xyz.z
            )
            return ContactSurface(
                id: surface.id,
                center: center,
                halfExtents: scaledSize * 0.5,
                planeZ: center.z + scaledSize.z * 0.5,
                normal: SIMD3<Double>(0, 0, 1),
                material: ContactMaterial(
                    staticFriction: material.staticFriction,
                    dynamicFriction: material.dynamicFriction,
                    restitution: material.restitution
                )
            )
        }
    }

    private func validateCollisions(body: KuyuBodyModel) throws {
        let collidableLinks = body.links.filter { !$0.collisions.isEmpty }
        guard !collidableLinks.isEmpty else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.collisions")
        }
        for link in collidableLinks {
            _ = try material(for: link)
            for geometry in link.collisions where geometry.kind == .mesh {
                throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.geometry.mesh.\(geometry.id)")
            }
        }
    }
}

private struct ContactSurface: Sendable, Equatable {
    let id: String
    let center: SIMD3<Double>
    let halfExtents: SIMD3<Double>
    let planeZ: Double
    let normal: SIMD3<Double>
    let material: ContactMaterial

    func contains(_ point: SIMD3<Double>) -> Bool {
        abs(point.x - center.x) <= halfExtents.x + 1e-9
            && abs(point.y - center.y) <= halfExtents.y + 1e-9
    }
}

private struct ContactMaterial: Sendable, Equatable {
    let staticFriction: Double
    let dynamicFriction: Double
    let restitution: Double

    func combined(with other: ContactMaterial) -> ContactMaterial {
        ContactMaterial(
            staticFriction: min(staticFriction, other.staticFriction),
            dynamicFriction: min(dynamicFriction, other.dynamicFriction),
            restitution: min(restitution, other.restitution)
        )
    }
}

private struct ArticulatedContact: Sendable, Equatable {
    let linkID: String
    let point: SIMD3<Double>
    let pointVelocity: SIMD3<Double>
    let normal: SIMD3<Double>
    let penetration: Double
    let normalVelocity: Double
    let material: ContactMaterial
    let linkMass: Double
}

private struct ContactEvaluationBatch: Sendable, Equatable {
    let contacts: [ArticulatedContact]
    let evaluation: ArticulatedKinematicEvaluation
}

private struct CollisionWitness: Sendable, Equatable {
    let point: SIMD3<Double>
    let velocity: SIMD3<Double>
}

private struct ContactJacobianEntry: Sendable, Equatable {
    let index: Int
    let linear: SIMD3<Double>
}
