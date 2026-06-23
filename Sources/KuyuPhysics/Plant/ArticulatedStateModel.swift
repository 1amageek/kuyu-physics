import Foundation

struct ArticulatedStateModel: Sendable {
    private let world: KuyuWorldModel
    private let linkByID: [String: LinkDefinition]
    private let childLinksByParent: [String: ArticulatedChildLinks]

    init(body: KuyuBodyModel, world: KuyuWorldModel) {
        self.world = world
        var linkIndex: [String: LinkDefinition] = [:]
        for link in body.links {
            linkIndex[link.id] = link
        }
        self.linkByID = linkIndex
        var children: [String: ArticulatedChildLinks] = [:]
        for joint in body.joints {
            var childLinks = children[joint.parentLinkID] ?? ArticulatedChildLinks(ids: [])
            childLinks.ids.append(joint.childLinkID)
            children[joint.parentLinkID] = childLinks
        }
        self.childLinksByParent = children
    }

    func dynamics(for bindings: [ArticulatedJointBinding]) throws -> [ArticulatedJointDynamics] {
        var result: [ArticulatedJointDynamics] = []
        result.reserveCapacity(bindings.count)
        for binding in bindings {
            let effectiveInertia = inertia(for: binding)
            let effortLimit = effortLimit(for: binding)
            let timeConstant = binding.actuator.dynamics?.timeConstantSeconds ?? 0.001
            let stiffness = effectiveInertia / (timeConstant * timeConstant)
            let servoDamping = 2.0 * sqrt(max(stiffness * effectiveInertia, 0.0))
            let damping = binding.joint.damping + (binding.actuator.dynamics?.damping ?? 0)
            let coulombFriction = binding.joint.coulombFriction + (binding.actuator.dynamics?.coulombFriction ?? 0)
            let gravityMassLever = gravityMassLever(for: binding.joint)
            let hasGravityTorque = abs(binding.joint.axis.y) > 0 || abs(binding.joint.axis.x) > 0

            try ensureFinite(effectiveInertia, "inertia.\(binding.joint.id)")
            try ensureFinite(effortLimit, "effortLimit.\(binding.joint.id)")
            try ensureFinite(stiffness, "stiffness.\(binding.joint.id)")
            try ensureFinite(servoDamping, "servoDamping.\(binding.joint.id)")
            try ensureFinite(damping, "damping.\(binding.joint.id)")
            try ensureFinite(coulombFriction, "coulombFriction.\(binding.joint.id)")
            try ensureFinite(gravityMassLever, "gravityMassLever.\(binding.joint.id)")

            result.append(ArticulatedJointDynamics(
                joint: binding.joint,
                effectiveInertia: effectiveInertia,
                effortLimit: effortLimit,
                stiffness: stiffness,
                servoDamping: servoDamping,
                damping: damping,
                coulombFriction: coulombFriction,
                gravityMassLever: gravityMassLever,
                hasGravityTorque: hasGravityTorque
            ))
        }
        return result
    }

    func step(
        state: ArticulatedState,
        targets: [Double],
        dynamics: [ArticulatedJointDynamics],
        deltaTime: Double,
        contactSolver: ArticulatedContactSolver?
    ) throws -> ArticulatedStateStepResult {
        var next = state
        var contactMetrics = ArticulatedContactMetrics.disabled
        try ensureFinite(deltaTime, "deltaTime")
        guard deltaTime > 0 else {
            throw ArticulatedRigidBodySimulator.SimulationError.nonFiniteState("deltaTime")
        }
        let substeps = max(world.time.substeps, 1)
        let substepDelta = deltaTime / Double(substeps)
        var substep = 0
        while substep < substeps {
            let integrated = try integrateSubstep(
                state: next,
                targets: targets,
                dynamics: dynamics,
                deltaTime: substepDelta,
                contactSolver: contactSolver
            )
            next = integrated.state
            contactMetrics.merge(integrated.contactMetrics)
            if let contactSolver = contactSolver {
                let projected = try contactSolver.project(state: next, deltaTime: substepDelta)
                next = projected.state
                contactMetrics.merge(projected.contactMetrics)
            }
            substep += 1
        }
        return ArticulatedStateStepResult(state: next, contactMetrics: contactMetrics)
    }

    private func integrateSubstep(
        state: ArticulatedState,
        targets: [Double],
        dynamics: [ArticulatedJointDynamics],
        deltaTime: Double,
        contactSolver: ArticulatedContactSolver?
    ) throws -> ArticulatedStateStepResult {
        var next = state
        guard dynamics.count == state.position.count else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("joint-dynamics-count")
        }
        let contactTorques: [Double]
        var contactMetrics = ArticulatedContactMetrics.disabled
        if let contactSolver = contactSolver {
            let result = try contactSolver.contactTorques(state: state, deltaTime: deltaTime)
            contactTorques = result.0
            contactMetrics.merge(result.1)
        } else {
            contactTorques = Array(repeating: 0.0, count: dynamics.count)
        }
        for index in dynamics.indices {
            let jointDynamics = dynamics[index]
            let joint = jointDynamics.joint
            let current = state.position[index]
            let velocity = state.velocity[index]
            let target = targets.indices.contains(index) ? targets[index] : current
            try ensureFinite(current, "position[\(index)]")
            try ensureFinite(velocity, "velocity[\(index)]")
            try ensureFinite(target, "target[\(index)]")
            let gravityTorque = gravityTorque(for: jointDynamics, position: current)
            let friction = jointDynamics.coulombFriction * sign(velocity)
            let springTorque = jointDynamics.stiffness * (target - current)
            let servoDampingTorque = jointDynamics.servoDamping * velocity
            let jointDampingTorque = jointDynamics.damping * velocity
            let dampingTorque = servoDampingTorque + jointDampingTorque + friction
            let rawServoTorque = springTorque + gravityTorque - dampingTorque
            let servoTorque = clamp(rawServoTorque, lower: -jointDynamics.effortLimit, upper: jointDynamics.effortLimit)
            let torque = servoTorque + contactTorques[index]
            let acceleration = torque / jointDynamics.effectiveInertia
            let limitedVelocity = velocityLimit(for: joint, proposed: velocity + acceleration * deltaTime)
            var proposedPosition = current + limitedVelocity * deltaTime
            var proposedVelocity = limitedVelocity
            if let lower = joint.lowerLimit, proposedPosition < lower {
                proposedPosition = lower
                proposedVelocity = 0
            }
            if let upper = joint.upperLimit, proposedPosition > upper {
                proposedPosition = upper
                proposedVelocity = 0
            }
            try ensureFinite(torque, "torque[\(index)]")
            try ensureFinite(acceleration, "acceleration[\(index)]")
            try ensureFinite(proposedPosition, "position.next[\(index)]")
            try ensureFinite(proposedVelocity, "velocity.next[\(index)]")
            next.position[index] = proposedPosition
            next.velocity[index] = proposedVelocity
            next.torque[index] = torque
        }
        return ArticulatedStateStepResult(state: next, contactMetrics: contactMetrics)
    }

    private func inertia(for joint: JointDefinition) -> Double {
        let descendants = descendantLinks(from: joint.childLinkID)
        if joint.kind == .prismatic {
            let mass = descendants.reduce(0.0) { $0 + $1.mass }
            return max(mass, 1e-6)
        }
        let axis = joint.axis
        var total = 0.0
        for link in descendants {
            let xInertia = abs(axis.x) * link.inertia.ixx
            let yInertia = abs(axis.y) * link.inertia.iyy
            let zInertia = abs(axis.z) * link.inertia.izz
            let rotational = xInertia + yInertia + zInertia
            let leverVector = KuyuVector3(
                x: joint.origin.xyz.x + link.centerOfMass.x,
                y: joint.origin.xyz.y + link.centerOfMass.y,
                z: joint.origin.xyz.z + link.centerOfMass.z
            )
            let lever = perpendicularLever(axis: axis, vector: leverVector)
            total += rotational + link.mass * lever * lever
        }
        return max(total, 1e-6)
    }

    private func inertia(for binding: ArticulatedJointBinding) -> Double {
        inertia(for: binding.joint) + (binding.attachment.reflectedInertia ?? 0)
    }

    private func effortLimit(for binding: ArticulatedJointBinding) -> Double {
        let joint = binding.joint
        let jointLimit = joint.effortLimit ?? .greatestFiniteMagnitude
        let attachmentLimit = binding.attachment.torqueLimit
        let actuatorLimit = binding.actuator.dynamics?.torqueLimit ?? .greatestFiniteMagnitude
        let reduction = binding.attachment.mechanicalReductionRatio
        let efficiency = binding.attachment.efficiency ?? 1.0
        let transmissionLimit = actuatorLimit * reduction * efficiency
        return max(min(jointLimit, attachmentLimit, transmissionLimit), 0)
    }

    private func gravityTorque(for dynamics: ArticulatedJointDynamics, position: Double) -> Double {
        guard dynamics.hasGravityTorque else { return 0 }
        let g = abs(world.gravity.acceleration.z)
        return -dynamics.gravityMassLever * g * sin(position)
    }

    private func gravityMassLever(for joint: JointDefinition) -> Double {
        var massLever = 0.0
        for link in descendantLinks(from: joint.childLinkID) {
            let lever = max(abs(joint.origin.xyz.x + link.centerOfMass.x), 0.01)
            massLever += link.mass * lever
        }
        return massLever
    }

    private func descendantLinks(from rootLinkID: String) -> [LinkDefinition] {
        guard let root = linkByID[rootLinkID] else {
            return []
        }
        var result = [root]
        let childIDs = childLinksByParent[rootLinkID]?.ids ?? []
        for childID in childIDs {
            result.append(contentsOf: descendantLinks(from: childID))
        }
        return result
    }

    private func perpendicularLever(axis: KuyuVector3, vector: KuyuVector3) -> Double {
        let axisLength = sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z)
        guard axisLength > 0 else { return 0.01 }
        let ax = axis.x / axisLength
        let ay = axis.y / axisLength
        let az = axis.z / axisLength
        let dot = vector.x * ax + vector.y * ay + vector.z * az
        let px = vector.x - dot * ax
        let py = vector.y - dot * ay
        let pz = vector.z - dot * az
        return max(sqrt(px * px + py * py + pz * pz), 0.01)
    }

    private func velocityLimit(for joint: JointDefinition, proposed: Double) -> Double {
        guard let limit = joint.velocityLimit else { return proposed }
        return clamp(proposed, lower: -limit, upper: limit)
    }

    private func sign(_ value: Double) -> Double {
        if value > 0 { return 1 }
        if value < 0 { return -1 }
        return 0
    }

    private func clamp(_ value: Double, lower: Double, upper: Double) -> Double {
        Swift.min(Swift.max(value, lower), upper)
    }

    private func ensureFinite(_ value: Double, _ field: String) throws {
        if !value.isFinite {
            throw ArticulatedRigidBodySimulator.SimulationError.nonFiniteState(field)
        }
    }
}
