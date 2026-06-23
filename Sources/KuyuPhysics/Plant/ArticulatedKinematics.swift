import KuyuCore
import simd

struct ArticulatedKinematics: Sendable {
    let topology: ArticulatedSnapshotTopology

    func evaluate(scalars: [String: Double]) throws -> ArticulatedKinematicEvaluation {
        guard !topology.rootLinks.isEmpty else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("root-link")
        }
        var linksByID = Dictionary(
            uniqueKeysWithValues: topology.rootLinks.map { ($0.id, ArticulatedLinkKinematicState.identity) }
        )
        var jointsByID: [String: ArticulatedJointKinematicState] = [:]
        for joint in topology.orderedJoints {
            guard let parentState = linksByID[joint.parentLinkID] else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("joint-topology.\(joint.id)")
            }
            let scalar = scalars[joint.id] ?? joint.homePosition ?? 0
            let velocity = scalars["velocity_\(joint.id)"] ?? 0
            let originTranslation = simdVector(joint.origin.xyz)
            let originWorldOffset = parentState.orientation.act(originTranslation)
            let originPosition = parentState.position + originWorldOffset
            let originVelocity = parentState.velocity + simd_cross(parentState.angularVelocity, originWorldOffset)
            let originOrientation = (parentState.orientation * orientation(fromRPY: joint.origin.rpy)).normalizedQuat
            let axisWorld: SIMD3<Double>
            let childState: ArticulatedLinkKinematicState
            switch joint.kind {
            case .fixed:
                axisWorld = SIMD3<Double>(repeating: 0)
                childState = ArticulatedLinkKinematicState(
                    position: originPosition,
                    velocity: originVelocity,
                    orientation: originOrientation,
                    angularVelocity: parentState.angularVelocity
                )
            case .revolute, .continuous:
                let axis = try normalizedAxis(joint)
                axisWorld = originOrientation.act(axis)
                childState = ArticulatedLinkKinematicState(
                    position: originPosition,
                    velocity: originVelocity,
                    orientation: (originOrientation * simd_quatd(angle: scalar, axis: axis)).normalizedQuat,
                    angularVelocity: parentState.angularVelocity + axisWorld * velocity
                )
            case .prismatic:
                let axis = try normalizedAxis(joint)
                axisWorld = originOrientation.act(axis)
                let displacement = axisWorld * scalar
                childState = ArticulatedLinkKinematicState(
                    position: originPosition + displacement,
                    velocity: originVelocity + simd_cross(parentState.angularVelocity, displacement) + axisWorld * velocity,
                    orientation: originOrientation,
                    angularVelocity: parentState.angularVelocity
                )
            }
            linksByID[joint.childLinkID] = childState
            jointsByID[joint.id] = ArticulatedJointKinematicState(
                joint: joint,
                originPosition: originPosition,
                originVelocity: originVelocity,
                originOrientation: originOrientation,
                axisWorld: axisWorld,
                childLinkState: childState
            )
        }
        return ArticulatedKinematicEvaluation(linksByID: linksByID, jointsByID: jointsByID)
    }

    func snapshots(scalars: [String: Double]) throws -> [RigidBodySnapshot] {
        let evaluation = try evaluate(scalars: scalars)
        var snapshots: [RigidBodySnapshot] = []
        snapshots.reserveCapacity(topology.orderedJoints.count)
        for joint in topology.orderedJoints {
            guard let state = evaluation.linksByID[joint.childLinkID] else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("joint-topology.\(joint.id)")
            }
            snapshots.append(RigidBodySnapshot(
                id: joint.childLinkID,
                position: axis3(state.position),
                velocity: axis3(state.velocity),
                orientation: QuaternionSnapshot(orientation: state.orientation),
                angularVelocity: axis3(state.angularVelocity)
            ))
        }
        return snapshots
    }

    private func normalizedAxis(_ joint: JointDefinition) throws -> SIMD3<Double> {
        let axis = simdVector(joint.axis)
        let length = simd_length(axis)
        guard length > 0 else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("joint-axis.\(joint.id)")
        }
        return axis / length
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

    private func axis3(_ vector: SIMD3<Double>) -> Axis3 {
        Axis3(x: vector.x, y: vector.y, z: vector.z)
    }
}
