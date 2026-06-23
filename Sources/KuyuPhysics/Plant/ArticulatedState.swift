import simd

struct ArticulatedState: Sendable, Equatable {
    var position: [Double]
    var velocity: [Double]
    var torque: [Double]
}

struct ArticulatedStateStepResult: Sendable, Equatable {
    var state: ArticulatedState
    var contactMetrics: ArticulatedContactMetrics
}

struct ArticulatedContactMetrics: Sendable, Equatable {
    var activeContacts: Int
    var maxPenetration: Double
    var maxNormalImpulse: Double
    var maxNormalForce: Double
    var solverIterations: Int

    static let disabled = ArticulatedContactMetrics(
        activeContacts: 0,
        maxPenetration: 0,
        maxNormalImpulse: 0,
        maxNormalForce: 0,
        solverIterations: 0
    )

    mutating func merge(_ other: ArticulatedContactMetrics) {
        activeContacts = max(activeContacts, other.activeContacts)
        maxPenetration = max(maxPenetration, other.maxPenetration)
        maxNormalImpulse = max(maxNormalImpulse, other.maxNormalImpulse)
        maxNormalForce = max(maxNormalForce, other.maxNormalForce)
        solverIterations = max(solverIterations, other.solverIterations)
    }
}

struct ArticulatedJointBinding: Sendable, Equatable {
    let joint: JointDefinition
    let attachment: ActuatorAttachment
    let actuator: ActuatorDefinition
    let actuatorSignal: SignalDefinition
}

struct ArticulatedJointDynamics: Sendable, Equatable {
    let joint: JointDefinition
    let effectiveInertia: Double
    let effortLimit: Double
    let stiffness: Double
    let servoDamping: Double
    let damping: Double
    let coulombFriction: Double
    let gravityMassLever: Double
    let hasGravityTorque: Bool
}

struct ArticulatedChildLinks: Sendable, Equatable {
    var ids: [String]
}

struct ArticulatedSnapshotTopology: Sendable, Equatable {
    let rootLinks: [LinkDefinition]
    let orderedJoints: [JointDefinition]
}

struct ArticulatedLinkKinematicState: Sendable, Equatable {
    var position: SIMD3<Double>
    var velocity: SIMD3<Double>
    var orientation: simd_quatd
    var angularVelocity: SIMD3<Double>

    static let identity = ArticulatedLinkKinematicState(
        position: SIMD3<Double>(repeating: 0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
}

struct ArticulatedJointKinematicState: Sendable, Equatable {
    let joint: JointDefinition
    let originPosition: SIMD3<Double>
    let originVelocity: SIMD3<Double>
    let originOrientation: simd_quatd
    let axisWorld: SIMD3<Double>
    let childLinkState: ArticulatedLinkKinematicState
}

struct ArticulatedKinematicEvaluation: Sendable, Equatable {
    let linksByID: [String: ArticulatedLinkKinematicState]
    let jointsByID: [String: ArticulatedJointKinematicState]
}
