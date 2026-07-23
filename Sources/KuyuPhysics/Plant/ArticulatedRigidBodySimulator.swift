public struct ArticulatedRigidBodySimulator: Sendable {
    public enum SimulationError: Error, Equatable {
        case invalidDuration(Double)
        case durationStepMismatch(duration: Double, timeStep: Double)
        case timeStepMismatch(request: Double, world: Double)
        case invalidBody(String)
        case invalidDriveProviderOutput(String)
        case missingJointRange(String)
        case nonFiniteState(String)
        case readinessFailed(String)
        case unstableTimeStep(substep: Double, timeConstant: Double, actuator: String)
        case unresolvedContact(maxPenetration: Double, tolerance: Double)
        case unsupportedWorld(String)
    }

    public init() {}
}
