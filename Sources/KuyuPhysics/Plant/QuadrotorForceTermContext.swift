import KuyuCore

public struct QuadrotorForceTermContext: Sendable {
    public let state: ReferenceQuadrotorState
    public let motorThrusts: MotorThrusts
    public let disturbances: DisturbanceState
    public let parameters: ReferenceQuadrotorParameters
    public let mixer: ReferenceQuadrotorMixer
    public let environment: WorldEnvironment

    public init(
        state: ReferenceQuadrotorState,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState,
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer,
        environment: WorldEnvironment
    ) {
        self.state = state
        self.motorThrusts = motorThrusts
        self.disturbances = disturbances
        self.parameters = parameters
        self.mixer = mixer
        self.environment = environment
    }

    public var effectiveGravity: Double {
        environment.effectiveGravity(defaultGravity: parameters.gravity)
    }
}
