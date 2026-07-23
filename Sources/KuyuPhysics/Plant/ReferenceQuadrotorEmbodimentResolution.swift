public struct ReferenceQuadrotorEmbodimentResolution: Sendable, Equatable {
    public let parameters: ReferenceQuadrotorParameters
    public let normalizedActuatorRateLimitPerSecond: Double

    public init(
        parameters: ReferenceQuadrotorParameters,
        normalizedActuatorRateLimitPerSecond: Double
    ) {
        self.parameters = parameters
        self.normalizedActuatorRateLimitPerSecond = normalizedActuatorRateLimitPerSecond
    }
}
