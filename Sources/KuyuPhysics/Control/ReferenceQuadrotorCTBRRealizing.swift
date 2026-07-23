import KuyuCore

public protocol ReferenceQuadrotorCTBRRealizing: Sendable {
    func motorCommand(
        for command: ReferenceQuadrotorCTBRCommand,
        currentAngularVelocity: Axis3,
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer
    ) throws -> ReferenceQuadrotorNormalizedMotorCommand
}
