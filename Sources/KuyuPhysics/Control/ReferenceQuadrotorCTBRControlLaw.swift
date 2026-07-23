import KuyuCore

public struct ReferenceQuadrotorCTBRControlLaw: ReferenceQuadrotorCTBRRealizing, Sendable {
    public enum ControlError: Error, Equatable {
        case nonFiniteAngularVelocity
        case unsupportedYawCoefficient(Double)
    }

    public let config: ReferenceQuadrotorCTBRControlConfig

    public init(config: ReferenceQuadrotorCTBRControlConfig = .canonical) {
        self.config = config
    }

    public func motorCommand(
        for command: ReferenceQuadrotorCTBRCommand,
        currentAngularVelocity: Axis3,
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer
    ) throws -> ReferenceQuadrotorNormalizedMotorCommand {
        guard currentAngularVelocity.isFinite else {
            throw ControlError.nonFiniteAngularVelocity
        }
        guard parameters.yawCoefficient > 0 else {
            throw ControlError.unsupportedYawCoefficient(parameters.yawCoefficient)
        }

        let targetRate = Axis3(
            x: command.normalizedBodyRate.x * config.bodyRateScale.x,
            y: command.normalizedBodyRate.y * config.bodyRateScale.y,
            z: command.normalizedBodyRate.z * config.bodyRateScale.z
        )
        let torque = Axis3(
            x: (targetRate.x - currentAngularVelocity.x) * config.torqueGain.x,
            y: (targetRate.y - currentAngularVelocity.y) * config.torqueGain.y,
            z: (targetRate.z - currentAngularVelocity.z) * config.torqueGain.z
        )
        let baseThrust = command.collectiveThrust * parameters.maxThrust
        let rollTerm = torque.x / (2.0 * parameters.armLength)
        let pitchTerm = torque.y / (2.0 * parameters.armLength)
        let yawTerm = torque.z / (4.0 * parameters.yawCoefficient)
        let spin = mixer.spinDirections

        return try ReferenceQuadrotorNormalizedMotorCommand(
            f1: normalized(baseThrust - pitchTerm + spin.x * yawTerm, maximum: parameters.maxThrust),
            f2: normalized(baseThrust + rollTerm + spin.y * yawTerm, maximum: parameters.maxThrust),
            f3: normalized(baseThrust + pitchTerm + spin.z * yawTerm, maximum: parameters.maxThrust),
            f4: normalized(baseThrust - rollTerm + spin.w * yawTerm, maximum: parameters.maxThrust)
        )
    }

    private func normalized(_ thrust: Double, maximum: Double) -> Double {
        min(max(thrust / maximum, 0.0), 1.0)
    }
}
