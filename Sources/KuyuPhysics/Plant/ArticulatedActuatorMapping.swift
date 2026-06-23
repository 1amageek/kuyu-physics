struct ArticulatedActuatorMapping: Sendable, Equatable {
    static func jointPosition(
        actuatorPosition: Double,
        attachment: ActuatorAttachment
    ) -> Double {
        let centeredActuator = actuatorPosition - attachment.actuatorZeroOffset
        let directedJoint = attachment.commandDirection * centeredActuator / attachment.transmissionRatio
        return attachment.jointZeroOffset + directedJoint
    }

    static func actuatorPosition(
        jointPosition: Double,
        attachment: ActuatorAttachment
    ) -> Double {
        let centeredJoint = jointPosition - attachment.jointZeroOffset
        let directedActuator = attachment.commandDirection * centeredJoint * attachment.transmissionRatio
        return attachment.actuatorZeroOffset + directedActuator
    }

    static func actuatorVelocity(
        jointVelocity: Double,
        attachment: ActuatorAttachment
    ) -> Double {
        attachment.commandDirection * jointVelocity * attachment.transmissionRatio
    }

    static func actuatorTorque(
        jointTorque: Double,
        attachment: ActuatorAttachment
    ) -> Double {
        let torqueScale = attachment.mechanicalReductionRatio * (attachment.efficiency ?? 1.0)
        return attachment.commandDirection * jointTorque / torqueScale
    }

    static func jointRange(
        actuatorLimits: ActuatorLimits,
        attachment: ActuatorAttachment
    ) -> ClosedRange<Double> {
        let lower = jointPosition(actuatorPosition: actuatorLimits.min, attachment: attachment)
        let upper = jointPosition(actuatorPosition: actuatorLimits.max, attachment: attachment)
        return min(lower, upper)...max(lower, upper)
    }
}
