import Foundation

public struct HardwareCalibrationPlan: Sendable, Codable, Equatable {
    public let schemaVersion: String
    public let planID: String
    public let generatedAt: String?
    public let robotID: String
    public let bodyID: String
    public let embodimentContractID: String
    public let safetyJointLimits: [JointLimitDeclaration]
    public let measurementRequirements: [String]
    public let steps: [HardwareCalibrationStep]

    public init(
        schemaVersion: String,
        planID: String,
        generatedAt: String? = nil,
        robotID: String,
        bodyID: String,
        embodimentContractID: String,
        safetyJointLimits: [JointLimitDeclaration],
        measurementRequirements: [String],
        steps: [HardwareCalibrationStep]
    ) {
        self.schemaVersion = schemaVersion
        self.planID = planID
        self.generatedAt = generatedAt
        self.robotID = robotID
        self.bodyID = bodyID
        self.embodimentContractID = embodimentContractID
        self.safetyJointLimits = safetyJointLimits
        self.measurementRequirements = measurementRequirements
        self.steps = steps
    }
}

public struct JointLimitDeclaration: Sendable, Codable, Equatable {
    public let jointID: String
    public let actuatorID: String
    public let lowerRadians: Double
    public let upperRadians: Double

    public init(jointID: String, actuatorID: String, lowerRadians: Double, upperRadians: Double) {
        self.jointID = jointID
        self.actuatorID = actuatorID
        self.lowerRadians = lowerRadians
        self.upperRadians = upperRadians
    }
}

public struct HardwareCalibrationStep: Sendable, Codable, Equatable {
    public let stepID: String
    public let commandTimeSeconds: Double
    public let holdSeconds: Double
    public let jointTargets: [JointTargetDeclaration]
    public let commandPulses: [Int]
    public let commandPayload: String

    public init(
        stepID: String,
        commandTimeSeconds: Double,
        holdSeconds: Double,
        jointTargets: [JointTargetDeclaration],
        commandPulses: [Int],
        commandPayload: String
    ) {
        self.stepID = stepID
        self.commandTimeSeconds = commandTimeSeconds
        self.holdSeconds = holdSeconds
        self.jointTargets = jointTargets
        self.commandPulses = commandPulses
        self.commandPayload = commandPayload
    }
}

public struct JointTargetDeclaration: Sendable, Codable, Equatable {
    public let jointID: String
    public let actuatorID: String
    public let targetRadians: Double

    public init(jointID: String, actuatorID: String, targetRadians: Double) {
        self.jointID = jointID
        self.actuatorID = actuatorID
        self.targetRadians = targetRadians
    }
}

public struct RoArmM1HardwareCalibrationPlanBuilder: Sendable {
    public enum PlanError: Error, Equatable {
        case invalidRobot(String)
        case invalidParameter(String)
        case payloadEncoding
    }

    public init() {}

    public func build(
        robot: LoadedKuyuRobot,
        jointLimits: [ClosedRange<Double>],
        speed: Int,
        acceleration: Int,
        amplitudeRadians: Double,
        holdSeconds: Double,
        repetitions: Int
    ) throws -> HardwareCalibrationPlan {
        guard robot.manifest.robotID == "roarm-m1-v0" else {
            throw PlanError.invalidRobot(robot.manifest.robotID)
        }
        guard jointLimits.count == RoArmM1ServoCommandEncoder.jointCount else {
            throw PlanError.invalidParameter("jointLimits")
        }
        guard amplitudeRadians.isFinite, amplitudeRadians > 0 else {
            throw PlanError.invalidParameter("amplitudeRadians")
        }
        guard holdSeconds.isFinite, holdSeconds > 0 else {
            throw PlanError.invalidParameter("holdSeconds")
        }
        guard repetitions > 0 else {
            throw PlanError.invalidParameter("repetitions")
        }

        let activeJoints = robot.body.joints.filter { joint in
            joint.mimic == nil && (joint.kind == .revolute || joint.kind == .continuous || joint.kind == .prismatic)
        }
        guard activeJoints.count == RoArmM1ServoCommandEncoder.jointCount else {
            throw PlanError.invalidRobot("activeJointCount")
        }
        let attachmentsByJoint = Dictionary(uniqueKeysWithValues: robot.body.actuatorAttachments.map { ($0.jointID, $0) })
        let encoder = try RoArmM1ServoCommandEncoder(
            jointLimits: jointLimits,
            speed: speed,
            acceleration: acceleration
        )

        var steps: [HardwareCalibrationStep] = []
        var currentTime = 0.0
        let home = Array(repeating: 0.0, count: RoArmM1ServoCommandEncoder.jointCount)
        try appendStep(
            id: "home-initial",
            targets: home,
            time: currentTime,
            holdSeconds: holdSeconds,
            activeJoints: activeJoints,
            attachmentsByJoint: attachmentsByJoint,
            encoder: encoder,
            steps: &steps
        )
        currentTime += holdSeconds

        for repetition in 0..<repetitions {
            for jointIndex in 0..<RoArmM1ServoCommandEncoder.jointCount {
                let limit = jointLimits[jointIndex]
                let positive = min(amplitudeRadians, limit.upperBound)
                if positive > 0 {
                    var target = home
                    target[jointIndex] = positive
                    try appendStep(
                        id: "joint-\(jointIndex + 1)-positive-r\(repetition + 1)",
                        targets: target,
                        time: currentTime,
                        holdSeconds: holdSeconds,
                        activeJoints: activeJoints,
                        attachmentsByJoint: attachmentsByJoint,
                        encoder: encoder,
                        steps: &steps
                    )
                    currentTime += holdSeconds
                }

                try appendStep(
                    id: "joint-\(jointIndex + 1)-home-after-positive-r\(repetition + 1)",
                    targets: home,
                    time: currentTime,
                    holdSeconds: holdSeconds,
                    activeJoints: activeJoints,
                    attachmentsByJoint: attachmentsByJoint,
                    encoder: encoder,
                    steps: &steps
                )
                currentTime += holdSeconds

                let negative = max(-amplitudeRadians, limit.lowerBound)
                if negative < 0 {
                    var target = home
                    target[jointIndex] = negative
                    try appendStep(
                        id: "joint-\(jointIndex + 1)-negative-r\(repetition + 1)",
                        targets: target,
                        time: currentTime,
                        holdSeconds: holdSeconds,
                        activeJoints: activeJoints,
                        attachmentsByJoint: attachmentsByJoint,
                        encoder: encoder,
                        steps: &steps
                    )
                    currentTime += holdSeconds
                }

                try appendStep(
                    id: "joint-\(jointIndex + 1)-home-after-negative-r\(repetition + 1)",
                    targets: home,
                    time: currentTime,
                    holdSeconds: holdSeconds,
                    activeJoints: activeJoints,
                    attachmentsByJoint: attachmentsByJoint,
                    encoder: encoder,
                    steps: &steps
                )
                currentTime += holdSeconds
            }
        }

        let limits = try activeJoints.enumerated().map { index, joint in
            guard let attachment = attachmentsByJoint[joint.id] else {
                throw PlanError.invalidRobot("attachment.\(joint.id)")
            }
            let limit = jointLimits[index]
            return JointLimitDeclaration(
                jointID: joint.id,
                actuatorID: attachment.actuatorID,
                lowerRadians: limit.lowerBound,
                upperRadians: limit.upperBound
            )
        }

        return HardwareCalibrationPlan(
            schemaVersion: "kuyu.hardware-calibration-plan.v1",
            planID: "roarm-m1-hardware-parity-plan-v1",
            generatedAt: Self.timestampString(),
            robotID: robot.manifest.robotID,
            bodyID: robot.body.bodyID,
            embodimentContractID: robot.embodiment.contractID,
            safetyJointLimits: limits,
            measurementRequirements: [
                "measuredPositionRadians",
                "observedTimeSeconds",
                "busVoltageVolts",
                "servoTemperatureCelsius",
                "loadEstimate"
            ],
            steps: steps
        )
    }

    private func appendStep(
        id: String,
        targets: [Double],
        time: Double,
        holdSeconds: Double,
        activeJoints: [JointDefinition],
        attachmentsByJoint: [String: ActuatorAttachment],
        encoder: RoArmM1ServoCommandEncoder,
        steps: inout [HardwareCalibrationStep]
    ) throws {
        let command = try encoder.command(forRadians: targets)
        let data = try encoder.commandData(forRadians: targets)
        guard let payload = String(data: data, encoding: .utf8) else {
            throw PlanError.payloadEncoding
        }
        let declarations = try activeJoints.enumerated().map { index, joint in
            guard let attachment = attachmentsByJoint[joint.id] else {
                throw PlanError.invalidRobot("attachment.\(joint.id)")
            }
            return JointTargetDeclaration(
                jointID: joint.id,
                actuatorID: attachment.actuatorID,
                targetRadians: targets[index]
            )
        }
        steps.append(HardwareCalibrationStep(
            stepID: id,
            commandTimeSeconds: time,
            holdSeconds: holdSeconds,
            jointTargets: declarations,
            commandPulses: command.positions,
            commandPayload: payload
        ))
    }

    private static func timestampString() -> String {
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        return formatter.string(from: Date())
    }
}
