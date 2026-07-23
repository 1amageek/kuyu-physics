import EmbodimentContract
import Foundation

public extension HardwareCalibrationReport {
    static let strictHardwareParityPositionToleranceRadians = 0.05

    func validate(body: KuyuBodyModel, embodiment: EmbodimentContract) throws {
        try validateIdentity(body: body, embodiment: embodiment)
        try validateJointEvidence(body: body)
        try validateSensorEvidence(embodiment: embodiment)
        try validateContactEvidence(body: body)
    }

    func validateHardwareParity(body: KuyuBodyModel, embodiment: EmbodimentContract) throws {
        try validate(body: body, embodiment: embodiment)
        if readinessLevel < .hardwareParity {
            throw HardwareCalibrationValidationError.insufficientCoverage("readinessLevel.\(readinessLevel.rawValue)")
        }
        try ensurePositive(positionToleranceRadians, "positionToleranceRadians")
        if positionToleranceRadians > Self.strictHardwareParityPositionToleranceRadians {
            throw HardwareCalibrationValidationError.invalidRange("positionToleranceRadians")
        }
        if minimumSamplesPerJoint < 3 {
            throw HardwareCalibrationValidationError.invalidRange("minimumSamplesPerJoint")
        }
        if minimumSamplesPerSensor < 3 {
            throw HardwareCalibrationValidationError.invalidRange("minimumSamplesPerSensor")
        }

        let activeJointIDs = Set(activeJoints(body: body).map(\.id))
        let calibratedJointIDs = Set(jointCalibrations.map(\.jointID))
        if activeJointIDs != calibratedJointIDs {
            throw HardwareCalibrationValidationError.insufficientCoverage("jointCalibrations.activeJointCoverage")
        }

        for calibration in jointCalibrations {
            if calibration.samples.count < minimumSamplesPerJoint {
                throw HardwareCalibrationValidationError.insufficientCoverage(
                    "jointCalibrations.\(calibration.jointID).samples"
                )
            }
            if calibration.identifiedDynamics.meanAbsoluteErrorRadians > positionToleranceRadians {
                throw HardwareCalibrationValidationError.invalidRange(
                    "jointCalibrations.\(calibration.jointID).meanAbsoluteErrorRadians"
                )
            }
            if calibration.identifiedDynamics.maxObservedErrorRadians > positionToleranceRadians {
                throw HardwareCalibrationValidationError.invalidRange(
                    "jointCalibrations.\(calibration.jointID).maxObservedErrorRadians"
                )
            }
            for sample in calibration.samples where sample.measuredPositionRadians == nil {
                throw HardwareCalibrationValidationError.missingMeasuredEvidence(
                    "jointCalibrations.\(calibration.jointID).measuredPositionRadians"
                )
            }
            for sample in calibration.samples {
                guard let observedTimeSeconds = sample.observedTimeSeconds else {
                    throw HardwareCalibrationValidationError.missingMeasuredEvidence(
                        "jointCalibrations.\(calibration.jointID).observedTimeSeconds"
                    )
                }
                if observedTimeSeconds < sample.commandTimeSeconds {
                    throw HardwareCalibrationValidationError.invalidRange(
                        "jointCalibrations.\(calibration.jointID).observedTimeSeconds"
                    )
                }
            }
        }

        let sensorIDs = Set(embodiment.sensors.map(\.id))
        let calibratedSensorIDs = Set(sensorCalibrations.map(\.sensorID))
        if !sensorIDs.isEmpty && sensorIDs != calibratedSensorIDs {
            throw HardwareCalibrationValidationError.insufficientCoverage("sensorCalibrations.activeSensorCoverage")
        }
        let sensorsByID = Dictionary(uniqueKeysWithValues: embodiment.sensors.map { ($0.id, $0) })
        for calibration in sensorCalibrations {
            guard let sensor = sensorsByID[calibration.sensorID] else { continue }
            let requiredSampleCount = sensor.channels.count * minimumSamplesPerSensor
            if calibration.samples.count < requiredSampleCount {
                throw HardwareCalibrationValidationError.insufficientCoverage(
                    "sensorCalibrations.\(calibration.sensorID).samples"
                )
            }
            for channelID in sensor.channels {
                let channelSampleCount = calibration.samples.filter { $0.channelID == channelID }.count
                if channelSampleCount < minimumSamplesPerSensor {
                    throw HardwareCalibrationValidationError.insufficientCoverage(
                        "sensorCalibrations.\(calibration.sensorID).samples.\(channelID)"
                    )
                }
            }
            for sample in calibration.samples {
                if sample.observedTimeSeconds < sample.stimulusTimeSeconds {
                    throw HardwareCalibrationValidationError.invalidRange(
                        "sensorCalibrations.\(calibration.sensorID).samples.observedTimeSeconds"
                    )
                }
                let observedLatencySeconds = sample.observedTimeSeconds - sample.stimulusTimeSeconds
                if observedLatencySeconds > calibration.measuredLatencySeconds + calibration.latencyToleranceSeconds {
                    throw HardwareCalibrationValidationError.invalidRange(
                        "sensorCalibrations.\(calibration.sensorID).samples.observedLatencySeconds"
                    )
                }
            }
        }
    }

    private func validateIdentity(body: KuyuBodyModel, embodiment: EmbodimentContract) throws {
        if schemaVersion.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw HardwareCalibrationValidationError.empty("schemaVersion")
        }
        if reportID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw HardwareCalibrationValidationError.empty("reportID")
        }
        if robotID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw HardwareCalibrationValidationError.empty("robotID")
        }
        if bodyID != body.bodyID {
            throw HardwareCalibrationValidationError.unknownReference("bodyID.\(bodyID)")
        }
        if embodimentContractID != embodiment.contractID {
            throw HardwareCalibrationValidationError.unknownReference("embodimentContractID.\(embodimentContractID)")
        }
        if source.measurementSystem.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw HardwareCalibrationValidationError.empty("source.measurementSystem")
        }
        try ensurePositive(positionToleranceRadians, "positionToleranceRadians")
    }

    private func validateJointEvidence(body: KuyuBodyModel) throws {
        if jointCalibrations.isEmpty {
            throw HardwareCalibrationValidationError.empty("jointCalibrations")
        }
        let activeJointIDs = Set(activeJoints(body: body).map(\.id))
        let attachmentsByJoint = Dictionary(uniqueKeysWithValues: body.actuatorAttachments.map { ($0.jointID, $0) })
        var jointIDs: Set<String> = []
        for calibration in jointCalibrations {
            if calibration.jointID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw HardwareCalibrationValidationError.empty("jointCalibrations.jointID")
            }
            if !jointIDs.insert(calibration.jointID).inserted {
                throw HardwareCalibrationValidationError.duplicate("jointCalibrations.\(calibration.jointID)")
            }
            if !activeJointIDs.contains(calibration.jointID) {
                throw HardwareCalibrationValidationError.unknownReference("jointCalibrations.\(calibration.jointID)")
            }
            guard let attachment = attachmentsByJoint[calibration.jointID] else {
                throw HardwareCalibrationValidationError.unknownReference("jointCalibrations.\(calibration.jointID).attachment")
            }
            if calibration.actuatorID != attachment.actuatorID {
                throw HardwareCalibrationValidationError.unknownReference("jointCalibrations.\(calibration.jointID).actuatorID")
            }
            try ensureFinite(calibration.commandDirection, "jointCalibrations.commandDirection")
            if calibration.commandDirection != attachment.commandDirection {
                throw HardwareCalibrationValidationError.invalidRange("jointCalibrations.\(calibration.jointID).commandDirection")
            }
            try ensurePositive(calibration.mechanicalReductionRatio, "jointCalibrations.mechanicalReductionRatio")
            if calibration.mechanicalReductionRatio != attachment.mechanicalReductionRatio {
                throw HardwareCalibrationValidationError.invalidRange(
                    "jointCalibrations.\(calibration.jointID).mechanicalReductionRatio"
                )
            }
            try validateDynamics(calibration.identifiedDynamics, field: "jointCalibrations.\(calibration.jointID)")
            for sample in calibration.samples {
                try validateSample(sample, field: "jointCalibrations.\(calibration.jointID).samples")
            }
        }
    }

    private func validateSensorEvidence(embodiment: EmbodimentContract) throws {
        let sensorsByID = Dictionary(uniqueKeysWithValues: embodiment.sensors.map { ($0.id, $0) })
        var sensorIDs: Set<String> = []
        for calibration in sensorCalibrations {
            if calibration.sensorID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw HardwareCalibrationValidationError.empty("sensorCalibrations.sensorID")
            }
            if !sensorIDs.insert(calibration.sensorID).inserted {
                throw HardwareCalibrationValidationError.duplicate("sensorCalibrations.\(calibration.sensorID)")
            }
            guard let sensor = sensorsByID[calibration.sensorID] else {
                throw HardwareCalibrationValidationError.unknownReference("sensorCalibrations.\(calibration.sensorID)")
            }
            if Set(calibration.channelIDs) != Set(sensor.channels) {
                throw HardwareCalibrationValidationError.insufficientCoverage(
                    "sensorCalibrations.\(calibration.sensorID).channels"
                )
            }
            try ensureNonNegative(
                calibration.measuredLatencySeconds,
                "sensorCalibrations.\(calibration.sensorID).measuredLatencySeconds"
            )
            try ensurePositive(
                calibration.latencyToleranceSeconds,
                "sensorCalibrations.\(calibration.sensorID).latencyToleranceSeconds"
            )
            if abs(calibration.measuredLatencySeconds - sensor.latencySeconds) > calibration.latencyToleranceSeconds {
                throw HardwareCalibrationValidationError.invalidRange(
                    "sensorCalibrations.\(calibration.sensorID).measuredLatencySeconds"
                )
            }
            for sample in calibration.samples {
                try validateSensorSample(
                    sample,
                    channelIDs: Set(sensor.channels),
                    field: "sensorCalibrations.\(calibration.sensorID).samples"
                )
            }
        }
    }

    private func validateContactEvidence(body: KuyuBodyModel) throws {
        let linkIDs = Set(body.links.map(\.id))
        let materialIDs = Set(body.materials.map(\.id))
        var pairIDs: Set<String> = []
        for calibration in contactCalibrations {
            if calibration.contactPairID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw HardwareCalibrationValidationError.empty("contactCalibrations.contactPairID")
            }
            if !pairIDs.insert(calibration.contactPairID).inserted {
                throw HardwareCalibrationValidationError.duplicate("contactCalibrations.\(calibration.contactPairID)")
            }
            if !linkIDs.contains(calibration.linkID) {
                throw HardwareCalibrationValidationError.unknownReference("contactCalibrations.\(calibration.linkID)")
            }
            if !materialIDs.contains(calibration.materialID) {
                throw HardwareCalibrationValidationError.unknownReference("contactCalibrations.\(calibration.materialID)")
            }
            try ensureNonNegative(calibration.staticFriction, "contactCalibrations.staticFriction")
            try ensureNonNegative(calibration.dynamicFriction, "contactCalibrations.dynamicFriction")
            try ensurePositive(calibration.normalStiffness, "contactCalibrations.normalStiffness")
            try ensureNonNegative(calibration.normalDamping, "contactCalibrations.normalDamping")
            for sample in calibration.samples {
                try ensureFinite(sample.normalForceNewtons, "contactCalibrations.samples.normalForceNewtons")
                try ensureFinite(sample.tangentialForceNewtons, "contactCalibrations.samples.tangentialForceNewtons")
                try ensureFinite(sample.slipVelocityMetersPerSecond, "contactCalibrations.samples.slipVelocityMetersPerSecond")
                try ensureFinite(sample.penetrationMeters, "contactCalibrations.samples.penetrationMeters")
                try ensureFinite(sample.timestampSeconds, "contactCalibrations.samples.timestampSeconds")
            }
        }
    }
}
