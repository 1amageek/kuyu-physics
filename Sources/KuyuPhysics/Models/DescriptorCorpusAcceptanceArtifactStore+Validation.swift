import Foundation

extension DescriptorCorpusAcceptanceArtifactStore {
    func validate(
        record: DescriptorCorpusAcceptanceRecord,
        entryIDs: inout Set<String>
    ) throws {
        try requireNonEmpty(record.entryID, "records.entryID")
        try requireNonEmpty(record.robotID, "records.\(record.entryID).robotID")
        try requireNonEmpty(record.label, "records.\(record.entryID).label")
        try requireNonEmpty(record.bodyID, "records.\(record.entryID).bodyID")
        try requireNonEmpty(record.worldID, "records.\(record.entryID).worldID")
        try requireNonEmpty(record.embodimentContractID, "records.\(record.entryID).embodimentContractID")
        guard entryIDs.insert(record.entryID).inserted else {
            throw StoreError.invalidSummary("records.\(record.entryID).duplicate")
        }
        guard record.achievedReadiness >= record.requiredReadiness else {
            throw StoreError.invalidSummary("records.\(record.entryID).readiness")
        }
        try validateHardwareParity(record: record)
        guard record.accepted else {
            throw StoreError.invalidSummary("records.\(record.entryID).accepted")
        }
        try validate(replay: record.replay, entryID: record.entryID)
        if record.requiredReadiness == .contactTraining {
            guard let contact = record.replay.contact, contact.maxActiveContactCount > 0 else {
                throw StoreError.invalidSummary("records.\(record.entryID).replay.contact")
            }
        }
    }

    func validate(
        replay: DescriptorCorpusReplayEvidence,
        entryID: String
    ) throws {
        guard replay.passed else {
            throw StoreError.invalidSummary("records.\(entryID).replay.passed")
        }
        guard replay.sortedJSONByteStable else {
            throw StoreError.invalidSummary("records.\(entryID).replay.sortedJSONByteStable")
        }
        guard replay.stepCount > 0 else {
            throw StoreError.invalidSummary("records.\(entryID).replay.stepCount")
        }
        try requireNonEmpty(replay.configHash, "records.\(entryID).replay.configHash")
        let residuals = [
            replay.residuals.position,
            replay.residuals.velocity,
            replay.residuals.angularVelocity,
            replay.residuals.quaternionResidual,
            replay.residuals.motorThrust,
            replay.residuals.sensor
        ]
        guard residuals.allSatisfy({ $0.isFinite && $0 >= 0 }) else {
            throw StoreError.invalidSummary("records.\(entryID).replay.residuals")
        }
        if let contact = replay.contact {
            try validate(contact: contact, entryID: entryID)
        }
    }

    func validate(
        contact: DescriptorCorpusContactReplayEvidence,
        entryID: String
    ) throws {
        let values = [
            contact.maxActiveContactCount,
            contact.maxPenetration,
            contact.maxNormalImpulse,
            contact.maxNormalForce,
            contact.maxSolverIterations
        ]
        guard values.allSatisfy({ $0.isFinite && $0 >= 0 }) else {
            throw StoreError.invalidSummary("records.\(entryID).replay.contact")
        }
    }

    func validateHardwareParity(record: DescriptorCorpusAcceptanceRecord) throws {
        for gap in record.readinessGaps {
            try requireNonEmpty(gap.entryID, "records.\(record.entryID).readinessGaps.entryID")
            try requireNonEmpty(gap.reason, "records.\(record.entryID).readinessGaps.reason")
            guard gap.entryID == record.entryID else {
                throw StoreError.invalidSummary("records.\(record.entryID).readinessGaps.entryID")
            }
        }
        switch record.hardwareParity {
        case .accepted:
            guard let evidence = record.hardwareEvidence else {
                throw StoreError.invalidSummary("records.\(record.entryID).hardwareParity.evidence")
            }
            try validate(hardwareEvidence: evidence, record: record)
            guard evidence.readinessLevel >= .hardwareParity else {
                throw StoreError.invalidSummary("records.\(record.entryID).hardwareParity.evidence.readinessLevel")
            }
            guard evidence.observedJointSampleCount == evidence.jointSampleCount else {
                throw StoreError.invalidSummary("records.\(record.entryID).hardwareParity.evidence.observedJointSampleCount")
            }
            guard evidence.observedSensorSampleCount == evidence.sensorSampleCount else {
                throw StoreError.invalidSummary("records.\(record.entryID).hardwareParity.evidence.observedSensorSampleCount")
            }
            guard record.achievedReadiness >= .hardwareParity else {
                throw StoreError.invalidSummary("records.\(record.entryID).hardwareParity.readiness")
            }
            guard record.readinessGaps.isEmpty else {
                throw StoreError.invalidSummary("records.\(record.entryID).hardwareParity.gaps")
            }
        case .notRequested:
            guard record.hardwareEvidence == nil else {
                throw StoreError.invalidSummary("records.\(record.entryID).hardwareParity.evidence")
            }
            guard record.readinessGaps.isEmpty else {
                throw StoreError.invalidSummary("records.\(record.entryID).hardwareParity.gaps")
            }
        case .rejected(let reason):
            if let evidence = record.hardwareEvidence {
                try validate(hardwareEvidence: evidence, record: record)
            }
            try requireNonEmpty(reason, "records.\(record.entryID).hardwareParity.reason")
            guard record.readinessGaps.contains(where: { $0.readinessLevel == .hardwareParity }) else {
                throw StoreError.invalidSummary("records.\(record.entryID).hardwareParity.gap")
            }
        }
    }

    func validate(
        hardwareEvidence: DescriptorCorpusHardwareEvidence,
        record: DescriptorCorpusAcceptanceRecord
    ) throws {
        try requireNonEmpty(hardwareEvidence.reportID, "records.\(record.entryID).hardwareEvidence.reportID")
        try requireNonEmpty(hardwareEvidence.robotID, "records.\(record.entryID).hardwareEvidence.robotID")
        try requireNonEmpty(hardwareEvidence.bodyID, "records.\(record.entryID).hardwareEvidence.bodyID")
        try requireNonEmpty(
            hardwareEvidence.embodimentContractID,
            "records.\(record.entryID).hardwareEvidence.embodimentContractID"
        )
        try requireNonEmpty(hardwareEvidence.reportHash, "records.\(record.entryID).hardwareEvidence.reportHash")
        try requireNonEmpty(
            hardwareEvidence.measurementSystem,
            "records.\(record.entryID).hardwareEvidence.measurementSystem"
        )
        guard hardwareEvidence.robotID == record.robotID else {
            throw StoreError.invalidSummary("records.\(record.entryID).hardwareEvidence.robotID")
        }
        guard hardwareEvidence.bodyID == record.bodyID else {
            throw StoreError.invalidSummary("records.\(record.entryID).hardwareEvidence.bodyID")
        }
        guard hardwareEvidence.embodimentContractID == record.embodimentContractID else {
            throw StoreError.invalidSummary("records.\(record.entryID).hardwareEvidence.embodimentContractID")
        }
        guard hardwareEvidence.jointCalibrationCount > 0 else {
            throw StoreError.invalidSummary("records.\(record.entryID).hardwareEvidence.jointCalibrationCount")
        }
        guard hardwareEvidence.jointSampleCount >= hardwareEvidence.jointCalibrationCount else {
            throw StoreError.invalidSummary("records.\(record.entryID).hardwareEvidence.jointSampleCount")
        }
        guard hardwareEvidence.measuredJointSampleCount >= 0,
              hardwareEvidence.measuredJointSampleCount <= hardwareEvidence.jointSampleCount else {
            throw StoreError.invalidSummary("records.\(record.entryID).hardwareEvidence.measuredJointSampleCount")
        }
        if let observedJointSampleCount = hardwareEvidence.observedJointSampleCount {
            guard observedJointSampleCount >= 0,
                  observedJointSampleCount <= hardwareEvidence.jointSampleCount else {
                throw StoreError.invalidSummary("records.\(record.entryID).hardwareEvidence.observedJointSampleCount")
            }
        }
        guard hardwareEvidence.sensorCalibrationCount >= 0,
              hardwareEvidence.sensorSampleCount >= 0,
              hardwareEvidence.observedSensorSampleCount >= 0,
              hardwareEvidence.observedSensorSampleCount <= hardwareEvidence.sensorSampleCount else {
            throw StoreError.invalidSummary("records.\(record.entryID).hardwareEvidence.sensorCoverage")
        }
        guard hardwareEvidence.contactCalibrationCount >= 0,
              hardwareEvidence.contactSampleCount >= 0 else {
            throw StoreError.invalidSummary("records.\(record.entryID).hardwareEvidence.contactCoverage")
        }
    }

    func requireNonEmpty(_ value: String, _ field: String) throws {
        guard !value.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty else {
            throw StoreError.invalidSummary(field)
        }
    }
}
