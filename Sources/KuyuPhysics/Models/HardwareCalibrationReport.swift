import EmbodimentContract
import Foundation

public enum HardwareCalibrationValidationError: Error, Equatable {
    case empty(String)
    case duplicate(String)
    case unknownReference(String)
    case invalidRange(String)
    case nonFinite(String)
    case missingMeasuredEvidence(String)
    case insufficientCoverage(String)
}

public struct HardwareCalibrationReport: Sendable, Codable, Equatable {
    public let schemaVersion: String
    public let reportID: String
    public let generatedAt: String?
    public let robotID: String
    public let bodyID: String
    public let embodimentContractID: String
    public let readinessLevel: ReadinessLevel
    public let positionToleranceRadians: Double
    public let minimumSamplesPerJoint: Int
    public let source: HardwareCalibrationSource
    public let jointCalibrations: [JointHardwareCalibration]
    public let contactCalibrations: [ContactHardwareCalibration]

    public init(
        schemaVersion: String,
        reportID: String,
        generatedAt: String? = nil,
        robotID: String,
        bodyID: String,
        embodimentContractID: String,
        readinessLevel: ReadinessLevel,
        positionToleranceRadians: Double,
        minimumSamplesPerJoint: Int = 3,
        source: HardwareCalibrationSource,
        jointCalibrations: [JointHardwareCalibration],
        contactCalibrations: [ContactHardwareCalibration] = []
    ) {
        self.schemaVersion = schemaVersion
        self.reportID = reportID
        self.generatedAt = generatedAt
        self.robotID = robotID
        self.bodyID = bodyID
        self.embodimentContractID = embodimentContractID
        self.readinessLevel = readinessLevel
        self.positionToleranceRadians = positionToleranceRadians
        self.minimumSamplesPerJoint = minimumSamplesPerJoint
        self.source = source
        self.jointCalibrations = jointCalibrations
        self.contactCalibrations = contactCalibrations
    }
}

public struct HardwareCalibrationSource: Sendable, Codable, Equatable {
    public let operatorID: String?
    public let deviceID: String?
    public let firmwareVersion: String?
    public let measurementSystem: String
    public let notes: String?

    public init(
        operatorID: String? = nil,
        deviceID: String? = nil,
        firmwareVersion: String? = nil,
        measurementSystem: String,
        notes: String? = nil
    ) {
        self.operatorID = operatorID
        self.deviceID = deviceID
        self.firmwareVersion = firmwareVersion
        self.measurementSystem = measurementSystem
        self.notes = notes
    }
}

public struct JointHardwareCalibration: Sendable, Codable, Equatable {
    public let jointID: String
    public let actuatorID: String
    public let commandDirection: Double
    public let mechanicalReductionRatio: Double
    public let identifiedDynamics: IdentifiedJointDynamics
    public let samples: [JointCalibrationSample]

    public init(
        jointID: String,
        actuatorID: String,
        commandDirection: Double,
        mechanicalReductionRatio: Double,
        identifiedDynamics: IdentifiedJointDynamics,
        samples: [JointCalibrationSample]
    ) {
        self.jointID = jointID
        self.actuatorID = actuatorID
        self.commandDirection = commandDirection
        self.mechanicalReductionRatio = mechanicalReductionRatio
        self.identifiedDynamics = identifiedDynamics
        self.samples = samples
    }
}

public struct IdentifiedJointDynamics: Sendable, Codable, Equatable {
    public let latencySeconds: Double
    public let timeConstantSeconds: Double
    public let deadbandRadians: Double
    public let backlashRadians: Double
    public let viscousDamping: Double
    public let coulombFriction: Double
    public let meanAbsoluteErrorRadians: Double
    public let maxObservedErrorRadians: Double

    public init(
        latencySeconds: Double,
        timeConstantSeconds: Double,
        deadbandRadians: Double,
        backlashRadians: Double,
        viscousDamping: Double,
        coulombFriction: Double,
        meanAbsoluteErrorRadians: Double,
        maxObservedErrorRadians: Double
    ) {
        self.latencySeconds = latencySeconds
        self.timeConstantSeconds = timeConstantSeconds
        self.deadbandRadians = deadbandRadians
        self.backlashRadians = backlashRadians
        self.viscousDamping = viscousDamping
        self.coulombFriction = coulombFriction
        self.meanAbsoluteErrorRadians = meanAbsoluteErrorRadians
        self.maxObservedErrorRadians = maxObservedErrorRadians
    }
}

public struct JointCalibrationSample: Sendable, Codable, Equatable {
    public let commandedPositionRadians: Double
    public let measuredPositionRadians: Double?
    public let commandPulse: Int?
    public let commandTimeSeconds: Double
    public let observedTimeSeconds: Double?
    public let measuredVelocityRadiansPerSecond: Double?
    public let busVoltageVolts: Double?
    public let servoTemperatureCelsius: Double?
    public let loadEstimate: Double?

    public init(
        commandedPositionRadians: Double,
        measuredPositionRadians: Double? = nil,
        commandPulse: Int? = nil,
        commandTimeSeconds: Double,
        observedTimeSeconds: Double? = nil,
        measuredVelocityRadiansPerSecond: Double? = nil,
        busVoltageVolts: Double? = nil,
        servoTemperatureCelsius: Double? = nil,
        loadEstimate: Double? = nil
    ) {
        self.commandedPositionRadians = commandedPositionRadians
        self.measuredPositionRadians = measuredPositionRadians
        self.commandPulse = commandPulse
        self.commandTimeSeconds = commandTimeSeconds
        self.observedTimeSeconds = observedTimeSeconds
        self.measuredVelocityRadiansPerSecond = measuredVelocityRadiansPerSecond
        self.busVoltageVolts = busVoltageVolts
        self.servoTemperatureCelsius = servoTemperatureCelsius
        self.loadEstimate = loadEstimate
    }
}

public struct ContactHardwareCalibration: Sendable, Codable, Equatable {
    public let contactPairID: String
    public let linkID: String
    public let materialID: String
    public let staticFriction: Double
    public let dynamicFriction: Double
    public let normalStiffness: Double
    public let normalDamping: Double
    public let samples: [ContactCalibrationSample]

    public init(
        contactPairID: String,
        linkID: String,
        materialID: String,
        staticFriction: Double,
        dynamicFriction: Double,
        normalStiffness: Double,
        normalDamping: Double,
        samples: [ContactCalibrationSample]
    ) {
        self.contactPairID = contactPairID
        self.linkID = linkID
        self.materialID = materialID
        self.staticFriction = staticFriction
        self.dynamicFriction = dynamicFriction
        self.normalStiffness = normalStiffness
        self.normalDamping = normalDamping
        self.samples = samples
    }
}

public struct ContactCalibrationSample: Sendable, Codable, Equatable {
    public let normalForceNewtons: Double
    public let tangentialForceNewtons: Double
    public let slipVelocityMetersPerSecond: Double
    public let penetrationMeters: Double
    public let timestampSeconds: Double

    public init(
        normalForceNewtons: Double,
        tangentialForceNewtons: Double,
        slipVelocityMetersPerSecond: Double,
        penetrationMeters: Double,
        timestampSeconds: Double
    ) {
        self.normalForceNewtons = normalForceNewtons
        self.tangentialForceNewtons = tangentialForceNewtons
        self.slipVelocityMetersPerSecond = slipVelocityMetersPerSecond
        self.penetrationMeters = penetrationMeters
        self.timestampSeconds = timestampSeconds
    }
}

public extension HardwareCalibrationReport {
    static let strictHardwareParityPositionToleranceRadians = 0.05

    func validate(body: KuyuBodyModel, embodiment: EmbodimentContract) throws {
        try validateIdentity(body: body, embodiment: embodiment)
        try validateJointEvidence(body: body)
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

private func activeJoints(body: KuyuBodyModel) -> [JointDefinition] {
    body.joints.filter { joint in
        joint.mimic == nil && (joint.kind == .revolute || joint.kind == .continuous || joint.kind == .prismatic)
    }
}

private func validateDynamics(_ dynamics: IdentifiedJointDynamics, field: String) throws {
    try ensureNonNegative(dynamics.latencySeconds, "\(field).latencySeconds")
    try ensurePositive(dynamics.timeConstantSeconds, "\(field).timeConstantSeconds")
    try ensureNonNegative(dynamics.deadbandRadians, "\(field).deadbandRadians")
    try ensureNonNegative(dynamics.backlashRadians, "\(field).backlashRadians")
    try ensureNonNegative(dynamics.viscousDamping, "\(field).viscousDamping")
    try ensureNonNegative(dynamics.coulombFriction, "\(field).coulombFriction")
    try ensureNonNegative(dynamics.meanAbsoluteErrorRadians, "\(field).meanAbsoluteErrorRadians")
    try ensureNonNegative(dynamics.maxObservedErrorRadians, "\(field).maxObservedErrorRadians")
}

private func validateSample(_ sample: JointCalibrationSample, field: String) throws {
    try ensureFinite(sample.commandedPositionRadians, "\(field).commandedPositionRadians")
    try ensureFinite(sample.commandTimeSeconds, "\(field).commandTimeSeconds")
    try ensureOptionalFinite(sample.measuredPositionRadians, "\(field).measuredPositionRadians")
    try ensureOptionalFinite(sample.observedTimeSeconds, "\(field).observedTimeSeconds")
    try ensureOptionalFinite(sample.measuredVelocityRadiansPerSecond, "\(field).measuredVelocityRadiansPerSecond")
    try ensureOptionalFinite(sample.busVoltageVolts, "\(field).busVoltageVolts")
    try ensureOptionalFinite(sample.servoTemperatureCelsius, "\(field).servoTemperatureCelsius")
    try ensureOptionalFinite(sample.loadEstimate, "\(field).loadEstimate")
    if let commandPulse = sample.commandPulse, !(0...4095).contains(commandPulse) {
        throw HardwareCalibrationValidationError.invalidRange("\(field).commandPulse")
    }
}

private func ensureFinite(_ value: Double, _ field: String) throws {
    if !value.isFinite {
        throw HardwareCalibrationValidationError.nonFinite(field)
    }
}

private func ensureOptionalFinite(_ value: Double?, _ field: String) throws {
    guard let value else { return }
    try ensureFinite(value, field)
}

private func ensurePositive(_ value: Double, _ field: String) throws {
    try ensureFinite(value, field)
    if value <= 0 {
        throw HardwareCalibrationValidationError.invalidRange(field)
    }
}

private func ensureNonNegative(_ value: Double, _ field: String) throws {
    try ensureFinite(value, field)
    if value < 0 {
        throw HardwareCalibrationValidationError.invalidRange(field)
    }
}
