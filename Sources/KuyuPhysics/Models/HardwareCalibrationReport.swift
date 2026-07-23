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
    public let minimumSamplesPerSensor: Int
    public let source: HardwareCalibrationSource
    public let jointCalibrations: [JointHardwareCalibration]
    public let sensorCalibrations: [SensorHardwareCalibration]
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
        minimumSamplesPerSensor: Int = 3,
        source: HardwareCalibrationSource,
        jointCalibrations: [JointHardwareCalibration],
        sensorCalibrations: [SensorHardwareCalibration] = [],
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
        self.minimumSamplesPerSensor = minimumSamplesPerSensor
        self.source = source
        self.jointCalibrations = jointCalibrations
        self.sensorCalibrations = sensorCalibrations
        self.contactCalibrations = contactCalibrations
    }

    private enum CodingKeys: String, CodingKey {
        case schemaVersion
        case reportID
        case generatedAt
        case robotID
        case bodyID
        case embodimentContractID
        case readinessLevel
        case positionToleranceRadians
        case minimumSamplesPerJoint
        case minimumSamplesPerSensor
        case source
        case jointCalibrations
        case sensorCalibrations
        case contactCalibrations
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        schemaVersion = try container.decode(String.self, forKey: .schemaVersion)
        reportID = try container.decode(String.self, forKey: .reportID)
        generatedAt = try container.decodeIfPresent(String.self, forKey: .generatedAt)
        robotID = try container.decode(String.self, forKey: .robotID)
        bodyID = try container.decode(String.self, forKey: .bodyID)
        embodimentContractID = try container.decode(String.self, forKey: .embodimentContractID)
        readinessLevel = try container.decode(ReadinessLevel.self, forKey: .readinessLevel)
        positionToleranceRadians = try container.decode(Double.self, forKey: .positionToleranceRadians)
        minimumSamplesPerJoint = try container.decodeIfPresent(Int.self, forKey: .minimumSamplesPerJoint) ?? 3
        minimumSamplesPerSensor = try container.decodeIfPresent(Int.self, forKey: .minimumSamplesPerSensor) ?? 3
        source = try container.decode(HardwareCalibrationSource.self, forKey: .source)
        jointCalibrations = try container.decode([JointHardwareCalibration].self, forKey: .jointCalibrations)
        sensorCalibrations = try container.decodeIfPresent(
            [SensorHardwareCalibration].self,
            forKey: .sensorCalibrations
        ) ?? []
        contactCalibrations = try container.decodeIfPresent(
            [ContactHardwareCalibration].self,
            forKey: .contactCalibrations
        ) ?? []
    }

    public func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(schemaVersion, forKey: .schemaVersion)
        try container.encode(reportID, forKey: .reportID)
        try container.encodeIfPresent(generatedAt, forKey: .generatedAt)
        try container.encode(robotID, forKey: .robotID)
        try container.encode(bodyID, forKey: .bodyID)
        try container.encode(embodimentContractID, forKey: .embodimentContractID)
        try container.encode(readinessLevel, forKey: .readinessLevel)
        try container.encode(positionToleranceRadians, forKey: .positionToleranceRadians)
        try container.encode(minimumSamplesPerJoint, forKey: .minimumSamplesPerJoint)
        try container.encode(minimumSamplesPerSensor, forKey: .minimumSamplesPerSensor)
        try container.encode(source, forKey: .source)
        try container.encode(jointCalibrations, forKey: .jointCalibrations)
        try container.encode(sensorCalibrations, forKey: .sensorCalibrations)
        try container.encode(contactCalibrations, forKey: .contactCalibrations)
    }
}
