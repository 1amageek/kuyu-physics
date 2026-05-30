import Foundation

public enum CompatibilityMappingStatus: String, Sendable, Codable, Equatable {
    case exact
    case approximated
    case supplemented
    case ignored
    case unsupported
}

public struct CompatibilityMapping: Sendable, Codable, Equatable {
    public let source: String
    public let target: String
    public let status: CompatibilityMappingStatus
    public let notes: String?

    public init(
        source: String,
        target: String,
        status: CompatibilityMappingStatus,
        notes: String? = nil
    ) {
        self.source = source
        self.target = target
        self.status = status
        self.notes = notes
    }
}

public struct CompatibilityReport: Sendable, Codable, Equatable {
    public let schemaVersion: String
    public let reportID: String
    public let generatedAt: Date?
    public let sourceFormat: String
    public let targetContract: String
    public let mappings: [CompatibilityMapping]
    public let unsupportedFields: [String]
    public let readinessLevel: ReadinessLevel

    public init(
        schemaVersion: String,
        reportID: String,
        generatedAt: Date? = nil,
        sourceFormat: String,
        targetContract: String,
        mappings: [CompatibilityMapping],
        unsupportedFields: [String] = [],
        readinessLevel: ReadinessLevel
    ) {
        self.schemaVersion = schemaVersion
        self.reportID = reportID
        self.generatedAt = generatedAt
        self.sourceFormat = sourceFormat
        self.targetContract = targetContract
        self.mappings = mappings
        self.unsupportedFields = unsupportedFields
        self.readinessLevel = readinessLevel
    }

    public var hasUnsupportedMappings: Bool {
        !unsupportedFields.isEmpty || mappings.contains { $0.status == .unsupported }
    }
}
