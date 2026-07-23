import Foundation

public struct DescriptorCorpusAcceptanceSummary: Sendable, Codable, Equatable {
    public let schemaVersion: Int
    public let corpusID: String
    public let generatedAt: String?
    public let records: [DescriptorCorpusAcceptanceRecord]

    public init(
        schemaVersion: Int = 1,
        corpusID: String,
        generatedAt: String? = nil,
        records: [DescriptorCorpusAcceptanceRecord]
    ) {
        self.schemaVersion = schemaVersion
        self.corpusID = corpusID
        self.generatedAt = generatedAt
        self.records = records
    }

    public var accepted: Bool {
        !records.isEmpty && records.allSatisfy(\.accepted)
    }

    public var hardwareParityGaps: [DescriptorCorpusReadinessGap] {
        records.flatMap(\.readinessGaps)
    }
}
