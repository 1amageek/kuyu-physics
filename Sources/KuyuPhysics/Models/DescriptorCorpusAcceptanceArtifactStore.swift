import Foundation

public struct DescriptorCorpusAcceptanceArtifactStore: Sendable {
    public enum StoreError: Error, Equatable {
        case missingArtifact(String)
        case unreadable(String)
        case undecodable(String)
        case unwritable(String)
        case invalidSummary(String)
    }

    public static let fileName = "descriptor-corpus-acceptance.json"

    public init() {}

    public func write(
        _ summary: DescriptorCorpusAcceptanceSummary,
        to directory: URL
    ) throws -> URL {
        try validate(summary)
        do {
            try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        } catch {
            throw StoreError.unwritable(directory.path)
        }
        let url = directory.appendingPathComponent(Self.fileName)
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        do {
            let data = try encoder.encode(summary)
            try data.write(to: url, options: [.atomic])
            return url
        } catch {
            throw StoreError.unwritable(url.path)
        }
    }

    public func validatedSummary(in directory: URL) throws -> DescriptorCorpusAcceptanceSummary {
        try validatedSummary(at: directory.appendingPathComponent(Self.fileName))
    }

    public func validatedSummary(at url: URL) throws -> DescriptorCorpusAcceptanceSummary {
        let data: Data
        do {
            data = try Data(contentsOf: url)
        } catch {
            throw StoreError.missingArtifact(url.path)
        }
        let summary: DescriptorCorpusAcceptanceSummary
        do {
            summary = try JSONDecoder().decode(DescriptorCorpusAcceptanceSummary.self, from: data)
        } catch {
            throw StoreError.undecodable(url.path)
        }
        try validate(summary)
        return summary
    }

    public func validate(_ summary: DescriptorCorpusAcceptanceSummary) throws {
        guard summary.schemaVersion == 1 else {
            throw StoreError.invalidSummary("schemaVersion")
        }
        guard !summary.corpusID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty else {
            throw StoreError.invalidSummary("corpusID")
        }
        guard !summary.records.isEmpty else {
            throw StoreError.invalidSummary("records")
        }

        var entryIDs: Set<String> = []
        for record in summary.records {
            try validate(record: record, entryIDs: &entryIDs)
        }
    }
}
