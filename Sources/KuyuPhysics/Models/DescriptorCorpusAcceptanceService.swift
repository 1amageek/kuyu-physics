import Foundation

public struct DescriptorCorpusAcceptanceService: Sendable {
    enum ReplayPath {
        case articulated
        case rigidActuator
    }

    public init() {}

    public func accept(
        corpusID: String,
        entries: [DescriptorCorpusEntry],
        generatedAt: String? = nil
    ) async throws -> DescriptorCorpusAcceptanceSummary {
        let trimmedCorpusID = corpusID.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmedCorpusID.isEmpty else {
            throw DescriptorCorpusAcceptanceError.emptyField(entryID: corpusID, field: "corpusID")
        }
        guard !entries.isEmpty else {
            throw DescriptorCorpusAcceptanceError.emptyCorpus
        }

        var entryIDs: Set<String> = []
        var records: [DescriptorCorpusAcceptanceRecord] = []
        records.reserveCapacity(entries.count)
        for entry in entries {
            let trimmedEntryID = entry.entryID.trimmingCharacters(in: .whitespacesAndNewlines)
            guard !trimmedEntryID.isEmpty else {
                throw DescriptorCorpusAcceptanceError.emptyField(entryID: entry.entryID, field: "entryID")
            }
            guard entryIDs.insert(trimmedEntryID).inserted else {
                throw DescriptorCorpusAcceptanceError.duplicateEntryID(trimmedEntryID)
            }
            records.append(try await accept(entry: entry))
        }

        return DescriptorCorpusAcceptanceSummary(
            corpusID: trimmedCorpusID,
            generatedAt: generatedAt,
            records: records
        )
    }

    func accept(entry: DescriptorCorpusEntry) async throws -> DescriptorCorpusAcceptanceRecord {
        guard entry.duration.isFinite, entry.duration > 0 else {
            throw DescriptorCorpusAcceptanceError.invalidDuration(
                entryID: entry.entryID,
                duration: entry.duration
            )
        }

        let gate = ReadinessGate()
        let validatedReadiness: ReadinessLevel
        do {
            validatedReadiness = try gate.validate(
                body: entry.body,
                world: entry.world,
                embodiment: entry.embodiment,
                report: entry.compatibilityReport,
                requiredLevel: entry.requiredReadiness,
                hardwareReport: entry.hardwareReport
            )
        } catch {
            throw DescriptorCorpusAcceptanceError.readinessFailed(
                entryID: entry.entryID,
                reason: String(describing: error)
            )
        }
        let hardwareEvidence = try hardwareEvidence(entry: entry)

        let hardwareParity = hardwareParityStatus(entry: entry, gate: gate)
        let achievedReadiness = hardwareParity == .accepted ? .hardwareParity : validatedReadiness
        let gaps = readinessGaps(
            entry: entry,
            hardwareParity: hardwareParity
        )
        let replay = try await replayEvidence(entry: entry)

        return DescriptorCorpusAcceptanceRecord(
            entryID: entry.entryID,
            robotID: entry.robotID,
            label: entry.label,
            bodyID: entry.body.bodyID,
            worldID: entry.world.worldID,
            embodimentContractID: entry.embodiment.contractID,
            requiredReadiness: entry.requiredReadiness,
            achievedReadiness: achievedReadiness,
            hardwareParity: hardwareParity,
            hardwareEvidence: hardwareEvidence,
            readinessGaps: gaps,
            replay: replay
        )
    }
}
