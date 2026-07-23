import Foundation
import KuyuCore

extension DescriptorCorpusAcceptanceService {
    func hardwareEvidence(entry: DescriptorCorpusEntry) throws -> DescriptorCorpusHardwareEvidence? {
        guard let report = entry.hardwareReport else {
            return nil
        }
        do {
            try report.validate(body: entry.body, embodiment: entry.embodiment)
        } catch {
            throw DescriptorCorpusAcceptanceError.readinessFailed(
                entryID: entry.entryID,
                reason: "hardwareEvidence.\(String(describing: error))"
            )
        }
        return try DescriptorCorpusHardwareEvidence(
            report: report,
            reportHash: ConfigHash.hash(report)
        )
    }

    func hardwareParityStatus(
        entry: DescriptorCorpusEntry,
        gate: ReadinessGate
    ) -> DescriptorCorpusHardwareParityStatus {
        guard entry.requiredReadiness < .hardwareParity || entry.hardwareReport != nil else {
            return .rejected(reason: "hardwareParity.report.missing")
        }
        guard entry.hardwareReport != nil else {
            return .notRequested
        }

        do {
            _ = try gate.validate(
                body: entry.body,
                world: entry.world,
                embodiment: entry.embodiment,
                report: entry.compatibilityReport,
                requiredLevel: .hardwareParity,
                hardwareReport: entry.hardwareReport
            )
            return .accepted
        } catch {
            return .rejected(reason: String(describing: error))
        }
    }

    func readinessGaps(
        entry: DescriptorCorpusEntry,
        hardwareParity: DescriptorCorpusHardwareParityStatus
    ) -> [DescriptorCorpusReadinessGap] {
        switch hardwareParity {
        case .accepted, .notRequested:
            return []
        case .rejected(let reason):
            return [
                DescriptorCorpusReadinessGap(
                    entryID: entry.entryID,
                    readinessLevel: .hardwareParity,
                    reason: reason
                )
            ]
        }
    }
}
