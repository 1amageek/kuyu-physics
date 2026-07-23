import Foundation

public struct DescriptorCorpusAcceptanceRecord: Sendable, Codable, Equatable {
    public let entryID: String
    public let robotID: String
    public let label: String
    public let bodyID: String
    public let worldID: String
    public let embodimentContractID: String
    public let requiredReadiness: ReadinessLevel
    public let achievedReadiness: ReadinessLevel
    public let hardwareParity: DescriptorCorpusHardwareParityStatus
    public let hardwareEvidence: DescriptorCorpusHardwareEvidence?
    public let readinessGaps: [DescriptorCorpusReadinessGap]
    public let replay: DescriptorCorpusReplayEvidence

    public init(
        entryID: String,
        robotID: String,
        label: String,
        bodyID: String,
        worldID: String,
        embodimentContractID: String,
        requiredReadiness: ReadinessLevel,
        achievedReadiness: ReadinessLevel,
        hardwareParity: DescriptorCorpusHardwareParityStatus,
        hardwareEvidence: DescriptorCorpusHardwareEvidence? = nil,
        readinessGaps: [DescriptorCorpusReadinessGap],
        replay: DescriptorCorpusReplayEvidence
    ) {
        self.entryID = entryID
        self.robotID = robotID
        self.label = label
        self.bodyID = bodyID
        self.worldID = worldID
        self.embodimentContractID = embodimentContractID
        self.requiredReadiness = requiredReadiness
        self.achievedReadiness = achievedReadiness
        self.hardwareParity = hardwareParity
        self.hardwareEvidence = hardwareEvidence
        self.readinessGaps = readinessGaps
        self.replay = replay
    }

    public var accepted: Bool {
        guard replay.passed, achievedReadiness >= requiredReadiness else {
            return false
        }
        switch hardwareParity {
        case .notRequested:
            return requiredReadiness < .hardwareParity && hardwareEvidence == nil
        case .accepted:
            return hardwareEvidence != nil
        case .rejected:
            return requiredReadiness < .hardwareParity
        }
    }
}

public enum DescriptorCorpusHardwareParityStatus: Sendable, Codable, Equatable {
    case notRequested
    case accepted
    case rejected(reason: String)
}

public struct DescriptorCorpusReadinessGap: Sendable, Codable, Equatable {
    public let entryID: String
    public let readinessLevel: ReadinessLevel
    public let reason: String

    public init(entryID: String, readinessLevel: ReadinessLevel, reason: String) {
        self.entryID = entryID
        self.readinessLevel = readinessLevel
        self.reason = reason
    }
}
