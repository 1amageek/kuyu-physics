import Foundation
import KuyuCore

public struct DescriptorCorpusReplayEvidence: Sendable, Codable, Equatable {
    public let passed: Bool
    public let tier: DeterminismTier
    public let stepCount: Int
    public let configHash: String
    public let sortedJSONByteStable: Bool
    public let issues: [String]
    public let residuals: ReplayResiduals
    public let contact: DescriptorCorpusContactReplayEvidence?

    public init(
        passed: Bool,
        tier: DeterminismTier,
        stepCount: Int,
        configHash: String,
        sortedJSONByteStable: Bool,
        issues: [String],
        residuals: ReplayResiduals,
        contact: DescriptorCorpusContactReplayEvidence? = nil
    ) {
        self.passed = passed
        self.tier = tier
        self.stepCount = stepCount
        self.configHash = configHash
        self.sortedJSONByteStable = sortedJSONByteStable
        self.issues = issues
        self.residuals = residuals
        self.contact = contact
    }
}

public struct DescriptorCorpusContactReplayEvidence: Sendable, Codable, Equatable {
    public let maxActiveContactCount: Double
    public let maxPenetration: Double
    public let maxNormalImpulse: Double
    public let maxNormalForce: Double
    public let maxSolverIterations: Double

    public init(
        maxActiveContactCount: Double,
        maxPenetration: Double,
        maxNormalImpulse: Double,
        maxNormalForce: Double,
        maxSolverIterations: Double
    ) {
        self.maxActiveContactCount = maxActiveContactCount
        self.maxPenetration = maxPenetration
        self.maxNormalImpulse = maxNormalImpulse
        self.maxNormalForce = maxNormalForce
        self.maxSolverIterations = maxSolverIterations
    }
}

public enum DescriptorCorpusAcceptanceError: Error, Equatable {
    case emptyCorpus
    case emptyField(entryID: String, field: String)
    case duplicateEntryID(String)
    case invalidDuration(entryID: String, duration: Double)
    case readinessFailed(entryID: String, reason: String)
    case simulationFailed(entryID: String, reason: String)
    case replayFailed(entryID: String, issues: [String])
}
