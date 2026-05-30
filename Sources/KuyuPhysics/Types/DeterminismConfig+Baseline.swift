import KuyuCore

public extension DeterminismConfig {
    static let tier1Baseline: DeterminismConfig = {
        do {
            return try DeterminismConfig(tier: .tier1, tier1Tolerance: .baseline)
        } catch {
            preconditionFailure("Invalid Tier1 baseline config: \(error)")
        }
    }()

    /// Tier-0: bit-exact replay (same seed → identical float results, no tolerance).
    static let tier0Strict: DeterminismConfig = {
        do {
            return try DeterminismConfig(tier: .tier0)
        } catch {
            preconditionFailure("Invalid Tier0 strict config: \(error)")
        }
    }()
}
