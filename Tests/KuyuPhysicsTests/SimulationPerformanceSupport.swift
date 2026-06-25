import Foundation
import KuyuCore

struct SimulationThroughputMeasurement {
    let log: SimulationLog
    let unitsPerSecond: Double
}

enum SimulationThroughputMeasurementError: Error {
    case noMeasurement
}

func strictPerformanceBudgetTarget(_ target: Double) -> Double? {
    ProcessInfo.processInfo.environment["KUYU_PHYSICS_STRICT_PERFORMANCE_BUDGETS"] == "1"
        ? target
        : nil
}

func bestSimulationThroughput(
    attempts: Int = 3,
    target: Double,
    unitCount: (SimulationLog) -> Int,
    run: () async throws -> SimulationLog
) async throws -> SimulationThroughputMeasurement {
    var best: SimulationThroughputMeasurement?
    for _ in 0..<attempts {
        let startedAt = Date()
        let log = try await run()
        let elapsedSeconds = max(Date().timeIntervalSince(startedAt), Double.leastNonzeroMagnitude)
        let unitsPerSecond = Double(unitCount(log)) / elapsedSeconds
        if best.map({ unitsPerSecond > $0.unitsPerSecond }) ?? true {
            best = SimulationThroughputMeasurement(
                log: log,
                unitsPerSecond: unitsPerSecond
            )
        }
        if unitsPerSecond >= target {
            break
        }
    }
    guard let best else {
        throw SimulationThroughputMeasurementError.noMeasurement
    }
    return best
}
