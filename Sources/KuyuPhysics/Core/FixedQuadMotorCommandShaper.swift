import Foundation
import KuyuCore

struct FixedQuadMotorCommandShaper: Sendable {
    struct Config: Sendable, Equatable {
        let motorMaxThrusts: MotorMaxThrusts
        let rateLimitPerSecond: Double
        let smoothingTimeConstant: Double?
    }

    private let config: Config
    private var lastOutput = [0.0, 0.0, 0.0, 0.0]
    private var lastFiltered = [0.0, 0.0, 0.0, 0.0]
    private var lastTime: Double?
    private(set) var lastTrace: MotorNerveTrace?

    init(config: Config) {
        self.config = config
    }

    mutating func commands(
        rawValues: [Double],
        telemetry: MotorNerveTelemetry,
        time: WorldTime
    ) throws -> [ActuatorValue] {
        precondition(rawValues.count == 4)
        let saturated = rawValues.map { min(max($0, 0.0), 1.0) }
        let delta = max(0.0, time.time - (lastTime ?? time.time))
        let rateLimited = rateLimitedValues(saturated, delta: delta)
        let filtered = smoothedValues(rateLimited, delta: delta)
        let output = telemetry.failsafeActive ? [0.0, 0.0, 0.0, 0.0] : filtered

        lastTime = time.time
        lastOutput = output
        lastFiltered = output
        lastTrace = MotorNerveTrace(
            uRaw: rawValues,
            uSat: saturated,
            uRate: rateLimited,
            uOut: output,
            failsafeActive: telemetry.failsafeActive
        )

        let scaled = [
            output[0] * config.motorMaxThrusts.f1,
            output[1] * config.motorMaxThrusts.f2,
            output[2] * config.motorMaxThrusts.f3,
            output[3] * config.motorMaxThrusts.f4
        ]
        return try scaled.enumerated().map { index, value in
            try ActuatorValue(index: ActuatorIndex(UInt32(index)), value: value)
        }
    }

    func corrected(
        drives: [DriveIntent],
        corrections: [ReflexCorrection]
    ) throws -> [DriveIntent] {
        guard !corrections.isEmpty else { return drives }

        var aggregate: [DriveIndex: ReflexAggregate] = [:]
        for correction in corrections {
            var entry = aggregate[correction.driveIndex] ?? ReflexAggregate()
            entry.clampMultiplier *= correction.clampMultiplier
            entry.damping = min(1.0, entry.damping + correction.damping)
            entry.delta += correction.delta
            aggregate[correction.driveIndex] = entry
        }

        return try drives.map { drive in
            guard let entry = aggregate[drive.index] else { return drive }
            let damped = drive.activation * (1.0 - entry.damping)
            let clamped = damped * entry.clampMultiplier
            return try DriveIntent(
                index: drive.index,
                activation: clamped + entry.delta,
                parameters: drive.parameters
            )
        }
    }

    private func rateLimitedValues(_ values: [Double], delta: Double) -> [Double] {
        guard delta > 0.0, config.rateLimitPerSecond > 0.0 else { return values }
        let maximumDelta = config.rateLimitPerSecond * delta
        return zip(values, lastOutput).map { current, previous in
            previous + min(max(current - previous, -maximumDelta), maximumDelta)
        }
    }

    private func smoothedValues(_ values: [Double], delta: Double) -> [Double] {
        guard let timeConstant = config.smoothingTimeConstant,
              timeConstant > 0.0,
              delta > 0.0 else {
            return values
        }
        let alpha = exp(-delta / timeConstant)
        return zip(values, lastFiltered).map { current, previous in
            alpha * previous + (1.0 - alpha) * current
        }
    }

    private struct ReflexAggregate {
        var clampMultiplier = 1.0
        var damping = 0.0
        var delta = 0.0
    }
}
