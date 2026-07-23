import KuyuCore

public struct LiftMotorNerve: MotorNerveEndpoint, MotorNerveTraceProvider, Sendable {
    public let motorMaxThrusts: MotorMaxThrusts

    public var lastTrace: MotorNerveTrace? { lastTraceStorage }

    private var lastTraceStorage: MotorNerveTrace?

    public init(motorMaxThrusts: MotorMaxThrusts) {
        self.motorMaxThrusts = motorMaxThrusts
        self.lastTraceStorage = nil
    }

    public mutating func update(
        input drives: [DriveIntent],
        corrections: [ReflexCorrection],
        telemetry: MotorNerveTelemetry,
        time: WorldTime
    ) throws -> [ActuatorValue] {
        _ = telemetry
        _ = time
        let adjusted = try applyCorrections(drives: drives, corrections: corrections)
        let throttle = driveValue(index: 0, from: adjusted)
        let clamped = clamp(throttle, lower: 0.0, upper: 1.0)
        let normalized = telemetry.failsafeActive ? [0.0, 0.0, 0.0, 0.0] : [clamped, clamped, clamped, clamped]
        var commands: [ActuatorValue] = []
        commands.reserveCapacity(4)
        let scaled = [
            normalized[0] * motorMaxThrusts.f1,
            normalized[1] * motorMaxThrusts.f2,
            normalized[2] * motorMaxThrusts.f3,
            normalized[3] * motorMaxThrusts.f4
        ]
        for (index, value) in scaled.enumerated() {
            let actuatorIndex = ActuatorIndex(UInt32(index))
            let command = try ActuatorValue(index: actuatorIndex, value: value)
            commands.append(command)
        }
        lastTraceStorage = MotorNerveTrace(
            uRaw: [throttle, throttle, throttle, throttle],
            uSat: [clamped, clamped, clamped, clamped],
            uRate: [clamped, clamped, clamped, clamped],
            uOut: normalized,
            failsafeActive: telemetry.failsafeActive
        )
        return commands
    }

    private func applyCorrections(
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
            let adjusted = clamped + entry.delta
            return try DriveIntent(index: drive.index, activation: adjusted)
        }
    }

    private func driveValue(index: UInt32, from drives: [DriveIntent]) -> Double {
        drives.first(where: { $0.index.rawValue == index })?.activation ?? 0.0
    }

    private func clamp(_ value: Double, lower: Double, upper: Double) -> Double {
        min(max(value, lower), upper)
    }

    private struct ReflexAggregate {
        var clampMultiplier: Double = 1.0
        var damping: Double = 0.0
        var delta: Double = 0.0
    }
}
