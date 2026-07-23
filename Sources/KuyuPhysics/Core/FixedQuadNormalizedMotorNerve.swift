import KuyuCore

public struct FixedQuadNormalizedMotorNerve: MotorNerveEndpoint, MotorNerveTraceProvider, Sendable {
    public struct Config: Sendable, Equatable {
        public let motorMaxThrusts: MotorMaxThrusts
        public let rateLimitPerSecond: Double
        public let smoothingTimeConstant: Double?

        public init(
            motorMaxThrusts: MotorMaxThrusts,
            rateLimitPerSecond: Double = 2.0,
            smoothingTimeConstant: Double? = 0.08
        ) {
            self.motorMaxThrusts = motorMaxThrusts
            self.rateLimitPerSecond = rateLimitPerSecond
            self.smoothingTimeConstant = smoothingTimeConstant
        }
    }

    public var lastTrace: MotorNerveTrace? { shaper.lastTrace }

    private var shaper: FixedQuadMotorCommandShaper

    public init(config: Config) {
        self.shaper = FixedQuadMotorCommandShaper(
            config: FixedQuadMotorCommandShaper.Config(
                motorMaxThrusts: config.motorMaxThrusts,
                rateLimitPerSecond: config.rateLimitPerSecond,
                smoothingTimeConstant: config.smoothingTimeConstant
            )
        )
    }

    public mutating func update(
        input drives: [DriveIntent],
        corrections: [ReflexCorrection],
        telemetry: MotorNerveTelemetry,
        time: WorldTime
    ) throws -> [ActuatorValue] {
        let adjusted = try shaper.corrected(drives: drives, corrections: corrections)
        let rawValues = (0..<4).map { index in
            adjusted.first { $0.index.rawValue == UInt32(index) }?.activation ?? 0.0
        }
        return try shaper.commands(rawValues: rawValues, telemetry: telemetry, time: time)
    }
}
