import KuyuCore

public struct FixedQuadMotorNerve: MotorNerveEndpoint, MotorNerveTraceProvider, Sendable {
    public struct Config: Sendable, Equatable {
        public let mixer: ReferenceQuadrotorMixer
        public let motorMaxThrusts: MotorMaxThrusts
        public let rollScale: Double
        public let pitchScale: Double
        public let yawScale: Double
        public let rateLimitPerSecond: Double
        public let smoothingTimeConstant: Double?

        public init(
            mixer: ReferenceQuadrotorMixer,
            motorMaxThrusts: MotorMaxThrusts,
            rollScale: Double = 1.0,
            pitchScale: Double = 1.0,
            yawScale: Double = 1.0,
            rateLimitPerSecond: Double = 2.0,
            smoothingTimeConstant: Double? = 0.08
        ) {
            self.mixer = mixer
            self.motorMaxThrusts = motorMaxThrusts
            self.rollScale = rollScale
            self.pitchScale = pitchScale
            self.yawScale = yawScale
            self.rateLimitPerSecond = rateLimitPerSecond
            self.smoothingTimeConstant = smoothingTimeConstant
        }
    }

    public var lastTrace: MotorNerveTrace? { shaper.lastTrace }

    private let config: Config
    private var shaper: FixedQuadMotorCommandShaper

    public init(config: Config) {
        self.config = config
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

        let throttle = clamp(driveValue(index: 0, from: adjusted), lower: 0.0, upper: 1.0)
        let roll = clamp(driveValue(index: 1, from: adjusted), lower: -1.0, upper: 1.0) * config.rollScale
        let pitch = clamp(driveValue(index: 2, from: adjusted), lower: -1.0, upper: 1.0) * config.pitchScale
        let yaw = clamp(driveValue(index: 3, from: adjusted), lower: -1.0, upper: 1.0) * config.yawScale

        let spin = config.mixer.spinDirections
        let uRaw = [
            throttle - pitch + spin.x * yaw,
            throttle + roll + spin.y * yaw,
            throttle + pitch + spin.z * yaw,
            throttle - roll + spin.w * yaw
        ]

        return try shaper.commands(rawValues: uRaw, telemetry: telemetry, time: time)
    }

    private func driveValue(index: UInt32, from drives: [DriveIntent]) -> Double {
        drives.first(where: { $0.index.rawValue == index })?.activation ?? 0.0
    }

    private func clamp(_ value: Double, lower: Double, upper: Double) -> Double {
        min(max(value, lower), upper)
    }
}
