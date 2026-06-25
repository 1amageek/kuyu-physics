import KuyuCore

public struct IMU6NoiseConfig: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case nonFinite(String)
        case negative(String)
    }

    public let gyroNoiseStdDev: Double
    public let gyroBias: Double
    public let gyroRandomWalkSigma: Double
    public let accelNoiseStdDev: Double
    public let accelBias: Double
    public let accelRandomWalkSigma: Double
    public let delaySteps: UInt64

    public init(
        gyroNoiseStdDev: Double,
        gyroBias: Double,
        gyroRandomWalkSigma: Double,
        accelNoiseStdDev: Double,
        accelBias: Double,
        accelRandomWalkSigma: Double,
        delaySteps: UInt64
    ) throws {
        guard gyroNoiseStdDev.isFinite else { throw ValidationError.nonFinite("gyroNoiseStdDev") }
        guard gyroBias.isFinite else { throw ValidationError.nonFinite("gyroBias") }
        guard gyroRandomWalkSigma.isFinite else { throw ValidationError.nonFinite("gyroRandomWalkSigma") }
        guard accelNoiseStdDev.isFinite else { throw ValidationError.nonFinite("accelNoiseStdDev") }
        guard accelBias.isFinite else { throw ValidationError.nonFinite("accelBias") }
        guard accelRandomWalkSigma.isFinite else { throw ValidationError.nonFinite("accelRandomWalkSigma") }

        guard gyroNoiseStdDev >= 0 else { throw ValidationError.negative("gyroNoiseStdDev") }
        guard gyroRandomWalkSigma >= 0 else { throw ValidationError.negative("gyroRandomWalkSigma") }
        guard accelNoiseStdDev >= 0 else { throw ValidationError.negative("accelNoiseStdDev") }
        guard accelRandomWalkSigma >= 0 else { throw ValidationError.negative("accelRandomWalkSigma") }

        self.gyroNoiseStdDev = gyroNoiseStdDev
        self.gyroBias = gyroBias
        self.gyroRandomWalkSigma = gyroRandomWalkSigma
        self.accelNoiseStdDev = accelNoiseStdDev
        self.accelBias = accelBias
        self.accelRandomWalkSigma = accelRandomWalkSigma
        self.delaySteps = delaySteps
    }

    public static let zero = IMU6NoiseConfig(
        uncheckedGyroNoiseStdDev: 0,
        uncheckedGyroBias: 0,
        uncheckedGyroRandomWalkSigma: 0,
        uncheckedAccelNoiseStdDev: 0,
        uncheckedAccelBias: 0,
        uncheckedAccelRandomWalkSigma: 0,
        uncheckedDelaySteps: 0
    )

    private enum CodingKeys: String, CodingKey {
        case gyroNoiseStdDev
        case gyroBias
        case gyroRandomWalkSigma
        case accelNoiseStdDev
        case accelBias
        case accelRandomWalkSigma
        case delaySteps
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            gyroNoiseStdDev: try container.decode(Double.self, forKey: .gyroNoiseStdDev),
            gyroBias: try container.decode(Double.self, forKey: .gyroBias),
            gyroRandomWalkSigma: try container.decode(Double.self, forKey: .gyroRandomWalkSigma),
            accelNoiseStdDev: try container.decode(Double.self, forKey: .accelNoiseStdDev),
            accelBias: try container.decode(Double.self, forKey: .accelBias),
            accelRandomWalkSigma: try container.decode(Double.self, forKey: .accelRandomWalkSigma),
            delaySteps: try container.decode(UInt64.self, forKey: .delaySteps)
        )
    }

    private init(
        uncheckedGyroNoiseStdDev gyroNoiseStdDev: Double,
        uncheckedGyroBias gyroBias: Double,
        uncheckedGyroRandomWalkSigma gyroRandomWalkSigma: Double,
        uncheckedAccelNoiseStdDev accelNoiseStdDev: Double,
        uncheckedAccelBias accelBias: Double,
        uncheckedAccelRandomWalkSigma accelRandomWalkSigma: Double,
        uncheckedDelaySteps delaySteps: UInt64
    ) {
        self.gyroNoiseStdDev = gyroNoiseStdDev
        self.gyroBias = gyroBias
        self.gyroRandomWalkSigma = gyroRandomWalkSigma
        self.accelNoiseStdDev = accelNoiseStdDev
        self.accelBias = accelBias
        self.accelRandomWalkSigma = accelRandomWalkSigma
        self.delaySteps = delaySteps
    }
}
