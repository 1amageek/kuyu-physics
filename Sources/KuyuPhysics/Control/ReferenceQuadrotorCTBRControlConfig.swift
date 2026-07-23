import KuyuCore

public struct ReferenceQuadrotorCTBRControlConfig: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case nonFinite(String)
        case nonPositive(String)
    }

    public static let canonical = ReferenceQuadrotorCTBRControlConfig(
        uncheckedBodyRateScale: Axis3(x: 6.0, y: 6.0, z: 3.0),
        uncheckedTorqueGain: Axis3(x: 0.035, y: 0.035, z: 0.018)
    )

    public let bodyRateScale: Axis3
    public let torqueGain: Axis3

    public init(bodyRateScale: Axis3, torqueGain: Axis3) throws {
        guard bodyRateScale.isFinite else {
            throw ValidationError.nonFinite("bodyRateScale")
        }
        guard torqueGain.isFinite else {
            throw ValidationError.nonFinite("torqueGain")
        }
        guard bodyRateScale.x > 0, bodyRateScale.y > 0, bodyRateScale.z > 0 else {
            throw ValidationError.nonPositive("bodyRateScale")
        }
        guard torqueGain.x > 0, torqueGain.y > 0, torqueGain.z > 0 else {
            throw ValidationError.nonPositive("torqueGain")
        }

        self.bodyRateScale = bodyRateScale
        self.torqueGain = torqueGain
    }

    private enum CodingKeys: String, CodingKey {
        case bodyRateScale
        case torqueGain
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            bodyRateScale: container.decode(Axis3.self, forKey: .bodyRateScale),
            torqueGain: container.decode(Axis3.self, forKey: .torqueGain)
        )
    }

    private init(
        uncheckedBodyRateScale bodyRateScale: Axis3,
        uncheckedTorqueGain torqueGain: Axis3
    ) {
        self.bodyRateScale = bodyRateScale
        self.torqueGain = torqueGain
    }
}
