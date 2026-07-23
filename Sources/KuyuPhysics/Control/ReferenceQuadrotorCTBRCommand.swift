import KuyuCore

public struct ReferenceQuadrotorCTBRCommand: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case nonFinite(String)
    }

    public let collectiveThrust: Double
    public let normalizedBodyRate: Axis3

    public init(
        collectiveThrust: Double,
        normalizedBodyRate: Axis3
    ) throws {
        guard collectiveThrust.isFinite else {
            throw ValidationError.nonFinite("collectiveThrust")
        }
        guard normalizedBodyRate.isFinite else {
            throw ValidationError.nonFinite("normalizedBodyRate")
        }

        self.collectiveThrust = min(max(collectiveThrust, 0.0), 1.0)
        self.normalizedBodyRate = Axis3(
            x: min(max(normalizedBodyRate.x, -1.0), 1.0),
            y: min(max(normalizedBodyRate.y, -1.0), 1.0),
            z: min(max(normalizedBodyRate.z, -1.0), 1.0)
        )
    }

    private enum CodingKeys: String, CodingKey {
        case collectiveThrust
        case normalizedBodyRate
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            collectiveThrust: container.decode(Double.self, forKey: .collectiveThrust),
            normalizedBodyRate: container.decode(Axis3.self, forKey: .normalizedBodyRate)
        )
    }
}
