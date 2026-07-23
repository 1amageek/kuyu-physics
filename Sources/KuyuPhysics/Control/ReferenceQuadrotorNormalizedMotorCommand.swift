public struct ReferenceQuadrotorNormalizedMotorCommand: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case nonFinite(String)
        case outOfRange(String)
    }

    public let f1: Double
    public let f2: Double
    public let f3: Double
    public let f4: Double

    public var values: [Double] { [f1, f2, f3, f4] }

    public init(f1: Double, f2: Double, f3: Double, f4: Double) throws {
        let fields = [("f1", f1), ("f2", f2), ("f3", f3), ("f4", f4)]
        for (name, value) in fields {
            guard value.isFinite else { throw ValidationError.nonFinite(name) }
            guard (0.0...1.0).contains(value) else {
                throw ValidationError.outOfRange(name)
            }
        }

        self.f1 = f1
        self.f2 = f2
        self.f3 = f3
        self.f4 = f4
    }

    private enum CodingKeys: String, CodingKey {
        case f1
        case f2
        case f3
        case f4
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            f1: container.decode(Double.self, forKey: .f1),
            f2: container.decode(Double.self, forKey: .f2),
            f3: container.decode(Double.self, forKey: .f3),
            f4: container.decode(Double.self, forKey: .f4)
        )
    }
}
