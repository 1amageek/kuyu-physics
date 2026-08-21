public struct CanonicalDimension: Sendable, Codable, Equatable, Hashable {
    public let mass: Int
    public let length: Int
    public let time: Int
    public let temperature: Int

    public init(mass: Int = 0, length: Int = 0, time: Int = 0, temperature: Int = 0) {
        self.mass = mass
        self.length = length
        self.time = time
        self.temperature = temperature
    }

    public static let dimensionless = CanonicalDimension()

    public static func * (lhs: Self, rhs: Self) -> Self {
        CanonicalDimension(
            mass: lhs.mass + rhs.mass,
            length: lhs.length + rhs.length,
            time: lhs.time + rhs.time,
            temperature: lhs.temperature + rhs.temperature
        )
    }

    public static func / (lhs: Self, rhs: Self) -> Self {
        CanonicalDimension(
            mass: lhs.mass - rhs.mass,
            length: lhs.length - rhs.length,
            time: lhs.time - rhs.time,
            temperature: lhs.temperature - rhs.temperature
        )
    }
}
