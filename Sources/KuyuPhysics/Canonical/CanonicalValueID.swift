public struct CanonicalValueID: Sendable, Codable, Equatable, Hashable, CustomStringConvertible {
    public enum ValidationError: Error, Equatable {
        case invalidValue(String)
    }

    public let rawValue: String

    public init(_ rawValue: String) throws {
        guard CanonicalIdentifier.isValid(rawValue) else {
            throw ValidationError.invalidValue(rawValue)
        }
        self.rawValue = rawValue
    }

    public var description: String {
        rawValue
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.singleValueContainer()
        try self.init(container.decode(String.self))
    }

    public func encode(to encoder: Encoder) throws {
        var container = encoder.singleValueContainer()
        try container.encode(rawValue)
    }
}
