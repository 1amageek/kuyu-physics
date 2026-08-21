public struct CanonicalProgramDigest: Sendable, Codable, Equatable, Hashable {
    public enum ValidationError: Error, Equatable {
        case invalidFormat(String)
    }

    public let rawValue: String

    public init(_ rawValue: String) throws {
        guard rawValue.utf8.count == 64,
              rawValue.utf8.allSatisfy({ byte in
                  (48...57).contains(byte) || (97...102).contains(byte)
              })
        else {
            throw ValidationError.invalidFormat(rawValue)
        }
        self.rawValue = rawValue
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
