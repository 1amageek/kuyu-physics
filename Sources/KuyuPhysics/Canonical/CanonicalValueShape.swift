public struct CanonicalValueShape: Sendable, Codable, Equatable {
    public enum Kind: String, Sendable, Codable, Equatable {
        case scalar
        case vector
        case quaternion
    }

    public enum ValidationError: Error, Equatable {
        case invalidVectorLength(Int)
        case unexpectedLength(kind: String)
    }

    public let kind: Kind
    public let vectorLength: Int?

    public static let scalar = CanonicalValueShape(
        uncheckedKind: .scalar,
        vectorLength: nil
    )
    public static let vector2 = CanonicalValueShape(
        uncheckedKind: .vector,
        vectorLength: 2
    )
    public static let vector3 = CanonicalValueShape(
        uncheckedKind: .vector,
        vectorLength: 3
    )
    public static let vector4 = CanonicalValueShape(
        uncheckedKind: .vector,
        vectorLength: 4
    )
    public static let quaternion = CanonicalValueShape(
        uncheckedKind: .quaternion,
        vectorLength: nil
    )

    public static func vector(_ length: Int) throws -> Self {
        guard length > 1 else {
            throw ValidationError.invalidVectorLength(length)
        }
        return CanonicalValueShape(
            uncheckedKind: .vector,
            vectorLength: length
        )
    }

    public var elementCount: Int {
        switch kind {
        case .scalar:
            1
        case .vector:
            vectorLength ?? 0
        case .quaternion:
            4
        }
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        let kind = try container.decode(Kind.self, forKey: .kind)
        switch kind {
        case .scalar, .quaternion:
            guard !container.contains(.length) else {
                throw ValidationError.unexpectedLength(kind: kind.rawValue)
            }
            self.init(uncheckedKind: kind, vectorLength: nil)
        case .vector:
            self = try .vector(container.decode(Int.self, forKey: .length))
        }
    }

    public func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(kind, forKey: .kind)
        if let vectorLength {
            try container.encode(vectorLength, forKey: .length)
        }
    }

    private init(uncheckedKind kind: Kind, vectorLength: Int?) {
        self.kind = kind
        self.vectorLength = vectorLength
    }

    private enum CodingKeys: String, CodingKey {
        case kind
        case length
    }
}
