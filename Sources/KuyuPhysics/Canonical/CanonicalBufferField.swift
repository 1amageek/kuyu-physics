public struct CanonicalBufferField: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case invalidID(String)
        case invalidOffset(Int)
        case invalidUnit(String)
    }

    public let id: String
    public let offset: Int
    public let shape: CanonicalValueShape
    public let unit: String
    public let role: CanonicalValueRole
    public let differentiability: CanonicalDifferentiability

    public var elementCount: Int {
        shape.elementCount
    }

    public init(
        id: String,
        offset: Int,
        shape: CanonicalValueShape,
        unit: String,
        role: CanonicalValueRole,
        differentiability: CanonicalDifferentiability
    ) throws {
        guard CanonicalIdentifier.isValid(id) else {
            throw ValidationError.invalidID(id)
        }
        guard offset >= 0 else {
            throw ValidationError.invalidOffset(offset)
        }
        guard !unit.isEmpty, !unit.utf8.contains(where: { $0 <= 32 || $0 >= 127 }) else {
            throw ValidationError.invalidUnit(unit)
        }

        self.id = id
        self.offset = offset
        self.shape = shape
        self.unit = unit
        self.role = role
        self.differentiability = differentiability
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            id: container.decode(String.self, forKey: .id),
            offset: container.decode(Int.self, forKey: .offset),
            shape: container.decode(CanonicalValueShape.self, forKey: .shape),
            unit: container.decode(String.self, forKey: .unit),
            role: container.decode(CanonicalValueRole.self, forKey: .role),
            differentiability: container.decode(CanonicalDifferentiability.self, forKey: .differentiability)
        )
    }

    private enum CodingKeys: String, CodingKey {
        case id
        case offset
        case shape
        case unit
        case role
        case differentiability
    }
}
