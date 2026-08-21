public struct CanonicalGraphOutput: Sendable, Codable, Equatable {
    public let id: String
    public let value: CanonicalValueID
    public let shape: CanonicalValueShape
    public let unit: CanonicalUnit

    public init(
        id: String,
        value: CanonicalValueID,
        shape: CanonicalValueShape,
        unit: CanonicalUnit
    ) throws {
        guard CanonicalIdentifier.isValid(id) else {
            throw CanonicalValueID.ValidationError.invalidValue(id)
        }
        self.id = id
        self.value = value
        self.shape = shape
        self.unit = unit
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            id: container.decode(String.self, forKey: .id),
            value: container.decode(CanonicalValueID.self, forKey: .value),
            shape: container.decode(CanonicalValueShape.self, forKey: .shape),
            unit: container.decode(CanonicalUnit.self, forKey: .unit)
        )
    }

    private enum CodingKeys: String, CodingKey {
        case id
        case value
        case shape
        case unit
    }
}
