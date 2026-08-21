public struct CanonicalGraphInput: Sendable, Codable, Equatable {
    public let id: CanonicalValueID
    public let layoutID: String
    public let fieldID: String

    public init(id: CanonicalValueID, layoutID: String, fieldID: String) throws {
        guard CanonicalIdentifier.isValid(layoutID) else {
            throw CanonicalValueID.ValidationError.invalidValue(layoutID)
        }
        guard CanonicalIdentifier.isValid(fieldID) else {
            throw CanonicalValueID.ValidationError.invalidValue(fieldID)
        }
        self.id = id
        self.layoutID = layoutID
        self.fieldID = fieldID
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            id: container.decode(CanonicalValueID.self, forKey: .id),
            layoutID: container.decode(String.self, forKey: .layoutID),
            fieldID: container.decode(String.self, forKey: .fieldID)
        )
    }

    private enum CodingKeys: String, CodingKey {
        case id
        case layoutID
        case fieldID
    }
}
