public struct CanonicalBufferLayout: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case invalidID(String)
        case invalidVersion(Int)
        case emptyFields
        case duplicateFieldID(String)
        case nonContiguousField(fieldID: String, expectedOffset: Int, actualOffset: Int)
        case elementCountOverflow(fieldID: String)
    }

    public let id: String
    public let version: Int
    public let fields: [CanonicalBufferField]

    public var elementCount: Int {
        guard let lastField = fields.last else {
            return 0
        }
        return lastField.offset + lastField.elementCount
    }

    public init(
        id: String,
        version: Int,
        fields: [CanonicalBufferField]
    ) throws {
        guard CanonicalIdentifier.isValid(id) else {
            throw ValidationError.invalidID(id)
        }
        guard version > 0 else {
            throw ValidationError.invalidVersion(version)
        }
        guard !fields.isEmpty else {
            throw ValidationError.emptyFields
        }

        var fieldIDs = Set<String>()
        var nextOffset = 0
        for field in fields {
            guard fieldIDs.insert(field.id).inserted else {
                throw ValidationError.duplicateFieldID(field.id)
            }
            guard field.offset == nextOffset else {
                throw ValidationError.nonContiguousField(
                    fieldID: field.id,
                    expectedOffset: nextOffset,
                    actualOffset: field.offset
                )
            }
            let (endOffset, overflowed) = nextOffset.addingReportingOverflow(field.elementCount)
            guard !overflowed else {
                throw ValidationError.elementCountOverflow(fieldID: field.id)
            }
            nextOffset = endOffset
        }

        self.id = id
        self.version = version
        self.fields = fields
    }

    public func field(named id: String) -> CanonicalBufferField? {
        fields.first { $0.id == id }
    }

    public func range(ofFieldNamed id: String) -> Range<Int>? {
        guard let field = field(named: id) else {
            return nil
        }
        return field.offset..<(field.offset + field.elementCount)
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            id: container.decode(String.self, forKey: .id),
            version: container.decode(Int.self, forKey: .version),
            fields: container.decode([CanonicalBufferField].self, forKey: .fields)
        )
    }

    private enum CodingKeys: String, CodingKey {
        case id
        case version
        case fields
    }
}
