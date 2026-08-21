public struct CanonicalFidelityDefinition: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case invalidID(String)
        case duplicateTerm(CanonicalForceTermID)
        case overlappingPartition(CanonicalForceTermID)
    }

    public let id: String
    public let active: [CanonicalForceTermID]
    public let worldModelTargets: [CanonicalForceTermID]
    public let ignored: [CanonicalForceTermID]
    public let projection: CanonicalConstraintProjectionKind

    public init(
        id: String,
        active: [CanonicalForceTermID],
        worldModelTargets: [CanonicalForceTermID],
        ignored: [CanonicalForceTermID],
        projection: CanonicalConstraintProjectionKind
    ) throws {
        guard CanonicalIdentifier.isValid(id) else {
            throw ValidationError.invalidID(id)
        }
        var all = Set<CanonicalForceTermID>()
        for term in active + worldModelTargets + ignored {
            guard all.insert(term).inserted else {
                let isDuplicateWithinPartition = active.filter({ $0 == term }).count > 1
                    || worldModelTargets.filter({ $0 == term }).count > 1
                    || ignored.filter({ $0 == term }).count > 1
                throw isDuplicateWithinPartition
                    ? ValidationError.duplicateTerm(term)
                    : ValidationError.overlappingPartition(term)
            }
        }
        self.id = id
        self.active = active
        self.worldModelTargets = worldModelTargets
        self.ignored = ignored
        self.projection = projection
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            id: container.decode(String.self, forKey: .id),
            active: container.decode([CanonicalForceTermID].self, forKey: .active),
            worldModelTargets: container.decode([CanonicalForceTermID].self, forKey: .worldModelTargets),
            ignored: container.decode([CanonicalForceTermID].self, forKey: .ignored),
            projection: container.decode(CanonicalConstraintProjectionKind.self, forKey: .projection)
        )
    }

    private enum CodingKeys: String, CodingKey {
        case id
        case active
        case worldModelTargets
        case ignored
        case projection
    }
}
