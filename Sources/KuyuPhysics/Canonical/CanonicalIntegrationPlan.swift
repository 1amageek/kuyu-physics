public struct CanonicalIntegrationPlan: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case duplicateProjectionStage(CanonicalIntegrationStage)
        case incompleteRungeKutta4ProjectionStages
    }

    public let scheme: CanonicalIntegrationScheme
    public let projectionStages: [CanonicalIntegrationStage]

    public init(
        scheme: CanonicalIntegrationScheme,
        projectionStages: [CanonicalIntegrationStage]
    ) throws {
        var unique = Set<CanonicalIntegrationStage>()
        for stage in projectionStages where !unique.insert(stage).inserted {
            throw ValidationError.duplicateProjectionStage(stage)
        }
        if scheme == .rungeKutta4,
           projectionStages != CanonicalIntegrationStage.allCases {
            throw ValidationError.incompleteRungeKutta4ProjectionStages
        }
        self.scheme = scheme
        self.projectionStages = projectionStages
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            scheme: container.decode(CanonicalIntegrationScheme.self, forKey: .scheme),
            projectionStages: container.decode([CanonicalIntegrationStage].self, forKey: .projectionStages)
        )
    }

    private enum CodingKeys: String, CodingKey {
        case scheme
        case projectionStages
    }
}
