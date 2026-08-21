public struct CanonicalForceTermProgram: Sendable, Codable, Equatable {
    public let id: CanonicalForceTermID
    public let stiffness: ForceTermStiffness
    public let graph: CanonicalOperationGraph

    public init(
        id: CanonicalForceTermID,
        stiffness: ForceTermStiffness = .explicit,
        graph: CanonicalOperationGraph
    ) {
        self.id = id
        self.stiffness = stiffness
        self.graph = graph
    }
}
