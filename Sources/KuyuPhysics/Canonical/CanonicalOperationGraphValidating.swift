public protocol CanonicalOperationGraphValidating: Sendable {
    func signatures(
        for graph: CanonicalOperationGraph,
        layouts: [CanonicalBufferLayout]
    ) throws -> [CanonicalValueID: CanonicalValueSignature]
}
