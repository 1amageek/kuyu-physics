public struct CanonicalDynamicsProgramContent: Sendable, Codable, Equatable {
    public let id: String
    public let schemaVersion: Int
    public let layouts: [CanonicalBufferLayout]
    public let layoutBindings: CanonicalDynamicsLayoutBindings
    public let controlSemantics: CanonicalControlSemantics
    public let forceTerms: [CanonicalForceTermProgram]
    public let derivativeGraph: CanonicalOperationGraph
    public let observableGraph: CanonicalOperationGraph
    public let fidelities: [CanonicalFidelityDefinition]
    public let integration: CanonicalIntegrationPlan

    public init(
        id: String,
        schemaVersion: Int,
        layouts: [CanonicalBufferLayout],
        layoutBindings: CanonicalDynamicsLayoutBindings,
        controlSemantics: CanonicalControlSemantics,
        forceTerms: [CanonicalForceTermProgram],
        derivativeGraph: CanonicalOperationGraph,
        observableGraph: CanonicalOperationGraph,
        fidelities: [CanonicalFidelityDefinition],
        integration: CanonicalIntegrationPlan
    ) {
        self.id = id
        self.schemaVersion = schemaVersion
        self.layouts = layouts
        self.layoutBindings = layoutBindings
        self.controlSemantics = controlSemantics
        self.forceTerms = forceTerms
        self.derivativeGraph = derivativeGraph
        self.observableGraph = observableGraph
        self.fidelities = fidelities
        self.integration = integration
    }
}
