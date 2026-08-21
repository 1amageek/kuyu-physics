public enum CanonicalDifferentiability: String, Sendable, Codable, Equatable, CaseIterable {
    case differentiable
    case piecewiseDifferentiable
    case nonDifferentiable

    static func combining(_ values: some Sequence<Self>) -> Self {
        values.reduce(.differentiable) { current, next in
            switch (current, next) {
            case (.nonDifferentiable, _), (_, .nonDifferentiable):
                .nonDifferentiable
            case (.piecewiseDifferentiable, _), (_, .piecewiseDifferentiable):
                .piecewiseDifferentiable
            case (.differentiable, .differentiable):
                .differentiable
            }
        }
    }
}
