public struct CanonicalValueSignature: Sendable, Codable, Equatable {
    public let shape: CanonicalValueShape
    public let dimension: CanonicalDimension
    public let differentiability: CanonicalDifferentiability

    public init(
        shape: CanonicalValueShape,
        dimension: CanonicalDimension,
        differentiability: CanonicalDifferentiability
    ) {
        self.shape = shape
        self.dimension = dimension
        self.differentiability = differentiability
    }
}
