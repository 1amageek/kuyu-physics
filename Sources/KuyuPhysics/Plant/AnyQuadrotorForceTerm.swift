public struct AnyQuadrotorForceTerm: QuadrotorForceTerm {
    public let id: QuadrotorForceTermID
    public let mask: QuadrotorForceTermMask
    public let stiffness: ForceTermStiffness

    private let contribution: @Sendable (QuadrotorForceTermContext) -> QuadrotorGeneralizedForce

    public init(
        id: QuadrotorForceTermID,
        stiffness: ForceTermStiffness = .explicit,
        contribution: @escaping @Sendable (QuadrotorForceTermContext) -> QuadrotorGeneralizedForce
    ) {
        self.id = id
        self.mask = id.mask
        self.stiffness = stiffness
        self.contribution = contribution
    }

    public func generalizedForce(context: QuadrotorForceTermContext) -> QuadrotorGeneralizedForce {
        contribution(context)
    }
}
