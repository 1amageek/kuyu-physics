public struct ReferenceQuadrotorFidelity: Sendable, Equatable {
    public enum ValidationError: Error, Equatable {
        case overlappingTerms(String)
        case incompletePartition
    }

    public let active: Set<QuadrotorForceTermID>
    public let worldModelTargets: Set<QuadrotorForceTermID>
    public let ignoredByNegligibilityPolicy: Set<QuadrotorForceTermID>
    public let activeMask: QuadrotorForceTermMask
    public let worldModelTargetMask: QuadrotorForceTermMask
    public let ignoredByNegligibilityPolicyMask: QuadrotorForceTermMask
    public let constraint: QuadrotorConstraintProjection

    public init(
        active: Set<QuadrotorForceTermID>,
        worldModelTargets: Set<QuadrotorForceTermID> = [],
        ignoredByNegligibilityPolicy: Set<QuadrotorForceTermID> = [],
        constraint: QuadrotorConstraintProjection
    ) throws {
        try Self.validate(
            active: active,
            worldModelTargets: worldModelTargets,
            ignoredByNegligibilityPolicy: ignoredByNegligibilityPolicy
        )
        self.active = active
        self.worldModelTargets = worldModelTargets
        self.ignoredByNegligibilityPolicy = ignoredByNegligibilityPolicy
        self.activeMask = QuadrotorForceTermMask(active)
        self.worldModelTargetMask = QuadrotorForceTermMask(worldModelTargets)
        self.ignoredByNegligibilityPolicyMask = QuadrotorForceTermMask(ignoredByNegligibilityPolicy)
        self.constraint = constraint
    }

    private init(
        uncheckedActive active: Set<QuadrotorForceTermID>,
        worldModelTargets: Set<QuadrotorForceTermID> = [],
        ignoredByNegligibilityPolicy: Set<QuadrotorForceTermID> = [],
        constraint: QuadrotorConstraintProjection
    ) {
        self.active = active
        self.worldModelTargets = worldModelTargets
        self.ignoredByNegligibilityPolicy = ignoredByNegligibilityPolicy
        self.activeMask = QuadrotorForceTermMask(active)
        self.worldModelTargetMask = QuadrotorForceTermMask(worldModelTargets)
        self.ignoredByNegligibilityPolicyMask = QuadrotorForceTermMask(ignoredByNegligibilityPolicy)
        self.constraint = constraint
    }

    public static let full = ReferenceQuadrotorFidelity(
        uncheckedActive: Set(QuadrotorForceTermID.allCases),
        constraint: .free
    )

    public static let singleProp = ReferenceQuadrotorFidelity(
        uncheckedActive: [.gravity, .propulsion, .disturbance],
        worldModelTargets: Set(QuadrotorForceTermID.allCases).subtracting([.gravity, .propulsion, .disturbance]),
        constraint: .verticalOnly
    )

    public func residualTargetIDs(toward high: ReferenceQuadrotorFidelity) -> Set<QuadrotorForceTermID> {
        residualTargetMask(toward: high).ids
    }

    public func residualTargetMask(toward high: ReferenceQuadrotorFidelity) -> QuadrotorForceTermMask {
        worldModelTargetMask.intersection(high.activeMask)
    }

    private static func validate(
        active: Set<QuadrotorForceTermID>,
        worldModelTargets: Set<QuadrotorForceTermID>,
        ignoredByNegligibilityPolicy: Set<QuadrotorForceTermID>
    ) throws {
        if !active.isDisjoint(with: worldModelTargets) {
            throw ValidationError.overlappingTerms("active.worldModelTargets")
        }
        if !active.isDisjoint(with: ignoredByNegligibilityPolicy) {
            throw ValidationError.overlappingTerms("active.ignoredByNegligibilityPolicy")
        }
        if !worldModelTargets.isDisjoint(with: ignoredByNegligibilityPolicy) {
            throw ValidationError.overlappingTerms("worldModelTargets.ignoredByNegligibilityPolicy")
        }
        let partition = active.union(worldModelTargets).union(ignoredByNegligibilityPolicy)
        guard partition == Set(QuadrotorForceTermID.allCases) else {
            throw ValidationError.incompletePartition
        }
    }
}
