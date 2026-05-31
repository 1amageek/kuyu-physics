public struct QuadrotorForceTermMask: OptionSet, Sendable, Codable, Equatable {
    public let rawValue: UInt16

    public init(rawValue: UInt16) {
        self.rawValue = rawValue
    }

    public init(_ ids: Set<QuadrotorForceTermID>) {
        self = ids.reduce([]) { partial, id in
            partial.union(id.mask)
        }
    }

    public var ids: Set<QuadrotorForceTermID> {
        Set(orderedIDs)
    }

    public var orderedIDs: [QuadrotorForceTermID] {
        QuadrotorForceTermID.allCases.filter { contains($0.mask) }
    }

    public static let gravity = QuadrotorForceTermMask(rawValue: 1 << 0)
    public static let propulsion = QuadrotorForceTermMask(rawValue: 1 << 1)
    public static let thrustDensityScaling = QuadrotorForceTermMask(rawValue: 1 << 2)
    public static let disturbance = QuadrotorForceTermMask(rawValue: 1 << 3)
    public static let aerodynamicDrag = QuadrotorForceTermMask(rawValue: 1 << 4)
    public static let aerodynamicLift = QuadrotorForceTermMask(rawValue: 1 << 5)
    public static let buoyancy = QuadrotorForceTermMask(rawValue: 1 << 6)
    public static let angularDrag = QuadrotorForceTermMask(rawValue: 1 << 7)
    public static let gyroscopic = QuadrotorForceTermMask(rawValue: 1 << 8)

    public static let all = QuadrotorForceTermMask(Set(QuadrotorForceTermID.allCases))
}

public extension QuadrotorForceTermID {
    var mask: QuadrotorForceTermMask {
        switch self {
        case .gravity:
            return .gravity
        case .propulsion:
            return .propulsion
        case .thrustDensityScaling:
            return .thrustDensityScaling
        case .disturbance:
            return .disturbance
        case .aerodynamicDrag:
            return .aerodynamicDrag
        case .aerodynamicLift:
            return .aerodynamicLift
        case .buoyancy:
            return .buoyancy
        case .angularDrag:
            return .angularDrag
        case .gyroscopic:
            return .gyroscopic
        }
    }
}
