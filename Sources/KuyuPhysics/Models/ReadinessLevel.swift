public enum ReadinessLevel: String, Sendable, Codable, Equatable, Comparable {
    case visualPreview
    case kinematicPreview
    case dynamicSimulation
    case contactTraining
    case hardwareParity

    public static func < (lhs: ReadinessLevel, rhs: ReadinessLevel) -> Bool {
        lhs.rank < rhs.rank
    }

    private var rank: Int {
        switch self {
        case .visualPreview:
            return 0
        case .kinematicPreview:
            return 1
        case .dynamicSimulation:
            return 2
        case .contactTraining:
            return 3
        case .hardwareParity:
            return 4
        }
    }
}
