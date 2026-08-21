public enum CanonicalValueRole: String, Sendable, Codable, Equatable, CaseIterable {
    case state
    case parameter
    case control
    case disturbance
    case temporary
    case observable
}
