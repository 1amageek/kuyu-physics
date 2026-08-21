public protocol CanonicalDynamicsProgramValidating: Sendable {
    func validate(_ content: CanonicalDynamicsProgramContent) throws
}
