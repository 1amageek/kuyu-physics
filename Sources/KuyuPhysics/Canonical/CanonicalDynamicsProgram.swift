public struct CanonicalDynamicsProgram: Sendable, Codable, Equatable {
    public static let currentSchemaVersion = 1

    public enum ValidationError: Error, Equatable {
        case digestMismatch(expected: CanonicalProgramDigest, actual: CanonicalProgramDigest)
    }

    public let content: CanonicalDynamicsProgramContent
    public let digest: CanonicalProgramDigest

    public init(content: CanonicalDynamicsProgramContent) throws {
        try CanonicalDynamicsProgramValidator().validate(content)
        self.content = content
        self.digest = try CanonicalProgramDigest.canonicalSHA256(of: content)
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        let content = try container.decode(CanonicalDynamicsProgramContent.self, forKey: .content)
        let expectedDigest = try container.decode(CanonicalProgramDigest.self, forKey: .digest)
        try CanonicalDynamicsProgramValidator().validate(content)
        let actualDigest = try CanonicalProgramDigest.canonicalSHA256(of: content)
        guard expectedDigest == actualDigest else {
            throw ValidationError.digestMismatch(expected: expectedDigest, actual: actualDigest)
        }
        self.content = content
        self.digest = actualDigest
    }

    private enum CodingKeys: String, CodingKey {
        case content
        case digest
    }
}
