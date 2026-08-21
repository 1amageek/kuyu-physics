public struct CanonicalOperationGraph: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case invalidID(String)
        case emptyOutputs
    }

    public let id: String
    public let inputs: [CanonicalGraphInput]
    public let instructions: [CanonicalInstruction]
    public let outputs: [CanonicalGraphOutput]

    public init(
        id: String,
        inputs: [CanonicalGraphInput],
        instructions: [CanonicalInstruction],
        outputs: [CanonicalGraphOutput]
    ) throws {
        guard CanonicalIdentifier.isValid(id) else {
            throw ValidationError.invalidID(id)
        }
        guard !outputs.isEmpty else {
            throw ValidationError.emptyOutputs
        }
        self.id = id
        self.inputs = inputs
        self.instructions = instructions
        self.outputs = outputs
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            id: container.decode(String.self, forKey: .id),
            inputs: container.decode([CanonicalGraphInput].self, forKey: .inputs),
            instructions: container.decode([CanonicalInstruction].self, forKey: .instructions),
            outputs: container.decode([CanonicalGraphOutput].self, forKey: .outputs)
        )
    }

    private enum CodingKeys: String, CodingKey {
        case id
        case inputs
        case instructions
        case outputs
    }
}
