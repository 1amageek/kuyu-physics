public struct CanonicalInstruction: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case nonFiniteConstant(Double)
        case invalidComponentIndex(Int)
    }

    public let result: CanonicalValueID
    public let opcode: CanonicalOpcode
    public let operands: [CanonicalValueID]
    public let constants: [Double]
    public let constantShape: CanonicalValueShape?
    public let constantUnit: CanonicalUnit?
    public let componentIndex: Int?

    public init(
        result: CanonicalValueID,
        opcode: CanonicalOpcode,
        operands: [CanonicalValueID] = [],
        constants: [Double] = [],
        constantShape: CanonicalValueShape? = nil,
        constantUnit: CanonicalUnit? = nil,
        componentIndex: Int? = nil
    ) throws {
        if let nonFinite = constants.first(where: { !$0.isFinite }) {
            throw ValidationError.nonFiniteConstant(nonFinite)
        }
        if let componentIndex, !(0..<4).contains(componentIndex) {
            throw ValidationError.invalidComponentIndex(componentIndex)
        }
        self.result = result
        self.opcode = opcode
        self.operands = operands
        self.constants = constants
        self.constantShape = constantShape
        self.constantUnit = constantUnit
        self.componentIndex = componentIndex
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            result: container.decode(CanonicalValueID.self, forKey: .result),
            opcode: container.decode(CanonicalOpcode.self, forKey: .opcode),
            operands: container.decodeIfPresent([CanonicalValueID].self, forKey: .operands) ?? [],
            constants: container.decodeIfPresent([Double].self, forKey: .constants) ?? [],
            constantShape: container.decodeIfPresent(CanonicalValueShape.self, forKey: .constantShape),
            constantUnit: container.decodeIfPresent(CanonicalUnit.self, forKey: .constantUnit),
            componentIndex: container.decodeIfPresent(Int.self, forKey: .componentIndex)
        )
    }

    private enum CodingKeys: String, CodingKey {
        case result
        case opcode
        case operands
        case constants
        case constantShape
        case constantUnit
        case componentIndex
    }
}
