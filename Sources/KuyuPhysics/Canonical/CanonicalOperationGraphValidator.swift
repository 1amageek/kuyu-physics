public struct CanonicalOperationGraphValidator: CanonicalOperationGraphValidating, Sendable {
    public enum ValidationError: Error, Equatable {
        case duplicateLayoutID(String)
        case duplicateValueID(CanonicalValueID)
        case duplicateOutputID(String)
        case missingLayout(String)
        case missingField(layoutID: String, fieldID: String)
        case unsupportedUnit(String)
        case unknownOperand(CanonicalValueID)
        case invalidOperandCount(opcode: CanonicalOpcode, expected: Int, actual: Int)
        case invalidConstantMetadata(CanonicalValueID)
        case invalidConstantCount(CanonicalValueID, expected: Int, actual: Int)
        case invalidComponentMetadata(CanonicalValueID)
        case unexpectedMetadata(CanonicalValueID)
        case shapeMismatch(CanonicalValueID)
        case dimensionMismatch(CanonicalValueID)
        case missingOutputValue(CanonicalValueID)
        case outputSignatureMismatch(String)
    }

    public init() {}

    public func signatures(
        for graph: CanonicalOperationGraph,
        layouts: [CanonicalBufferLayout]
    ) throws -> [CanonicalValueID: CanonicalValueSignature] {
        let layoutsByID = try indexedLayouts(layouts)
        var signatures: [CanonicalValueID: CanonicalValueSignature] = [:]

        for input in graph.inputs {
            guard signatures[input.id] == nil else {
                throw ValidationError.duplicateValueID(input.id)
            }
            guard let layout = layoutsByID[input.layoutID] else {
                throw ValidationError.missingLayout(input.layoutID)
            }
            guard let field = layout.field(named: input.fieldID) else {
                throw ValidationError.missingField(layoutID: input.layoutID, fieldID: input.fieldID)
            }
            guard let unit = CanonicalUnit(rawValue: field.unit) else {
                throw ValidationError.unsupportedUnit(field.unit)
            }
            signatures[input.id] = CanonicalValueSignature(
                shape: field.shape,
                dimension: unit.dimension,
                differentiability: field.differentiability
            )
        }

        for instruction in graph.instructions {
            guard signatures[instruction.result] == nil else {
                throw ValidationError.duplicateValueID(instruction.result)
            }
            let operands = try instruction.operands.map { operand -> CanonicalValueSignature in
                guard let signature = signatures[operand] else {
                    throw ValidationError.unknownOperand(operand)
                }
                return signature
            }
            signatures[instruction.result] = try signature(for: instruction, operands: operands)
        }

        var outputIDs = Set<String>()
        for output in graph.outputs {
            guard outputIDs.insert(output.id).inserted else {
                throw ValidationError.duplicateOutputID(output.id)
            }
            guard let signature = signatures[output.value] else {
                throw ValidationError.missingOutputValue(output.value)
            }
            guard signature.shape == output.shape,
                  signature.dimension == output.unit.dimension else {
                throw ValidationError.outputSignatureMismatch(output.id)
            }
        }
        return signatures
    }

    private func indexedLayouts(
        _ layouts: [CanonicalBufferLayout]
    ) throws -> [String: CanonicalBufferLayout] {
        var result: [String: CanonicalBufferLayout] = [:]
        for layout in layouts {
            guard result.updateValue(layout, forKey: layout.id) == nil else {
                throw ValidationError.duplicateLayoutID(layout.id)
            }
        }
        return result
    }

    private func signature(
        for instruction: CanonicalInstruction,
        operands: [CanonicalValueSignature]
    ) throws -> CanonicalValueSignature {
        switch instruction.opcode {
        case .constant:
            try requireOperandCount(0, instruction: instruction)
            guard let shape = instruction.constantShape,
                  let unit = instruction.constantUnit,
                  instruction.componentIndex == nil else {
                throw ValidationError.invalidConstantMetadata(instruction.result)
            }
            guard instruction.constants.count == shape.elementCount else {
                throw ValidationError.invalidConstantCount(
                    instruction.result,
                    expected: shape.elementCount,
                    actual: instruction.constants.count
                )
            }
            return CanonicalValueSignature(
                shape: shape,
                dimension: unit.dimension,
                differentiability: .differentiable
            )
        case .add, .subtract:
            try requireNoMetadata(instruction)
            try requireOperandCount(2, instruction: instruction)
            guard operands[0].shape == operands[1].shape else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            guard operands[0].dimension == operands[1].dimension else {
                throw ValidationError.dimensionMismatch(instruction.result)
            }
            return CanonicalValueSignature(
                shape: operands[0].shape,
                dimension: operands[0].dimension,
                differentiability: .combining(operands.map(\.differentiability))
            )
        case .multiply:
            try requireNoMetadata(instruction)
            try requireOperandCount(2, instruction: instruction)
            let shape: CanonicalValueShape
            if operands[0].shape == .scalar {
                shape = operands[1].shape
            } else if operands[1].shape == .scalar {
                shape = operands[0].shape
            } else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            return CanonicalValueSignature(
                shape: shape,
                dimension: operands[0].dimension * operands[1].dimension,
                differentiability: .combining(operands.map(\.differentiability))
            )
        case .multiplyComponents, .divideComponents:
            try requireNoMetadata(instruction)
            try requireOperandCount(2, instruction: instruction)
            guard operands[0].shape == .vector3, operands[1].shape == .vector3 else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            let dimension = instruction.opcode == .multiplyComponents
                ? operands[0].dimension * operands[1].dimension
                : operands[0].dimension / operands[1].dimension
            return CanonicalValueSignature(
                shape: .vector3,
                dimension: dimension,
                differentiability: .combining(operands.map(\.differentiability))
            )
        case .divide:
            try requireNoMetadata(instruction)
            try requireOperandCount(2, instruction: instruction)
            guard operands[1].shape == .scalar else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            return CanonicalValueSignature(
                shape: operands[0].shape,
                dimension: operands[0].dimension / operands[1].dimension,
                differentiability: .combining(operands.map(\.differentiability))
            )
        case .negate:
            try requireNoMetadata(instruction)
            try requireOperandCount(1, instruction: instruction)
            return operands[0]
        case .component:
            guard instruction.constants.isEmpty,
                  instruction.constantShape == nil,
                  instruction.constantUnit == nil,
                  let index = instruction.componentIndex else {
                throw ValidationError.invalidComponentMetadata(instruction.result)
            }
            try requireOperandCount(1, instruction: instruction)
            guard index < operands[0].shape.elementCount,
                  operands[0].shape == .vector3
                    || operands[0].shape == .vector4
                    || operands[0].shape == .quaternion else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            return CanonicalValueSignature(
                shape: .scalar,
                dimension: operands[0].dimension,
                differentiability: operands[0].differentiability
            )
        case .composeVector3:
            try requireNoMetadata(instruction)
            try requireOperandCount(3, instruction: instruction)
            guard operands.allSatisfy({ $0.shape == .scalar }) else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            guard operands.dropFirst().allSatisfy({ $0.dimension == operands[0].dimension }) else {
                throw ValidationError.dimensionMismatch(instruction.result)
            }
            return CanonicalValueSignature(
                shape: .vector3,
                dimension: operands[0].dimension,
                differentiability: .combining(operands.map(\.differentiability))
            )
        case .cross3:
            try requireNoMetadata(instruction)
            try requireOperandCount(2, instruction: instruction)
            guard operands.allSatisfy({ $0.shape == .vector3 }) else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            return CanonicalValueSignature(
                shape: .vector3,
                dimension: operands[0].dimension * operands[1].dimension,
                differentiability: .combining(operands.map(\.differentiability))
            )
        case .length3:
            try requireNoMetadata(instruction)
            try requireOperandCount(1, instruction: instruction)
            guard operands[0].shape == .vector3 else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            return CanonicalValueSignature(
                shape: .scalar,
                dimension: operands[0].dimension,
                differentiability: .combining([
                    operands[0].differentiability,
                    .piecewiseDifferentiable,
                ])
            )
        case .normalize3OrZero:
            try requireNoMetadata(instruction)
            try requireOperandCount(1, instruction: instruction)
            guard operands[0].shape == .vector3 else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            return CanonicalValueSignature(
                shape: .vector3,
                dimension: .dimensionless,
                differentiability: .combining([
                    operands[0].differentiability,
                    .piecewiseDifferentiable,
                ])
            )
        case .quaternionRotate3, .quaternionInverseRotate3:
            try requireNoMetadata(instruction)
            try requireOperandCount(2, instruction: instruction)
            guard operands[0].shape == .quaternion,
                  operands[0].dimension == .dimensionless,
                  operands[1].shape == .vector3 else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            return CanonicalValueSignature(
                shape: .vector3,
                dimension: operands[1].dimension,
                differentiability: .combining(operands.map(\.differentiability))
            )
        case .quaternionDerivative:
            try requireNoMetadata(instruction)
            try requireOperandCount(2, instruction: instruction)
            guard operands[0].shape == .quaternion,
                  operands[0].dimension == .dimensionless,
                  operands[1].shape == .vector3 else {
                throw ValidationError.shapeMismatch(instruction.result)
            }
            guard operands[1].dimension == CanonicalUnit.radianPerSecond.dimension else {
                throw ValidationError.dimensionMismatch(instruction.result)
            }
            return CanonicalValueSignature(
                shape: .vector4,
                dimension: CanonicalUnit.inverseSecond.dimension,
                differentiability: .combining(operands.map(\.differentiability))
            )
        }
    }

    private func requireOperandCount(
        _ expected: Int,
        instruction: CanonicalInstruction
    ) throws {
        guard instruction.operands.count == expected else {
            throw ValidationError.invalidOperandCount(
                opcode: instruction.opcode,
                expected: expected,
                actual: instruction.operands.count
            )
        }
    }

    private func requireNoMetadata(_ instruction: CanonicalInstruction) throws {
        guard instruction.constants.isEmpty,
              instruction.constantShape == nil,
              instruction.constantUnit == nil,
              instruction.componentIndex == nil else {
            throw ValidationError.unexpectedMetadata(instruction.result)
        }
    }
}
