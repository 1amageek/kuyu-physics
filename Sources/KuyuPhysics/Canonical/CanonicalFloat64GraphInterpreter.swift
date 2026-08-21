import simd

struct CanonicalFloat64GraphInterpreter: Sendable {
    func execute(
        _ graph: CanonicalOperationGraph,
        inputs: [CanonicalValueID: CanonicalFloat64Value]
    ) throws -> [String: CanonicalFloat64Value] {
        var values = inputs
        for input in graph.inputs where values[input.id] == nil {
            throw CanonicalFloat64ExecutionError.missingInput(input.id)
        }
        for instruction in graph.instructions {
            let operands = try instruction.operands.map { operand -> CanonicalFloat64Value in
                guard let value = values[operand] else {
                    throw CanonicalFloat64ExecutionError.missingOperand(operand)
                }
                return value
            }
            let value = try evaluate(instruction, operands: operands)
            guard value.isFinite else {
                throw CanonicalFloat64ExecutionError.nonFiniteResult(instruction.result)
            }
            values[instruction.result] = value
        }

        var outputs: [String: CanonicalFloat64Value] = [:]
        for output in graph.outputs {
            guard let value = values[output.value] else {
                throw CanonicalFloat64ExecutionError.missingOutput(
                    graphID: graph.id,
                    outputID: output.id
                )
            }
            outputs[output.id] = value
        }
        return outputs
    }

    private func evaluate(
        _ instruction: CanonicalInstruction,
        operands: [CanonicalFloat64Value]
    ) throws -> CanonicalFloat64Value {
        switch instruction.opcode {
        case .constant:
            guard let shape = instruction.constantShape else {
                throw CanonicalFloat64ExecutionError.unsupportedConstantShape(instruction.result)
            }
            switch shape {
            case .scalar:
                return .scalar(instruction.constants[0])
            case .vector3:
                return .vector3(SIMD3<Double>(instruction.constants[0], instruction.constants[1], instruction.constants[2]))
            case .vector4:
                return .vector4(
                    SIMD4<Double>(
                        instruction.constants[0],
                        instruction.constants[1],
                        instruction.constants[2],
                        instruction.constants[3]
                    )
                )
            case .quaternion:
                return .quaternion(
                    simd_quatd(
                        vector: SIMD4<Double>(
                            instruction.constants[0],
                            instruction.constants[1],
                            instruction.constants[2],
                            instruction.constants[3]
                        )
                    )
                )
            default:
                throw CanonicalFloat64ExecutionError.unsupportedConstantShape(instruction.result)
            }
        case .add:
            return try binarySameShape(instruction.result, operands, scalar: +, vector3: +, vector4: +)
        case .subtract:
            return try binarySameShape(instruction.result, operands, scalar: -, vector3: -, vector4: -)
        case .multiply:
            switch (operands[0], operands[1]) {
            case let (.scalar(lhs), .scalar(rhs)):
                return .scalar(lhs * rhs)
            case let (.scalar(lhs), .vector3(rhs)):
                return .vector3(lhs * rhs)
            case let (.vector3(lhs), .scalar(rhs)):
                return .vector3(lhs * rhs)
            case let (.scalar(lhs), .vector4(rhs)):
                return .vector4(lhs * rhs)
            case let (.vector4(lhs), .scalar(rhs)):
                return .vector4(lhs * rhs)
            default:
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
        case .multiplyComponents:
            guard case let .vector3(lhs) = operands[0],
                  case let .vector3(rhs) = operands[1] else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            return .vector3(lhs * rhs)
        case .divide:
            guard case let .scalar(rhs) = operands[1] else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            guard rhs != 0 else {
                throw CanonicalFloat64ExecutionError.divisionByZero(instruction.result)
            }
            switch operands[0] {
            case let .scalar(lhs):
                return .scalar(lhs / rhs)
            case let .vector3(lhs):
                return .vector3(lhs / rhs)
            case let .vector4(lhs):
                return .vector4(lhs / rhs)
            case .quaternion:
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
        case .divideComponents:
            guard case let .vector3(lhs) = operands[0],
                  case let .vector3(rhs) = operands[1] else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            guard rhs.x != 0, rhs.y != 0, rhs.z != 0 else {
                throw CanonicalFloat64ExecutionError.divisionByZero(instruction.result)
            }
            return .vector3(lhs / rhs)
        case .negate:
            switch operands[0] {
            case let .scalar(value):
                return .scalar(-value)
            case let .vector3(value):
                return .vector3(-value)
            case let .vector4(value):
                return .vector4(-value)
            case .quaternion:
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
        case .component:
            guard let index = instruction.componentIndex else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            switch operands[0] {
            case let .vector3(value):
                return .scalar(value[index])
            case let .vector4(value):
                return .scalar(value[index])
            case let .quaternion(value):
                return .scalar(value.vector[index])
            case .scalar:
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
        case .composeVector3:
            guard case let .scalar(x) = operands[0],
                  case let .scalar(y) = operands[1],
                  case let .scalar(z) = operands[2] else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            return .vector3(SIMD3<Double>(x, y, z))
        case .cross3:
            guard case let .vector3(lhs) = operands[0],
                  case let .vector3(rhs) = operands[1] else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            return .vector3(simd_cross(lhs, rhs))
        case .length3:
            guard case let .vector3(value) = operands[0] else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            return .scalar(simd_length(value))
        case .normalize3OrZero:
            guard case let .vector3(value) = operands[0] else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            let length = simd_length(value)
            return .vector3(length == 0 ? SIMD3<Double>(repeating: 0) : value / length)
        case .quaternionRotate3:
            guard case let .quaternion(quaternion) = operands[0],
                  case let .vector3(vector) = operands[1] else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            return .vector3(quaternion.act(vector))
        case .quaternionInverseRotate3:
            guard case let .quaternion(quaternion) = operands[0],
                  case let .vector3(vector) = operands[1] else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            return .vector3(quaternion.inverse.act(vector))
        case .quaternionDerivative:
            guard case let .quaternion(quaternion) = operands[0],
                  case let .vector3(angularVelocity) = operands[1] else {
                throw CanonicalFloat64ExecutionError.typeMismatch(instruction.result)
            }
            let omega = simd_quatd(real: 0, imag: angularVelocity)
            return .vector4((quaternion * omega).vector * 0.5)
        }
    }

    private func binarySameShape(
        _ result: CanonicalValueID,
        _ operands: [CanonicalFloat64Value],
        scalar: (Double, Double) -> Double,
        vector3: (SIMD3<Double>, SIMD3<Double>) -> SIMD3<Double>,
        vector4: (SIMD4<Double>, SIMD4<Double>) -> SIMD4<Double>
    ) throws -> CanonicalFloat64Value {
        switch (operands[0], operands[1]) {
        case let (.scalar(lhs), .scalar(rhs)):
            .scalar(scalar(lhs, rhs))
        case let (.vector3(lhs), .vector3(rhs)):
            .vector3(vector3(lhs, rhs))
        case let (.vector4(lhs), .vector4(rhs)):
            .vector4(vector4(lhs, rhs))
        default:
            throw CanonicalFloat64ExecutionError.typeMismatch(result)
        }
    }
}

enum CanonicalFloat64Value: Sendable {
    case scalar(Double)
    case vector3(SIMD3<Double>)
    case vector4(SIMD4<Double>)
    case quaternion(simd_quatd)

    var isFinite: Bool {
        switch self {
        case let .scalar(value):
            value.isFinite
        case let .vector3(value):
            value.x.isFinite && value.y.isFinite && value.z.isFinite
        case let .vector4(value):
            value.x.isFinite && value.y.isFinite && value.z.isFinite && value.w.isFinite
        case let .quaternion(value):
            value.vector.x.isFinite
                && value.vector.y.isFinite
                && value.vector.z.isFinite
                && value.vector.w.isFinite
        }
    }
}
