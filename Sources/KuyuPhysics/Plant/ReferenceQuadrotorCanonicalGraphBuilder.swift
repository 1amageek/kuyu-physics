struct ReferenceQuadrotorCanonicalGraphBuilder {
    let id: String
    private(set) var inputs: [CanonicalGraphInput] = []
    private(set) var instructions: [CanonicalInstruction] = []
    private var inputIDs: [String: CanonicalValueID] = [:]

    init(id: String) {
        self.id = id
    }

    mutating func input(
        _ layout: CanonicalBufferLayout,
        _ fieldID: String
    ) throws -> CanonicalValueID {
        let key = "\(layout.id).\(fieldID)"
        if let existing = inputIDs[key] {
            return existing
        }
        let valueID = try CanonicalValueID(key)
        inputs.append(
            try CanonicalGraphInput(
                id: valueID,
                layoutID: layout.id,
                fieldID: fieldID
            )
        )
        inputIDs[key] = valueID
        return valueID
    }

    mutating func constant(
        _ values: [Double],
        shape: CanonicalValueShape,
        unit: CanonicalUnit,
        id: String
    ) throws -> CanonicalValueID {
        let valueID = try CanonicalValueID(id)
        instructions.append(
            try CanonicalInstruction(
                result: valueID,
                opcode: .constant,
                constants: values,
                constantShape: shape,
                constantUnit: unit
            )
        )
        return valueID
    }

    mutating func zeroVector(
        unit: CanonicalUnit,
        id: String
    ) throws -> CanonicalValueID {
        try constant([0, 0, 0], shape: .vector3, unit: unit, id: id)
    }

    mutating func component(
        _ operand: CanonicalValueID,
        index: Int,
        id: String
    ) throws -> CanonicalValueID {
        let result = try CanonicalValueID(id)
        instructions.append(
            try CanonicalInstruction(
                result: result,
                opcode: .component,
                operands: [operand],
                componentIndex: index
            )
        )
        return result
    }

    mutating func operation(
        _ opcode: CanonicalOpcode,
        _ operands: [CanonicalValueID],
        _ id: String
    ) throws -> CanonicalValueID {
        let result = try CanonicalValueID(id)
        instructions.append(
            try CanonicalInstruction(
                result: result,
                opcode: opcode,
                operands: operands
            )
        )
        return result
    }

    func output(
        _ id: String,
        _ value: CanonicalValueID,
        shape: CanonicalValueShape,
        unit: CanonicalUnit
    ) throws -> CanonicalGraphOutput {
        try CanonicalGraphOutput(id: id, value: value, shape: shape, unit: unit)
    }

    func build(outputs: [CanonicalGraphOutput]) throws -> CanonicalOperationGraph {
        try CanonicalOperationGraph(
            id: id,
            inputs: inputs,
            instructions: instructions,
            outputs: outputs
        )
    }

    mutating func forceGraph(
        bodyForce: CanonicalValueID,
        bodyTorque: CanonicalValueID,
        worldForce: CanonicalValueID
    ) throws -> CanonicalOperationGraph {
        try build(outputs: [
            output("body_force", bodyForce, shape: .vector3, unit: .newton),
            output("body_torque", bodyTorque, shape: .vector3, unit: .newtonMeter),
            output("world_force", worldForce, shape: .vector3, unit: .newton),
        ])
    }
}
