extension ArticulatedContactSolver {
    func clampedPosition(_ value: Double, joint: JointDefinition, field: String) throws -> Double {
        guard value.isFinite else {
            throw ArticulatedRigidBodySimulator.SimulationError.nonFiniteState(field)
        }
        var result = value
        if let lower = joint.lowerLimit {
            result = max(result, lower)
        }
        if let upper = joint.upperLimit {
            result = min(result, upper)
        }
        return result
    }

    func clampedVelocity(_ value: Double, joint: JointDefinition, field: String) throws -> Double {
        guard value.isFinite else {
            throw ArticulatedRigidBodySimulator.SimulationError.nonFiniteState(field)
        }
        guard let limit = joint.velocityLimit else { return value }
        return clamp(value, lower: -limit, upper: limit)
    }

    func ensureFinite(_ values: [Double], field: String) throws {
        for (index, value) in values.enumerated() where !value.isFinite {
            throw ArticulatedRigidBodySimulator.SimulationError.nonFiniteState("\(field)[\(index)]")
        }
    }

    func clamp(_ value: Double, lower: Double, upper: Double) -> Double {
        min(max(value, lower), upper)
    }
}
