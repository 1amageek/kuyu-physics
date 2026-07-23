import simd

extension ArticulatedContactSolver {
    func project(
        state: ArticulatedState,
        deltaTime: Double
    ) throws -> ArticulatedStateStepResult {
        guard world.contact.mode == .constraint else {
            return ArticulatedStateStepResult(state: state, contactMetrics: .disabled)
        }
        var next = state
        let tolerance = max(world.solver.tolerance, 1e-9)
        let iterations = max(world.solver.iterations, 1)
        var metrics = ArticulatedContactMetrics.disabled
        var iterationsUsed = 0
        var observedContacts = 0

        for iteration in 0..<iterations {
            iterationsUsed = iteration + 1
            let batch = try allContacts(state: next)
            let contacts = batch.contacts.filter { $0.penetration > 0 }
            observedContacts = max(observedContacts, contacts.count)
            let maxPenetration = contacts.map(\.penetration).max() ?? 0
            if maxPenetration <= tolerance {
                metrics.maxPenetration = maxPenetration
                break
            }
            var deltas = Array(repeating: 0.0, count: next.position.count)
            for contact in contacts {
                let entries = jacobianEntries(for: contact, evaluation: batch.evaluation)
                let denominator = inverseInertiaWeightedNormalDenominator(entries: entries, normal: contact.normal)
                guard denominator > 1e-18 else { continue }
                let lambda = contact.penetration / denominator
                for entry in entries {
                    let normalJacobian = simd_dot(entry.linear, contact.normal)
                    let rawDelta = inverseInertiaByBindingIndex[entry.index] * normalJacobian * lambda
                    deltas[entry.index] += clamp(rawDelta, lower: -0.05, upper: 0.05)
                }
            }
            for index in next.position.indices {
                next.position[index] = try clampedPosition(
                    next.position[index] + deltas[index],
                    joint: bindings[index].joint,
                    field: "contact.position[\(index)]"
                )
            }
        }

        let finalBatch = try allContacts(state: next)
        let finalContacts = finalBatch.contacts.filter { $0.penetration > 0 }
        let residualPenetration = finalContacts.map(\.penetration).max() ?? 0
        if residualPenetration > tolerance {
            throw ArticulatedRigidBodySimulator.SimulationError.unresolvedContact(
                maxPenetration: residualPenetration,
                tolerance: tolerance
            )
        }

        var velocityImpulseMax = 0.0
        let velocityContacts = finalBatch.contacts.filter { $0.penetration >= -tolerance }
        observedContacts = max(observedContacts, velocityContacts.count)
        for contact in velocityContacts where contact.normalVelocity < 0 {
            let entries = jacobianEntries(for: contact, evaluation: finalBatch.evaluation)
            let denominator = inverseInertiaWeightedNormalDenominator(entries: entries, normal: contact.normal)
            guard denominator > 1e-18 else { continue }
            let impulse = -(1 + contact.material.restitution) * contact.normalVelocity / denominator
            for entry in entries {
                let normalJacobian = simd_dot(entry.linear, contact.normal)
                next.velocity[entry.index] = try clampedVelocity(
                    next.velocity[entry.index] + inverseInertiaByBindingIndex[entry.index] * normalJacobian * impulse,
                    joint: bindings[entry.index].joint,
                    field: "contact.velocity[\(entry.index)]"
                )
            }
            velocityImpulseMax = max(velocityImpulseMax, abs(impulse))
        }

        metrics.activeContacts = observedContacts
        metrics.maxPenetration = residualPenetration
        metrics.maxNormalImpulse = velocityImpulseMax
        metrics.solverIterations = iterationsUsed
        try ensureFinite(next.position, field: "contact.position")
        try ensureFinite(next.velocity, field: "contact.velocity")
        return ArticulatedStateStepResult(state: next, contactMetrics: metrics)
    }
}
