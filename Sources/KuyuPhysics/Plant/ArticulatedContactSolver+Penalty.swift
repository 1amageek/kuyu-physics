import simd

extension ArticulatedContactSolver {
    func contactTorques(
        state: ArticulatedState,
        deltaTime: Double
    ) throws -> ([Double], ArticulatedContactMetrics) {
        var torques = Array(repeating: 0.0, count: bindings.count)
        guard world.contact.mode == .penalty else {
            return (torques, .disabled)
        }
        let batch = try allContacts(state: state)
        let contacts = batch.contacts.filter { $0.penetration > 0 }
        guard !contacts.isEmpty else {
            return (torques, .disabled)
        }
        let stiffness = world.contact.stiffness ?? 0
        let damping = world.contact.damping ?? 0
        var metrics = ArticulatedContactMetrics(
            activeContacts: contacts.count,
            maxPenetration: contacts.map(\.penetration).max() ?? 0,
            maxNormalImpulse: 0,
            maxNormalForce: 0,
            solverIterations: 1
        )

        for contact in contacts {
            let closingVelocity = min(contact.normalVelocity, 0)
            var normalForce = stiffness * contact.penetration - damping * closingVelocity
            if contact.normalVelocity < 0 {
                normalForce += contact.material.restitution * (-contact.normalVelocity) * contact.linkMass / deltaTime
            }
            guard normalForce > 0 else { continue }
            let tangentialVelocity = contact.pointVelocity - contact.normal * simd_dot(contact.pointVelocity, contact.normal)
            let tangentialSpeed = simd_length(tangentialVelocity)
            var force = contact.normal * normalForce
            if tangentialSpeed > 1e-12 {
                let frictionLimit = contact.material.dynamicFriction * normalForce
                let stopForce = contact.linkMass * tangentialSpeed / deltaTime
                force -= (tangentialVelocity / tangentialSpeed) * min(frictionLimit, stopForce)
            }
            let entries = jacobianEntries(for: contact, evaluation: batch.evaluation)
            for entry in entries {
                torques[entry.index] += simd_dot(entry.linear, force)
            }
            metrics.maxNormalForce = max(metrics.maxNormalForce, normalForce)
            metrics.maxNormalImpulse = max(metrics.maxNormalImpulse, normalForce * deltaTime)
        }
        try ensureFinite(torques, field: "contact.torques")
        return (torques, metrics)
    }
}
