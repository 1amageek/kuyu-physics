import simd

struct ContactSurface: Sendable, Equatable {
    let id: String
    let center: SIMD3<Double>
    let halfExtents: SIMD3<Double>
    let planeZ: Double
    let normal: SIMD3<Double>
    let material: ContactMaterial

    func contains(_ point: SIMD3<Double>) -> Bool {
        abs(point.x - center.x) <= halfExtents.x + 1e-9
            && abs(point.y - center.y) <= halfExtents.y + 1e-9
    }
}

struct ContactMaterial: Sendable, Equatable {
    let staticFriction: Double
    let dynamicFriction: Double
    let restitution: Double

    func combined(with other: ContactMaterial) -> ContactMaterial {
        ContactMaterial(
            staticFriction: min(staticFriction, other.staticFriction),
            dynamicFriction: min(dynamicFriction, other.dynamicFriction),
            restitution: min(restitution, other.restitution)
        )
    }
}

struct ArticulatedContact: Sendable, Equatable {
    let linkID: String
    let point: SIMD3<Double>
    let pointVelocity: SIMD3<Double>
    let normal: SIMD3<Double>
    let penetration: Double
    let normalVelocity: Double
    let material: ContactMaterial
    let linkMass: Double
}

struct ContactEvaluationBatch: Sendable, Equatable {
    let contacts: [ArticulatedContact]
    let evaluation: ArticulatedKinematicEvaluation
}

struct CollisionWitness: Sendable, Equatable {
    let point: SIMD3<Double>
    let velocity: SIMD3<Double>
}

struct ContactJacobianEntry: Sendable, Equatable {
    let index: Int
    let linear: SIMD3<Double>
}
