import simd

public struct QuadrotorGeneralizedForce: Sendable, Equatable {
    public var bodyForce: SIMD3<Double>
    public var bodyTorque: SIMD3<Double>
    public var worldForce: SIMD3<Double>

    public init(
        bodyForce: SIMD3<Double> = SIMD3<Double>(repeating: 0),
        bodyTorque: SIMD3<Double> = SIMD3<Double>(repeating: 0),
        worldForce: SIMD3<Double> = SIMD3<Double>(repeating: 0)
    ) {
        self.bodyForce = bodyForce
        self.bodyTorque = bodyTorque
        self.worldForce = worldForce
    }

    public static let zero = QuadrotorGeneralizedForce()
}

public func + (lhs: QuadrotorGeneralizedForce, rhs: QuadrotorGeneralizedForce) -> QuadrotorGeneralizedForce {
    QuadrotorGeneralizedForce(
        bodyForce: lhs.bodyForce + rhs.bodyForce,
        bodyTorque: lhs.bodyTorque + rhs.bodyTorque,
        worldForce: lhs.worldForce + rhs.worldForce
    )
}

public func - (lhs: QuadrotorGeneralizedForce, rhs: QuadrotorGeneralizedForce) -> QuadrotorGeneralizedForce {
    QuadrotorGeneralizedForce(
        bodyForce: lhs.bodyForce - rhs.bodyForce,
        bodyTorque: lhs.bodyTorque - rhs.bodyTorque,
        worldForce: lhs.worldForce - rhs.worldForce
    )
}

public func += (lhs: inout QuadrotorGeneralizedForce, rhs: QuadrotorGeneralizedForce) {
    lhs = lhs + rhs
}
