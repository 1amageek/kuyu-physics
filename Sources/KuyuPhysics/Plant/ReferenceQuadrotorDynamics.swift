import simd
import KuyuCore

public struct ReferenceQuadrotorDynamics: Sendable {
    public static func derivative(
        state: ReferenceQuadrotorState,
        input: ReferenceQuadrotorInput,
        parameters: ReferenceQuadrotorParameters,
        gravity: Double
    ) -> ReferenceQuadrotorStateDerivative {
        let force = QuadrotorGeneralizedForce(
            bodyForce: input.bodyForce,
            bodyTorque: input.bodyTorque,
            worldForce: input.worldForce
        )
            + ReferenceQuadrotorForceTerms.gravityForce(parameters: parameters, gravity: gravity)
            + ReferenceQuadrotorForceTerms.gyroscopicForce(state: state, parameters: parameters)

        return derivative(state: state, force: force, parameters: parameters)
    }

    public static func derivative(
        state: ReferenceQuadrotorState,
        force: QuadrotorGeneralizedForce,
        parameters: ReferenceQuadrotorParameters
    ) -> ReferenceQuadrotorStateDerivative {
        let inertia = parameters.inertiaSIMD
        let forceWorld = state.orientation.act(force.bodyForce) + force.worldForce
        let acceleration = forceWorld / parameters.mass
        let angularAccel = force.bodyTorque / inertia

        let omegaQuat = simd_quatd(real: 0, imag: state.angularVelocity)
        let qDot = (state.orientation * omegaQuat).vector * 0.5

        return ReferenceQuadrotorStateDerivative(
            position: state.velocity,
            velocity: acceleration,
            orientation: qDot,
            angularVelocity: angularAccel
        )
    }

    public static func integrateRK4(
        state: ReferenceQuadrotorState,
        input: ReferenceQuadrotorInput,
        parameters: ReferenceQuadrotorParameters,
        gravity: Double,
        delta: Double
    ) -> ReferenceQuadrotorState {
        let k1 = derivative(state: state, input: input, parameters: parameters, gravity: gravity)
        let s2 = state.applying(derivative: k1, scale: delta * 0.5)
        let k2 = derivative(state: s2, input: input, parameters: parameters, gravity: gravity)
        let s3 = state.applying(derivative: k2, scale: delta * 0.5)
        let k3 = derivative(state: s3, input: input, parameters: parameters, gravity: gravity)
        let s4 = state.applying(derivative: k3, scale: delta)
        let k4 = derivative(state: s4, input: input, parameters: parameters, gravity: gravity)

        let step = delta / 6.0
        let positionDelta = weightedSum(k1.position, k2.position, k3.position, k4.position) * step
        let velocityDelta = weightedSum(k1.velocity, k2.velocity, k3.velocity, k4.velocity) * step
        let orientationDelta = weightedSum(k1.orientation, k2.orientation, k3.orientation, k4.orientation) * step
        let angularVelocityDelta = weightedSum(k1.angularVelocity, k2.angularVelocity, k3.angularVelocity, k4.angularVelocity) * step

        let position = state.position + positionDelta
        let velocity = state.velocity + velocityDelta
        let orientationVector = state.orientation.vector + orientationDelta
        let angularVelocity = state.angularVelocity + angularVelocityDelta

        let orientation = simd_quatd(vector: orientationVector).normalizedQuat

        return ReferenceQuadrotorState(
            uncheckedPosition: position,
            uncheckedVelocity: velocity,
            uncheckedOrientation: orientation,
            uncheckedAngularVelocity: angularVelocity
        )
    }

    public static func specificForceBody(
        state: ReferenceQuadrotorState,
        input: ReferenceQuadrotorInput,
        parameters: ReferenceQuadrotorParameters,
        gravity: Double
    ) -> SIMD3<Double> {
        let gravityWorld = SIMD3<Double>(0, 0, -gravity)
        let forceWorld = state.orientation.act(input.bodyForce) + input.worldForce
        let acceleration = (forceWorld / parameters.mass) + gravityWorld
        let withoutGravity = acceleration - gravityWorld
        return state.orientation.inverse.act(withoutGravity)
    }

    private static func weightedSum(
        _ k1: SIMD3<Double>,
        _ k2: SIMD3<Double>,
        _ k3: SIMD3<Double>,
        _ k4: SIMD3<Double>
    ) -> SIMD3<Double> {
        k1 + (k2 * 2.0) + (k3 * 2.0) + k4
    }

    private static func weightedSum(
        _ k1: SIMD4<Double>,
        _ k2: SIMD4<Double>,
        _ k3: SIMD4<Double>,
        _ k4: SIMD4<Double>
    ) -> SIMD4<Double> {
        k1 + (k2 * 2.0) + (k3 * 2.0) + k4
    }
}
