import KuyuCore
import simd

public struct ReferenceQuadrotorForceTerms: Sendable {
    public static let canonical: [AnyQuadrotorForceTerm] = [
        gravity,
        propulsion,
        thrustDensityScaling,
        disturbance,
        aerodynamicDrag,
        aerodynamicLift,
        buoyancy,
        angularDrag,
        gyroscopic,
    ]

    public static let gravity = AnyQuadrotorForceTerm(id: .gravity) { context in
        gravityForce(parameters: context.parameters, gravity: context.effectiveGravity)
    }

    public static let propulsion = AnyQuadrotorForceTerm(id: .propulsion) { context in
        let mix = context.mixer.mix(thrusts: context.motorThrusts)
        return QuadrotorGeneralizedForce(bodyForce: mix.forceBody, bodyTorque: mix.torqueBody)
    }

    public static let thrustDensityScaling = AnyQuadrotorForceTerm(id: .thrustDensityScaling) { context in
        guard context.environment.usage.useAtmosphere else {
            return .zero
        }
        let mix = context.mixer.mix(thrusts: context.motorThrusts)
        let ratio = context.environment.airDensity() / WorldEnvironment.seaLevelDensity
        let delta = ratio - 1.0
        return QuadrotorGeneralizedForce(
            bodyForce: mix.forceBody * delta,
            bodyTorque: mix.torqueBody * delta
        )
    }

    public static let disturbance = AnyQuadrotorForceTerm(id: .disturbance) { context in
        QuadrotorGeneralizedForce(
            bodyTorque: context.disturbances.torqueBody,
            worldForce: context.disturbances.forceWorld
        )
    }

    public static let aerodynamicDrag = AnyQuadrotorForceTerm(id: .aerodynamicDrag) { context in
        guard context.environment.usage.useAtmosphere else {
            return .zero
        }
        let aero = context.parameters.aerodynamics
        guard aero.dragCoefficient > 0, aero.referenceArea > 0 else {
            return .zero
        }
        let airVelocity = airVelocityWorld(context: context)
        let speed = simd_length(airVelocity)
        guard speed > 0 else {
            return .zero
        }
        let density = context.environment.airDensity()
        let dragMagnitude = 0.5 * density * aero.dragCoefficient * aero.referenceArea * speed * speed
        let drag = -dragMagnitude * (airVelocity / speed)
        return QuadrotorGeneralizedForce(worldForce: drag)
    }

    public static let aerodynamicLift = AnyQuadrotorForceTerm(id: .aerodynamicLift) { context in
        guard context.environment.usage.useAtmosphere else {
            return .zero
        }
        let aero = context.parameters.aerodynamics
        guard aero.liftCoefficient > 0, aero.referenceArea > 0 else {
            return .zero
        }
        let airVelocityBody = context.state.orientation.inverse.act(airVelocityWorld(context: context))
        let bodySpeed = simd_length(airVelocityBody)
        guard bodySpeed > 0 else {
            return .zero
        }
        let velocityDirection = airVelocityBody / bodySpeed
        let bodyUp = SIMD3<Double>(0, 0, 1)
        let liftPlane = simd_cross(velocityDirection, simd_cross(bodyUp, velocityDirection))
        let liftPlaneMagnitude = simd_length(liftPlane)
        guard liftPlaneMagnitude > 0 else {
            return .zero
        }
        let density = context.environment.airDensity()
        let liftMagnitude = 0.5 * density * aero.liftCoefficient * aero.referenceArea * bodySpeed * bodySpeed
        let liftBody = liftPlane / liftPlaneMagnitude * liftMagnitude
        return QuadrotorGeneralizedForce(worldForce: context.state.orientation.act(liftBody))
    }

    public static let buoyancy = AnyQuadrotorForceTerm(id: .buoyancy) { context in
        guard context.environment.usage.useAtmosphere else {
            return .zero
        }
        let volume = context.parameters.aerodynamics.bodyVolume
        guard volume > 0 else {
            return .zero
        }
        let buoyancy = SIMD3<Double>(0, 0, context.effectiveGravity * context.environment.airDensity() * volume)
        return QuadrotorGeneralizedForce(worldForce: buoyancy)
    }

    public static let angularDrag = AnyQuadrotorForceTerm(id: .angularDrag) { context in
        guard context.environment.usage.useAtmosphere else {
            return .zero
        }
        let angularDrag = context.parameters.aerodynamics.angularDrag
        guard angularDrag.x > 0 || angularDrag.y > 0 || angularDrag.z > 0 else {
            return .zero
        }
        let omega = context.state.angularVelocity
        let damping = SIMD3<Double>(
            angularDrag.x * omega.x,
            angularDrag.y * omega.y,
            angularDrag.z * omega.z
        )
        return QuadrotorGeneralizedForce(bodyTorque: -damping)
    }

    public static let gyroscopic = AnyQuadrotorForceTerm(id: .gyroscopic) { context in
        gyroscopicForce(state: context.state, parameters: context.parameters)
    }

    public static func gravityForce(
        parameters: ReferenceQuadrotorParameters,
        gravity: Double
    ) -> QuadrotorGeneralizedForce {
        QuadrotorGeneralizedForce(worldForce: SIMD3<Double>(0, 0, -parameters.mass * gravity))
    }

    public static func gyroscopicForce(
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters
    ) -> QuadrotorGeneralizedForce {
        let inertiaOmega = parameters.inertiaSIMD * state.angularVelocity
        let gyroscopicTorque = simd_cross(state.angularVelocity, inertiaOmega)
        return QuadrotorGeneralizedForce(bodyTorque: -gyroscopicTorque)
    }

    private static func airVelocityWorld(context: QuadrotorForceTermContext) -> SIMD3<Double> {
        let windVelocity = context.environment.usage.useWind
            ? SIMD3<Double>(
                context.environment.windVelocityWorld.x,
                context.environment.windVelocityWorld.y,
                context.environment.windVelocityWorld.z
            )
            : SIMD3<Double>(repeating: 0)
        return context.state.velocity - windVelocity
    }
}
