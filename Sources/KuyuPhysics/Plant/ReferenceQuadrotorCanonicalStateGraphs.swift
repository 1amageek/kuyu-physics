extension ReferenceQuadrotorCanonicalProgram {
    static func derivativeGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(
            id: "reference_quadrotor_derivative"
        )
        let velocity = try graph.input(layouts.state, "velocity")
        let orientation = try graph.input(layouts.state, "orientation")
        let omega = try graph.input(layouts.state, "angular_velocity")
        let mass = try graph.input(layouts.parameters, "mass")
        let inertia = try graph.input(layouts.parameters, "inertia")
        let bodyForce = try graph.input(layouts.generalizedForce, "body_force")
        let bodyTorque = try graph.input(layouts.generalizedForce, "body_torque")
        let worldForce = try graph.input(layouts.generalizedForce, "world_force")
        let rotatedBodyForce = try graph.operation(
            .quaternionRotate3,
            [orientation, bodyForce],
            "derivative.rotated_body_force"
        )
        let totalWorldForce = try graph.operation(
            .add,
            [rotatedBodyForce, worldForce],
            "derivative.total_world_force"
        )
        let linearAcceleration = try graph.operation(
            .divide,
            [totalWorldForce, mass],
            "derivative.linear_acceleration"
        )
        let angularAcceleration = try graph.operation(
            .divideComponents,
            [bodyTorque, inertia],
            "derivative.angular_acceleration"
        )
        let orientationRate = try graph.operation(
            .quaternionDerivative,
            [orientation, omega],
            "derivative.orientation_rate"
        )
        return try graph.build(outputs: [
            graph.output(
                "position_rate",
                velocity,
                shape: .vector3,
                unit: .meterPerSecond
            ),
            graph.output(
                "linear_acceleration",
                linearAcceleration,
                shape: .vector3,
                unit: .meterPerSecondSquared
            ),
            graph.output(
                "orientation_rate",
                orientationRate,
                shape: .vector4,
                unit: .inverseSecond
            ),
            graph.output(
                "angular_acceleration",
                angularAcceleration,
                shape: .vector3,
                unit: .radianPerSecondSquared
            ),
        ])
    }

    static func observableGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(
            id: "reference_quadrotor_observables"
        )
        let orientation = try graph.input(layouts.state, "orientation")
        let omega = try graph.input(layouts.state, "angular_velocity")
        let mass = try graph.input(layouts.parameters, "mass")
        let parameterGravity = try graph.input(layouts.parameters, "gravity")
        let environmentGravity = try graph.input(layouts.environment, "gravity")
        let useGravity = try graph.input(layouts.environment, "use_gravity")
        let bodyForce = try graph.input(layouts.generalizedForce, "body_force")
        let worldForce = try graph.input(layouts.generalizedForce, "world_force")
        let rotatedBodyForce = try graph.operation(
            .quaternionRotate3,
            [orientation, bodyForce],
            "observables.rotated_body_force"
        )
        let totalWorldForce = try graph.operation(
            .add,
            [rotatedBodyForce, worldForce],
            "observables.total_world_force"
        )
        let acceleration = try graph.operation(
            .divide,
            [totalWorldForce, mass],
            "observables.acceleration"
        )
        let gravityDelta = try graph.operation(
            .subtract,
            [environmentGravity, parameterGravity],
            "observables.gravity_delta"
        )
        let gravityAdjustment = try graph.operation(
            .multiply,
            [useGravity, gravityDelta],
            "observables.gravity_adjustment"
        )
        let gravity = try graph.operation(
            .add,
            [parameterGravity, gravityAdjustment],
            "observables.gravity"
        )
        let gravityDirection = try graph.constant(
            [0, 0, -1],
            shape: .vector3,
            unit: .dimensionless,
            id: "observables.gravity_direction"
        )
        let gravityWorld = try graph.operation(
            .multiply,
            [gravity, gravityDirection],
            "observables.gravity_world"
        )
        let specificForceWorld = try graph.operation(
            .subtract,
            [acceleration, gravityWorld],
            "observables.specific_force_world"
        )
        let specificForceBody = try graph.operation(
            .quaternionInverseRotate3,
            [orientation, specificForceWorld],
            "observables.specific_force_body"
        )
        return try graph.build(outputs: [
            graph.output(
                "angular_velocity_body",
                omega,
                shape: .vector3,
                unit: .radianPerSecond
            ),
            graph.output(
                "specific_force_body",
                specificForceBody,
                shape: .vector3,
                unit: .meterPerSecondSquared
            ),
        ])
    }
}
