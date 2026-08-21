import KuyuCore

extension ReferenceQuadrotorCanonicalProgram {
    static func forceTerms(
        layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> [CanonicalForceTermProgram] {
        try [
            CanonicalForceTermProgram(
                id: canonicalTermID(.gravity),
                graph: gravityGraph(layouts)
            ),
            CanonicalForceTermProgram(
                id: canonicalTermID(.propulsion),
                graph: propulsionGraph(layouts)
            ),
            CanonicalForceTermProgram(
                id: canonicalTermID(.thrustDensityScaling),
                graph: thrustDensityScalingGraph(layouts)
            ),
            CanonicalForceTermProgram(
                id: canonicalTermID(.disturbance),
                graph: disturbanceGraph(layouts)
            ),
            CanonicalForceTermProgram(
                id: canonicalTermID(.aerodynamicDrag),
                graph: aerodynamicDragGraph(layouts)
            ),
            CanonicalForceTermProgram(
                id: canonicalTermID(.aerodynamicLift),
                graph: aerodynamicLiftGraph(layouts)
            ),
            CanonicalForceTermProgram(
                id: canonicalTermID(.buoyancy),
                graph: buoyancyGraph(layouts)
            ),
            CanonicalForceTermProgram(
                id: canonicalTermID(.angularDrag),
                graph: angularDragGraph(layouts)
            ),
            CanonicalForceTermProgram(
                id: canonicalTermID(.gyroscopic),
                graph: gyroscopicGraph(layouts)
            ),
        ]
    }

    private static func gravityGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(id: "reference_quadrotor_gravity")
        let mass = try graph.input(layouts.parameters, "mass")
        let parameterGravity = try graph.input(layouts.parameters, "gravity")
        let environmentGravity = try graph.input(layouts.environment, "gravity")
        let useGravity = try graph.input(layouts.environment, "use_gravity")
        let gravityDelta = try graph.operation(
            .subtract,
            [environmentGravity, parameterGravity],
            "gravity.delta"
        )
        let gravityAdjustment = try graph.operation(
            .multiply,
            [useGravity, gravityDelta],
            "gravity.adjustment"
        )
        let gravity = try graph.operation(
            .add,
            [parameterGravity, gravityAdjustment],
            "gravity.effective"
        )
        let weight = try graph.operation(.multiply, [mass, gravity], "gravity.weight")
        let down = try graph.constant(
            [0, 0, -1],
            shape: .vector3,
            unit: .dimensionless,
            id: "gravity.down"
        )
        let worldForce = try graph.operation(
            .multiply,
            [weight, down],
            "gravity.world_force"
        )
        return try graph.forceGraph(
            bodyForce: graph.zeroVector(unit: .newton, id: "gravity.zero_body_force"),
            bodyTorque: graph.zeroVector(unit: .newtonMeter, id: "gravity.zero_body_torque"),
            worldForce: worldForce
        )
    }

    private static func propulsionGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(id: "reference_quadrotor_propulsion")
        let force = try propulsionValues(&graph, layouts: layouts, prefix: "propulsion")
        return try graph.forceGraph(
            bodyForce: force.bodyForce,
            bodyTorque: force.bodyTorque,
            worldForce: graph.zeroVector(unit: .newton, id: "propulsion.zero_world_force")
        )
    }

    private static func thrustDensityScalingGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(
            id: "reference_quadrotor_thrust_density_scaling"
        )
        let force = try propulsionValues(&graph, layouts: layouts, prefix: "thrust_density")
        let density = try airDensity(&graph, layouts: layouts, prefix: "thrust_density")
        let seaLevelDensity = try graph.constant(
            [WorldEnvironment.seaLevelDensity],
            shape: .scalar,
            unit: .kilogramPerCubicMeter,
            id: "thrust_density.sea_level_density"
        )
        let ratio = try graph.operation(
            .divide,
            [density, seaLevelDensity],
            "thrust_density.ratio"
        )
        let one = try graph.constant(
            [1],
            shape: .scalar,
            unit: .dimensionless,
            id: "thrust_density.one"
        )
        let delta = try graph.operation(.subtract, [ratio, one], "thrust_density.delta")
        let useAtmosphere = try graph.input(layouts.environment, "use_atmosphere")
        let scale = try graph.operation(
            .multiply,
            [useAtmosphere, delta],
            "thrust_density.scale"
        )
        return try graph.forceGraph(
            bodyForce: graph.operation(
                .multiply,
                [scale, force.bodyForce],
                "thrust_density.body_force"
            ),
            bodyTorque: graph.operation(
                .multiply,
                [scale, force.bodyTorque],
                "thrust_density.body_torque"
            ),
            worldForce: graph.zeroVector(unit: .newton, id: "thrust_density.zero_world_force")
        )
    }

    private static func disturbanceGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(id: "reference_quadrotor_disturbance")
        return try graph.forceGraph(
            bodyForce: graph.zeroVector(unit: .newton, id: "disturbance.zero_body_force"),
            bodyTorque: graph.input(layouts.disturbance, "body_torque"),
            worldForce: graph.input(layouts.disturbance, "world_force")
        )
    }

    private static func aerodynamicDragGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(
            id: "reference_quadrotor_aerodynamic_drag"
        )
        let airVelocity = try airVelocityWorld(&graph, layouts: layouts, prefix: "drag")
        let speed = try graph.operation(.length3, [airVelocity], "drag.speed")
        let direction = try graph.operation(.normalize3OrZero, [airVelocity], "drag.direction")
        let density = try airDensity(&graph, layouts: layouts, prefix: "drag")
        let coefficient = try graph.input(layouts.parameters, "drag_coefficient")
        let area = try graph.input(layouts.parameters, "reference_area")
        let half = try graph.constant(
            [0.5],
            shape: .scalar,
            unit: .dimensionless,
            id: "drag.half"
        )
        let speedSquared = try graph.operation(.multiply, [speed, speed], "drag.speed_squared")
        let densityCoefficient = try graph.operation(
            .multiply,
            [density, coefficient],
            "drag.density_coefficient"
        )
        let areaSpeedSquared = try graph.operation(
            .multiply,
            [area, speedSquared],
            "drag.area_speed_squared"
        )
        let forceScale = try graph.operation(
            .multiply,
            [densityCoefficient, areaSpeedSquared],
            "drag.force_scale"
        )
        let magnitude = try graph.operation(.multiply, [half, forceScale], "drag.magnitude")
        let useAtmosphere = try graph.input(layouts.environment, "use_atmosphere")
        let activeMagnitude = try graph.operation(
            .multiply,
            [useAtmosphere, magnitude],
            "drag.active_magnitude"
        )
        let force = try graph.operation(
            .multiply,
            [activeMagnitude, direction],
            "drag.positive_force"
        )
        let worldForce = try graph.operation(.negate, [force], "drag.world_force")
        return try graph.forceGraph(
            bodyForce: graph.zeroVector(unit: .newton, id: "drag.zero_body_force"),
            bodyTorque: graph.zeroVector(unit: .newtonMeter, id: "drag.zero_body_torque"),
            worldForce: worldForce
        )
    }

    private static func aerodynamicLiftGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(
            id: "reference_quadrotor_aerodynamic_lift"
        )
        let orientation = try graph.input(layouts.state, "orientation")
        let airVelocity = try airVelocityWorld(&graph, layouts: layouts, prefix: "lift")
        let airVelocityBody = try graph.operation(
            .quaternionInverseRotate3,
            [orientation, airVelocity],
            "lift.air_velocity_body"
        )
        let bodySpeed = try graph.operation(.length3, [airVelocityBody], "lift.body_speed")
        let velocityDirection = try graph.operation(
            .normalize3OrZero,
            [airVelocityBody],
            "lift.velocity_direction"
        )
        let bodyUp = try graph.constant(
            [0, 0, 1],
            shape: .vector3,
            unit: .dimensionless,
            id: "lift.body_up"
        )
        let innerPlane = try graph.operation(
            .cross3,
            [bodyUp, velocityDirection],
            "lift.inner_plane"
        )
        let liftPlane = try graph.operation(
            .cross3,
            [velocityDirection, innerPlane],
            "lift.plane"
        )
        let liftDirection = try graph.operation(
            .normalize3OrZero,
            [liftPlane],
            "lift.direction"
        )
        let density = try airDensity(&graph, layouts: layouts, prefix: "lift")
        let coefficient = try graph.input(layouts.parameters, "lift_coefficient")
        let area = try graph.input(layouts.parameters, "reference_area")
        let half = try graph.constant(
            [0.5],
            shape: .scalar,
            unit: .dimensionless,
            id: "lift.half"
        )
        let speedSquared = try graph.operation(
            .multiply,
            [bodySpeed, bodySpeed],
            "lift.speed_squared"
        )
        let densityCoefficient = try graph.operation(
            .multiply,
            [density, coefficient],
            "lift.density_coefficient"
        )
        let areaSpeedSquared = try graph.operation(
            .multiply,
            [area, speedSquared],
            "lift.area_speed_squared"
        )
        let forceScale = try graph.operation(
            .multiply,
            [densityCoefficient, areaSpeedSquared],
            "lift.force_scale"
        )
        let magnitude = try graph.operation(.multiply, [half, forceScale], "lift.magnitude")
        let useAtmosphere = try graph.input(layouts.environment, "use_atmosphere")
        let activeMagnitude = try graph.operation(
            .multiply,
            [useAtmosphere, magnitude],
            "lift.active_magnitude"
        )
        let bodyForce = try graph.operation(
            .multiply,
            [activeMagnitude, liftDirection],
            "lift.body_force"
        )
        let worldForce = try graph.operation(
            .quaternionRotate3,
            [orientation, bodyForce],
            "lift.world_force"
        )
        return try graph.forceGraph(
            bodyForce: graph.zeroVector(unit: .newton, id: "lift.zero_body_force"),
            bodyTorque: graph.zeroVector(unit: .newtonMeter, id: "lift.zero_body_torque"),
            worldForce: worldForce
        )
    }

    private static func buoyancyGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(id: "reference_quadrotor_buoyancy")
        let density = try airDensity(&graph, layouts: layouts, prefix: "buoyancy")
        let volume = try graph.input(layouts.parameters, "body_volume")
        let parameterGravity = try graph.input(layouts.parameters, "gravity")
        let environmentGravity = try graph.input(layouts.environment, "gravity")
        let useGravity = try graph.input(layouts.environment, "use_gravity")
        let gravityDelta = try graph.operation(
            .subtract,
            [environmentGravity, parameterGravity],
            "buoyancy.gravity_delta"
        )
        let gravityAdjustment = try graph.operation(
            .multiply,
            [useGravity, gravityDelta],
            "buoyancy.gravity_adjustment"
        )
        let gravity = try graph.operation(
            .add,
            [parameterGravity, gravityAdjustment],
            "buoyancy.gravity"
        )
        let densityVolume = try graph.operation(
            .multiply,
            [density, volume],
            "buoyancy.density_volume"
        )
        let magnitude = try graph.operation(
            .multiply,
            [gravity, densityVolume],
            "buoyancy.magnitude"
        )
        let useAtmosphere = try graph.input(layouts.environment, "use_atmosphere")
        let activeMagnitude = try graph.operation(
            .multiply,
            [useAtmosphere, magnitude],
            "buoyancy.active_magnitude"
        )
        let up = try graph.constant(
            [0, 0, 1],
            shape: .vector3,
            unit: .dimensionless,
            id: "buoyancy.up"
        )
        let worldForce = try graph.operation(
            .multiply,
            [activeMagnitude, up],
            "buoyancy.world_force"
        )
        return try graph.forceGraph(
            bodyForce: graph.zeroVector(unit: .newton, id: "buoyancy.zero_body_force"),
            bodyTorque: graph.zeroVector(unit: .newtonMeter, id: "buoyancy.zero_body_torque"),
            worldForce: worldForce
        )
    }

    private static func angularDragGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(
            id: "reference_quadrotor_angular_drag"
        )
        let angularDrag = try graph.input(layouts.parameters, "angular_drag")
        let omega = try graph.input(layouts.state, "angular_velocity")
        let damping = try graph.operation(
            .multiplyComponents,
            [angularDrag, omega],
            "angular_drag.damping"
        )
        let negativeDamping = try graph.operation(
            .negate,
            [damping],
            "angular_drag.negative_damping"
        )
        let useAtmosphere = try graph.input(layouts.environment, "use_atmosphere")
        let bodyTorque = try graph.operation(
            .multiply,
            [useAtmosphere, negativeDamping],
            "angular_drag.body_torque"
        )
        return try graph.forceGraph(
            bodyForce: graph.zeroVector(unit: .newton, id: "angular_drag.zero_body_force"),
            bodyTorque: bodyTorque,
            worldForce: graph.zeroVector(unit: .newton, id: "angular_drag.zero_world_force")
        )
    }

    private static func gyroscopicGraph(
        _ layouts: ReferenceQuadrotorCanonicalLayoutsSet
    ) throws -> CanonicalOperationGraph {
        var graph = ReferenceQuadrotorCanonicalGraphBuilder(
            id: "reference_quadrotor_gyroscopic"
        )
        let inertia = try graph.input(layouts.parameters, "inertia")
        let omega = try graph.input(layouts.state, "angular_velocity")
        let inertiaOmega = try graph.operation(
            .multiplyComponents,
            [inertia, omega],
            "gyroscopic.inertia_omega"
        )
        let gyroscopicTorque = try graph.operation(
            .cross3,
            [omega, inertiaOmega],
            "gyroscopic.positive_torque"
        )
        let bodyTorque = try graph.operation(
            .negate,
            [gyroscopicTorque],
            "gyroscopic.body_torque"
        )
        return try graph.forceGraph(
            bodyForce: graph.zeroVector(unit: .newton, id: "gyroscopic.zero_body_force"),
            bodyTorque: bodyTorque,
            worldForce: graph.zeroVector(unit: .newton, id: "gyroscopic.zero_world_force")
        )
    }

}
