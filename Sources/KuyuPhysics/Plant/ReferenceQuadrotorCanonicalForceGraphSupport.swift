import KuyuCore

extension ReferenceQuadrotorCanonicalProgram {
    static func propulsionValues(
        _ graph: inout ReferenceQuadrotorCanonicalGraphBuilder,
        layouts: ReferenceQuadrotorCanonicalLayoutsSet,
        prefix: String
    ) throws -> (bodyForce: CanonicalValueID, bodyTorque: CanonicalValueID) {
        let thrusts = try graph.input(layouts.control, "motor_thrusts")
        let armLength = try graph.input(layouts.mixer, "arm_length")
        let yawCoefficient = try graph.input(layouts.mixer, "yaw_coefficient")
        let spinDirections = try graph.input(layouts.mixer, "spin_directions")
        let f1 = try graph.component(thrusts, index: 0, id: "\(prefix).f1")
        let f2 = try graph.component(thrusts, index: 1, id: "\(prefix).f2")
        let f3 = try graph.component(thrusts, index: 2, id: "\(prefix).f3")
        let f4 = try graph.component(thrusts, index: 3, id: "\(prefix).f4")
        let f12 = try graph.operation(.add, [f1, f2], "\(prefix).f12")
        let f34 = try graph.operation(.add, [f3, f4], "\(prefix).f34")
        let total = try graph.operation(.add, [f12, f34], "\(prefix).total")
        let zeroForce = try graph.constant(
            [0],
            shape: .scalar,
            unit: .newton,
            id: "\(prefix).zero_force"
        )
        let bodyForce = try graph.operation(
            .composeVector3,
            [zeroForce, zeroForce, total],
            "\(prefix).mixed_body_force"
        )
        let f2MinusF4 = try graph.operation(
            .subtract,
            [f2, f4],
            "\(prefix).f2_minus_f4"
        )
        let f3MinusF1 = try graph.operation(
            .subtract,
            [f3, f1],
            "\(prefix).f3_minus_f1"
        )
        let tauX = try graph.operation(
            .multiply,
            [armLength, f2MinusF4],
            "\(prefix).tau_x"
        )
        let tauY = try graph.operation(
            .multiply,
            [armLength, f3MinusF1],
            "\(prefix).tau_y"
        )
        let spin0 = try graph.component(spinDirections, index: 0, id: "\(prefix).spin_0")
        let spin1 = try graph.component(spinDirections, index: 1, id: "\(prefix).spin_1")
        let spin2 = try graph.component(spinDirections, index: 2, id: "\(prefix).spin_2")
        let spin3 = try graph.component(spinDirections, index: 3, id: "\(prefix).spin_3")
        let yawTerm0 = try graph.operation(.multiply, [spin0, f1], "\(prefix).yaw_term_0")
        let yawTerm1 = try graph.operation(.multiply, [spin1, f2], "\(prefix).yaw_term_1")
        let yawTerm2 = try graph.operation(.multiply, [spin2, f3], "\(prefix).yaw_term_2")
        let yawTerm3 = try graph.operation(.multiply, [spin3, f4], "\(prefix).yaw_term_3")
        let yaw01 = try graph.operation(.add, [yawTerm0, yawTerm1], "\(prefix).yaw_01")
        let yaw23 = try graph.operation(.add, [yawTerm2, yawTerm3], "\(prefix).yaw_23")
        let yawForce = try graph.operation(.add, [yaw01, yaw23], "\(prefix).yaw_force")
        let tauZ = try graph.operation(
            .multiply,
            [yawCoefficient, yawForce],
            "\(prefix).tau_z"
        )
        let bodyTorque = try graph.operation(
            .composeVector3,
            [tauX, tauY, tauZ],
            "\(prefix).mixed_body_torque"
        )
        return (bodyForce, bodyTorque)
    }

    static func airDensity(
        _ graph: inout ReferenceQuadrotorCanonicalGraphBuilder,
        layouts: ReferenceQuadrotorCanonicalLayoutsSet,
        prefix: String
    ) throws -> CanonicalValueID {
        let pressure = try graph.input(layouts.environment, "air_pressure")
        let temperature = try graph.input(layouts.environment, "air_temperature")
        let gasConstant = try graph.constant(
            [WorldEnvironment.dryAirGasConstant],
            shape: .scalar,
            unit: .gasConstant,
            id: "\(prefix).gas_constant"
        )
        let denominator = try graph.operation(
            .multiply,
            [gasConstant, temperature],
            "\(prefix).density_denominator"
        )
        return try graph.operation(.divide, [pressure, denominator], "\(prefix).air_density")
    }

    static func airVelocityWorld(
        _ graph: inout ReferenceQuadrotorCanonicalGraphBuilder,
        layouts: ReferenceQuadrotorCanonicalLayoutsSet,
        prefix: String
    ) throws -> CanonicalValueID {
        let velocity = try graph.input(layouts.state, "velocity")
        let wind = try graph.input(layouts.environment, "wind_velocity_world")
        let useWind = try graph.input(layouts.environment, "use_wind")
        let activeWind = try graph.operation(
            .multiply,
            [useWind, wind],
            "\(prefix).active_wind"
        )
        return try graph.operation(
            .subtract,
            [velocity, activeWind],
            "\(prefix).air_velocity_world"
        )
    }
}
