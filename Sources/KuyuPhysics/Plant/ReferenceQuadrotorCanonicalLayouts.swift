public enum ReferenceQuadrotorCanonicalLayouts {
    public static func stateLayout() throws -> CanonicalBufferLayout {
        try CanonicalBufferLayout(
            id: "reference_quadrotor_state",
            version: 1,
            fields: [
                field("position", offset: 0, shape: .vector3, unit: "m", role: .state),
                field("velocity", offset: 3, shape: .vector3, unit: "m/s", role: .state),
                field("orientation", offset: 6, shape: .quaternion, unit: "1", role: .state),
                field("angular_velocity", offset: 10, shape: .vector3, unit: "rad/s", role: .state),
            ]
        )
    }

    public static func derivativeLayout() throws -> CanonicalBufferLayout {
        try CanonicalBufferLayout(
            id: "reference_quadrotor_state_derivative",
            version: 1,
            fields: [
                field("position_rate", offset: 0, shape: .vector3, unit: "m/s", role: .observable),
                field("linear_acceleration", offset: 3, shape: .vector3, unit: "m/s^2", role: .observable),
                field("orientation_rate", offset: 6, shape: .vector4, unit: "1/s", role: .observable),
                field("angular_acceleration", offset: 10, shape: .vector3, unit: "rad/s^2", role: .observable),
            ]
        )
    }

    public static func parameterLayout() throws -> CanonicalBufferLayout {
        try CanonicalBufferLayout(
            id: "reference_quadrotor_parameters",
            version: 1,
            fields: [
                field("mass", offset: 0, shape: .scalar, unit: "kg", role: .parameter),
                field("inertia", offset: 1, shape: .vector3, unit: "kg*m^2", role: .parameter),
                field("arm_length", offset: 4, shape: .scalar, unit: "m", role: .parameter),
                field("motor_time_constant", offset: 5, shape: .scalar, unit: "s", role: .parameter),
                field("max_thrust", offset: 6, shape: .scalar, unit: "N", role: .parameter),
                field("yaw_coefficient", offset: 7, shape: .scalar, unit: "m", role: .parameter),
                field("gravity", offset: 8, shape: .scalar, unit: "m/s^2", role: .parameter),
                field("drag_coefficient", offset: 9, shape: .scalar, unit: "1", role: .parameter),
                field("reference_area", offset: 10, shape: .scalar, unit: "m^2", role: .parameter),
                field("lift_coefficient", offset: 11, shape: .scalar, unit: "1", role: .parameter),
                field("body_volume", offset: 12, shape: .scalar, unit: "m^3", role: .parameter),
                field("angular_drag", offset: 13, shape: .vector3, unit: "N*m*s/rad", role: .parameter),
            ]
        )
    }

    public static func controlLayout() throws -> CanonicalBufferLayout {
        try CanonicalBufferLayout(
            id: "reference_quadrotor_motor_thrusts",
            version: 1,
            fields: [
                field("motor_thrusts", offset: 0, shape: .vector4, unit: "N", role: .control),
            ]
        )
    }

    public static func mixerLayout() throws -> CanonicalBufferLayout {
        try CanonicalBufferLayout(
            id: "reference_quadrotor_mixer",
            version: 1,
            fields: [
                field("arm_length", offset: 0, shape: .scalar, unit: "m", role: .parameter),
                field("yaw_coefficient", offset: 1, shape: .scalar, unit: "m", role: .parameter),
                field("spin_directions", offset: 2, shape: .vector4, unit: "1", role: .parameter),
                field(
                    "layout_code",
                    offset: 6,
                    shape: .scalar,
                    unit: "1",
                    role: .parameter,
                    differentiability: .nonDifferentiable
                ),
            ]
        )
    }

    public static func disturbanceLayout() throws -> CanonicalBufferLayout {
        try CanonicalBufferLayout(
            id: "reference_quadrotor_disturbance",
            version: 1,
            fields: [
                field("body_torque", offset: 0, shape: .vector3, unit: "N*m", role: .disturbance),
                field("world_force", offset: 3, shape: .vector3, unit: "N", role: .disturbance),
            ]
        )
    }

    public static func environmentLayout() throws -> CanonicalBufferLayout {
        try CanonicalBufferLayout(
            id: "reference_quadrotor_environment",
            version: 1,
            fields: [
                field("gravity", offset: 0, shape: .scalar, unit: "m/s^2", role: .parameter),
                field("wind_velocity_world", offset: 1, shape: .vector3, unit: "m/s", role: .parameter),
                field("air_pressure", offset: 4, shape: .scalar, unit: "Pa", role: .parameter),
                field("air_temperature", offset: 5, shape: .scalar, unit: "K", role: .parameter),
                field(
                    "use_gravity",
                    offset: 6,
                    shape: .scalar,
                    unit: "1",
                    role: .parameter,
                    differentiability: .nonDifferentiable
                ),
                field(
                    "use_wind",
                    offset: 7,
                    shape: .scalar,
                    unit: "1",
                    role: .parameter,
                    differentiability: .nonDifferentiable
                ),
                field(
                    "use_atmosphere",
                    offset: 8,
                    shape: .scalar,
                    unit: "1",
                    role: .parameter,
                    differentiability: .nonDifferentiable
                ),
            ]
        )
    }

    public static func generalizedForceLayout() throws -> CanonicalBufferLayout {
        try CanonicalBufferLayout(
            id: "reference_quadrotor_generalized_force",
            version: 1,
            fields: [
                field("body_force", offset: 0, shape: .vector3, unit: "N", role: .temporary),
                field("body_torque", offset: 3, shape: .vector3, unit: "N*m", role: .temporary),
                field("world_force", offset: 6, shape: .vector3, unit: "N", role: .temporary),
            ]
        )
    }

    public static func observableLayout() throws -> CanonicalBufferLayout {
        try CanonicalBufferLayout(
            id: "reference_quadrotor_observables",
            version: 1,
            fields: [
                field(
                    "angular_velocity_body",
                    offset: 0,
                    shape: .vector3,
                    unit: "rad/s",
                    role: .observable
                ),
                field(
                    "specific_force_body",
                    offset: 3,
                    shape: .vector3,
                    unit: "m/s^2",
                    role: .observable
                ),
            ]
        )
    }

    private static func field(
        _ id: String,
        offset: Int,
        shape: CanonicalValueShape,
        unit: String,
        role: CanonicalValueRole,
        differentiability: CanonicalDifferentiability = .differentiable
    ) throws -> CanonicalBufferField {
        try CanonicalBufferField(
            id: id,
            offset: offset,
            shape: shape,
            unit: unit,
            role: role,
            differentiability: differentiability
        )
    }
}
