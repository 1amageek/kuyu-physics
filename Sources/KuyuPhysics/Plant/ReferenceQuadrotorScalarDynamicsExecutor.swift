import KuyuCore
import simd

public struct ReferenceQuadrotorScalarDynamicsExecutor: ReferenceQuadrotorCanonicalExecuting, Sendable {
    public let executorVersion = "swift-float64-ssa-v1"

    private let interpreter = CanonicalFloat64GraphInterpreter()

    public init() {}

    public func generalizedForce(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState,
        environment: WorldEnvironment,
        activeTerms: Set<QuadrotorForceTermID>
    ) throws -> QuadrotorGeneralizedForce {
        let inputs = try runtimeInputs(
            program: program,
            state: state,
            parameters: parameters,
            mixer: mixer,
            motorThrusts: motorThrusts,
            disturbances: disturbances,
            environment: environment,
            force: nil
        )
        var total = QuadrotorGeneralizedForce.zero
        for term in program.content.forceTerms {
            guard let termID = quadrotorTermID(term.id) else {
                throw CanonicalFloat64ExecutionError.unknownForceTerm(term.id)
            }
            guard activeTerms.contains(termID) else {
                continue
            }
            let outputs = try interpreter.execute(term.graph, inputs: inputs)
            total += QuadrotorGeneralizedForce(
                bodyForce: try vector3Output("body_force", graph: term.graph, outputs: outputs),
                bodyTorque: try vector3Output("body_torque", graph: term.graph, outputs: outputs),
                worldForce: try vector3Output("world_force", graph: term.graph, outputs: outputs)
            )
        }
        return total
    }

    private func quadrotorTermID(
        _ id: CanonicalForceTermID
    ) -> QuadrotorForceTermID? {
        switch id.rawValue {
        case "gravity":
            .gravity
        case "propulsion":
            .propulsion
        case "thrust_density_scaling":
            .thrustDensityScaling
        case "disturbance":
            .disturbance
        case "aerodynamic_drag":
            .aerodynamicDrag
        case "aerodynamic_lift":
            .aerodynamicLift
        case "buoyancy":
            .buoyancy
        case "angular_drag":
            .angularDrag
        case "gyroscopic":
            .gyroscopic
        default:
            nil
        }
    }

    public func derivative(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        force: QuadrotorGeneralizedForce
    ) throws -> ReferenceQuadrotorStateDerivative {
        let inputs = try runtimeInputs(
            program: program,
            state: state,
            parameters: parameters,
            mixer: nil,
            motorThrusts: nil,
            disturbances: nil,
            environment: nil,
            force: force
        )
        let graph = program.content.derivativeGraph
        let outputs = try interpreter.execute(graph, inputs: inputs)
        return ReferenceQuadrotorStateDerivative(
            position: try vector3Output("position_rate", graph: graph, outputs: outputs),
            velocity: try vector3Output("linear_acceleration", graph: graph, outputs: outputs),
            orientation: try vector4Output("orientation_rate", graph: graph, outputs: outputs),
            angularVelocity: try vector3Output("angular_acceleration", graph: graph, outputs: outputs)
        )
    }

    public func observables(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        environment: WorldEnvironment,
        force: QuadrotorGeneralizedForce
    ) throws -> ReferenceQuadrotorCanonicalObservables {
        let inputs = try runtimeInputs(
            program: program,
            state: state,
            parameters: parameters,
            mixer: nil,
            motorThrusts: nil,
            disturbances: nil,
            environment: environment,
            force: force
        )
        let graph = program.content.observableGraph
        let outputs = try interpreter.execute(graph, inputs: inputs)
        return ReferenceQuadrotorCanonicalObservables(
            angularVelocityBody: try vector3Output(
                "angular_velocity_body",
                graph: graph,
                outputs: outputs
            ),
            specificForceBody: try vector3Output(
                "specific_force_body",
                graph: graph,
                outputs: outputs
            )
        )
    }

    private func runtimeInputs(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer?,
        motorThrusts: MotorThrusts?,
        disturbances: DisturbanceState?,
        environment: WorldEnvironment?,
        force: QuadrotorGeneralizedForce?
    ) throws -> [CanonicalValueID: CanonicalFloat64Value] {
        let bindings = program.content.layoutBindings
        var values: [CanonicalValueID: CanonicalFloat64Value] = [:]

        try bind(&values, layoutID: bindings.state, fieldID: "position", value: .vector3(state.position))
        try bind(&values, layoutID: bindings.state, fieldID: "velocity", value: .vector3(state.velocity))
        try bind(&values, layoutID: bindings.state, fieldID: "orientation", value: .quaternion(state.orientation))
        try bind(
            &values,
            layoutID: bindings.state,
            fieldID: "angular_velocity",
            value: .vector3(state.angularVelocity)
        )

        let aerodynamics = parameters.aerodynamics
        try bind(&values, layoutID: bindings.parameters, fieldID: "mass", value: .scalar(parameters.mass))
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "inertia",
            value: .vector3(parameters.inertiaSIMD)
        )
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "arm_length",
            value: .scalar(parameters.armLength)
        )
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "motor_time_constant",
            value: .scalar(parameters.motorTimeConstant)
        )
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "max_thrust",
            value: .scalar(parameters.maxThrust)
        )
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "yaw_coefficient",
            value: .scalar(parameters.yawCoefficient)
        )
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "gravity",
            value: .scalar(parameters.gravity)
        )
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "drag_coefficient",
            value: .scalar(aerodynamics.dragCoefficient)
        )
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "reference_area",
            value: .scalar(aerodynamics.referenceArea)
        )
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "lift_coefficient",
            value: .scalar(aerodynamics.liftCoefficient)
        )
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "body_volume",
            value: .scalar(aerodynamics.bodyVolume)
        )
        try bind(
            &values,
            layoutID: bindings.parameters,
            fieldID: "angular_drag",
            value: .vector3(
                SIMD3<Double>(
                    aerodynamics.angularDrag.x,
                    aerodynamics.angularDrag.y,
                    aerodynamics.angularDrag.z
                )
            )
        )

        if let mixer {
            try bind(&values, layoutID: bindings.mixer, fieldID: "arm_length", value: .scalar(mixer.armLength))
            try bind(
                &values,
                layoutID: bindings.mixer,
                fieldID: "yaw_coefficient",
                value: .scalar(mixer.yawCoefficient)
            )
            try bind(
                &values,
                layoutID: bindings.mixer,
                fieldID: "spin_directions",
                value: .vector4(mixer.spinDirections)
            )
            try bind(&values, layoutID: bindings.mixer, fieldID: "layout_code", value: .scalar(0))
        }

        if let motorThrusts {
            try bind(
                &values,
                layoutID: bindings.control,
                fieldID: "motor_thrusts",
                value: .vector4(
                    SIMD4<Double>(
                        motorThrusts.f1,
                        motorThrusts.f2,
                        motorThrusts.f3,
                        motorThrusts.f4
                    )
                )
            )
        }

        if let disturbances {
            try bind(
                &values,
                layoutID: bindings.disturbance,
                fieldID: "body_torque",
                value: .vector3(disturbances.torqueBody)
            )
            try bind(
                &values,
                layoutID: bindings.disturbance,
                fieldID: "world_force",
                value: .vector3(disturbances.forceWorld)
            )
        }

        if let environment {
            try bind(
                &values,
                layoutID: bindings.environment,
                fieldID: "gravity",
                value: .scalar(environment.gravity)
            )
            try bind(
                &values,
                layoutID: bindings.environment,
                fieldID: "wind_velocity_world",
                value: .vector3(
                    SIMD3<Double>(
                        environment.windVelocityWorld.x,
                        environment.windVelocityWorld.y,
                        environment.windVelocityWorld.z
                    )
                )
            )
            try bind(
                &values,
                layoutID: bindings.environment,
                fieldID: "air_pressure",
                value: .scalar(environment.airPressure)
            )
            try bind(
                &values,
                layoutID: bindings.environment,
                fieldID: "air_temperature",
                value: .scalar(environment.airTemperature)
            )
            try bind(
                &values,
                layoutID: bindings.environment,
                fieldID: "use_gravity",
                value: .scalar(environment.usage.useGravity ? 1 : 0)
            )
            try bind(
                &values,
                layoutID: bindings.environment,
                fieldID: "use_wind",
                value: .scalar(environment.usage.useWind ? 1 : 0)
            )
            try bind(
                &values,
                layoutID: bindings.environment,
                fieldID: "use_atmosphere",
                value: .scalar(environment.usage.useAtmosphere ? 1 : 0)
            )
        }

        if let force {
            try bind(
                &values,
                layoutID: bindings.generalizedForce,
                fieldID: "body_force",
                value: .vector3(force.bodyForce)
            )
            try bind(
                &values,
                layoutID: bindings.generalizedForce,
                fieldID: "body_torque",
                value: .vector3(force.bodyTorque)
            )
            try bind(
                &values,
                layoutID: bindings.generalizedForce,
                fieldID: "world_force",
                value: .vector3(force.worldForce)
            )
        }
        return values
    }

    private func bind(
        _ values: inout [CanonicalValueID: CanonicalFloat64Value],
        layoutID: String,
        fieldID: String,
        value: CanonicalFloat64Value
    ) throws {
        values[try CanonicalValueID("\(layoutID).\(fieldID)")] = value
    }

    private func vector3Output(
        _ outputID: String,
        graph: CanonicalOperationGraph,
        outputs: [String: CanonicalFloat64Value]
    ) throws -> SIMD3<Double> {
        guard let output = graph.outputs.first(where: { $0.id == outputID }) else {
            throw CanonicalFloat64ExecutionError.missingOutput(graphID: graph.id, outputID: outputID)
        }
        guard case let .vector3(value) = outputs[outputID] else {
            throw CanonicalFloat64ExecutionError.typeMismatch(output.value)
        }
        return value
    }

    private func vector4Output(
        _ outputID: String,
        graph: CanonicalOperationGraph,
        outputs: [String: CanonicalFloat64Value]
    ) throws -> SIMD4<Double> {
        guard let output = graph.outputs.first(where: { $0.id == outputID }) else {
            throw CanonicalFloat64ExecutionError.missingOutput(graphID: graph.id, outputID: outputID)
        }
        guard case let .vector4(value) = outputs[outputID] else {
            throw CanonicalFloat64ExecutionError.typeMismatch(output.value)
        }
        return value
    }
}
