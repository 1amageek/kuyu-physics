import Foundation
import KuyuPhysics
import KuyuCore
import simd
import Testing

@Test(.timeLimit(.minutes(1))) func referenceQuadrotorCanonicalProgramIsStableAndRoundTrips() throws {
    let first = try ReferenceQuadrotorCanonicalProgram.make()
    let second = try ReferenceQuadrotorCanonicalProgram.make()

    #expect(first == second)
    #expect(first.digest == second.digest)
    #expect(first.digest.rawValue == "6c6773c5a824508fd683390aa7a4acdc1636e8c8483f6ac9ee9667bf62d54310")
    #expect(first.content.forceTerms.count == QuadrotorForceTermID.allCases.count)
    #expect(first.content.fidelities.map(\.id) == ["full", "single_prop"])
    #expect(first.content.integration.projectionStages == CanonicalIntegrationStage.allCases)

    let data = try JSONEncoder().encode(first)
    let decoded = try JSONDecoder().decode(CanonicalDynamicsProgram.self, from: data)
    #expect(decoded == first)
}

@Test(.timeLimit(.minutes(1))) func canonicalProgramDecodeRejectsDigestMismatch() throws {
    let program = try ReferenceQuadrotorCanonicalProgram.make()
    let validData = try JSONEncoder().encode(program)
    var object = try #require(JSONSerialization.jsonObject(with: validData) as? [String: Any])
    object["digest"] = String(repeating: "0", count: 64)
    let corruptedData = try JSONSerialization.data(withJSONObject: object)

    #expect(throws: CanonicalDynamicsProgram.ValidationError.self) {
        _ = try JSONDecoder().decode(CanonicalDynamicsProgram.self, from: corruptedData)
    }
}

@Test(.timeLimit(.minutes(1))) func canonicalGraphValidationRejectsDimensionMismatch() throws {
    let layout = try ReferenceQuadrotorCanonicalLayouts.stateLayout()
    let position = try CanonicalValueID("state.position")
    let velocity = try CanonicalValueID("state.velocity")
    let invalidSum = try CanonicalValueID("invalid.sum")
    let graph = try CanonicalOperationGraph(
        id: "invalid_dimension_graph",
        inputs: [
            try CanonicalGraphInput(id: position, layoutID: layout.id, fieldID: "position"),
            try CanonicalGraphInput(id: velocity, layoutID: layout.id, fieldID: "velocity"),
        ],
        instructions: [
            try CanonicalInstruction(
                result: invalidSum,
                opcode: .add,
                operands: [position, velocity]
            ),
        ],
        outputs: [
            try CanonicalGraphOutput(
                id: "invalid",
                value: invalidSum,
                shape: .vector3,
                unit: .meter
            ),
        ]
    )

    #expect(
        throws: CanonicalOperationGraphValidator.ValidationError.dimensionMismatch(invalidSum)
    ) {
        _ = try CanonicalOperationGraphValidator().signatures(for: graph, layouts: [layout])
    }
}

@Test(.timeLimit(.minutes(1))) func canonicalGraphValidationRejectsForwardReferences() throws {
    let layout = try ReferenceQuadrotorCanonicalLayouts.stateLayout()
    let missing = try CanonicalValueID("missing.value")
    let result = try CanonicalValueID("invalid.result")
    let graph = try CanonicalOperationGraph(
        id: "invalid_forward_reference_graph",
        inputs: [],
        instructions: [
            try CanonicalInstruction(result: result, opcode: .negate, operands: [missing]),
        ],
        outputs: [
            try CanonicalGraphOutput(
                id: "invalid",
                value: result,
                shape: .vector3,
                unit: .meter
            ),
        ]
    )

    #expect(throws: CanonicalOperationGraphValidator.ValidationError.unknownOperand(missing)) {
        _ = try CanonicalOperationGraphValidator().signatures(for: graph, layouts: [layout])
    }
}

@Test(.timeLimit(.minutes(1))) func canonicalGraphValidationPropagatesPiecewiseDifferentiability() throws {
    let layout = try ReferenceQuadrotorCanonicalLayouts.stateLayout()
    let velocity = try CanonicalValueID("state.velocity")
    let direction = try CanonicalValueID("state.velocity_direction")
    let graph = try CanonicalOperationGraph(
        id: "piecewise_differentiability_graph",
        inputs: [
            try CanonicalGraphInput(
                id: velocity,
                layoutID: layout.id,
                fieldID: "velocity"
            ),
        ],
        instructions: [
            try CanonicalInstruction(
                result: direction,
                opcode: .normalize3OrZero,
                operands: [velocity]
            ),
        ],
        outputs: [
            try CanonicalGraphOutput(
                id: "direction",
                value: direction,
                shape: .vector3,
                unit: .dimensionless
            ),
        ]
    )

    let signatures = try CanonicalOperationGraphValidator().signatures(
        for: graph,
        layouts: [layout]
    )
    #expect(signatures[direction]?.differentiability == .piecewiseDifferentiable)
}

@Test(.timeLimit(.minutes(1))) func scalarExecutorMatchesGoldenForceTraceAcrossFidelities() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let mixer = ReferenceQuadrotorMixer(
        armLength: parameters.armLength,
        yawCoefficient: parameters.yawCoefficient,
        spinDirections: SIMD4<Double>(1, -1, 1, -1)
    )
    let environment = try WorldEnvironment(
        gravity: 9.7,
        windVelocityWorld: Axis3(x: 1.2, y: -0.4, z: 0.3),
        airPressure: 93_000,
        airTemperature: 279,
        usage: .full
    )
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0.2, -0.3, 1.4),
        velocity: SIMD3<Double>(2.1, -0.7, 0.5),
        orientation: simd_quatd(angle: 0.31, axis: simd_normalize(SIMD3<Double>(1, 2, -1))),
        angularVelocity: SIMD3<Double>(0.6, -0.2, 0.4)
    )
    let thrusts = try MotorThrusts(f1: 1.2, f2: 1.7, f3: 1.1, f4: 1.5)
    let disturbances = DisturbanceState(
        torqueBody: SIMD3<Double>(0.02, -0.01, 0.03),
        forceWorld: SIMD3<Double>(0.4, -0.2, 0.1)
    )
    let program = try ReferenceQuadrotorCanonicalProgram.make()
    let executor = ReferenceQuadrotorScalarDynamicsExecutor()

    for fidelity in [ReferenceQuadrotorFidelity.full, .singleProp] {
        let expected = fidelity == .full
            ? QuadrotorGeneralizedForce(
                bodyForce: SIMD3<Double>(0, 0, 5.213668887244016),
                bodyTorque: SIMD3<Double>(
                    0.031070555144337524,
                    -0.01641527757216875,
                    -0.0030629163582531448
                ),
                worldForce: SIMD3<Double>(
                    0.37095796079738663,
                    -0.19068960543767222,
                    -9.567070859468387
                )
            )
            : QuadrotorGeneralizedForce(
                bodyForce: SIMD3<Double>(0, 0, 5.5),
                bodyTorque: SIMD3<Double>(0.044, -0.021999999999999985, 0.012),
                worldForce: SIMD3<Double>(0.4, -0.2, -9.6)
            )
        let actual = try executor.generalizedForce(
            program: program,
            state: state,
            parameters: parameters,
            mixer: mixer,
            motorThrusts: thrusts,
            disturbances: disturbances,
            environment: environment,
            activeTerms: fidelity.active
        )
        assertForceClose(actual, expected, tolerance: 1e-12)
    }
}

@Test(.timeLimit(.minutes(1))) func scalarExecutorMatchesReferenceDerivativeAndObservables() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let environment = try WorldEnvironment(
        gravity: 9.73,
        windVelocityWorld: Axis3(x: 0.8, y: 0.2, z: -0.1),
        airPressure: 95_000,
        airTemperature: 282,
        usage: .full
    )
    let mixer = ReferenceQuadrotorMixer(
        armLength: parameters.armLength,
        yawCoefficient: parameters.yawCoefficient
    )
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0.1, 0.2, 1.3),
        velocity: SIMD3<Double>(1.1, -0.4, 0.3),
        orientation: simd_quatd(angle: 0.22, axis: simd_normalize(SIMD3<Double>(0.5, -1, 2))),
        angularVelocity: SIMD3<Double>(0.3, -0.5, 0.2)
    )
    let thrusts = try MotorThrusts(f1: 1.3, f2: 1.4, f3: 1.5, f4: 1.6)
    let disturbances = DisturbanceState(
        torqueBody: SIMD3<Double>(0.01, 0.02, -0.03),
        forceWorld: SIMD3<Double>(-0.1, 0.2, 0.05)
    )
    let program = try ReferenceQuadrotorCanonicalProgram.make()
    let executor = ReferenceQuadrotorScalarDynamicsExecutor()
    let force = try executor.generalizedForce(
        program: program,
        state: state,
        parameters: parameters,
        mixer: mixer,
        motorThrusts: thrusts,
        disturbances: disturbances,
        environment: environment,
        activeTerms: ReferenceQuadrotorFidelity.full.active
    )
    let derivative = try executor.derivative(
        program: program,
        state: state,
        parameters: parameters,
        force: force
    )
    let expectedDerivative = ReferenceQuadrotorStateDerivative(
        position: SIMD3<Double>(1.1, -0.4, 0.3),
        velocity: SIMD3<Double>(
            -0.6124869488309767,
            -0.09907953556589011,
            -4.128121649286439
        ),
        orientation: SIMD4<Double>(
            0.1682578860554088,
            -0.23651122988798404,
            0.10059338925578871,
            -0.025153368662499384
        ),
        angularVelocity: SIMD3<Double>(
            -3.7185164653448224,
            10.646516465344817,
            -4.648010783828225
        )
    )
    assertVectorClose(derivative.position, expectedDerivative.position, tolerance: 1e-12)
    assertVectorClose(derivative.velocity, expectedDerivative.velocity, tolerance: 1e-12)
    assertVector4Close(derivative.orientation, expectedDerivative.orientation, tolerance: 1e-12)
    assertVectorClose(derivative.angularVelocity, expectedDerivative.angularVelocity, tolerance: 1e-12)

    let observables = try executor.observables(
        program: program,
        state: state,
        parameters: parameters,
        environment: environment,
        force: force
    )
    let accelerationWorld = (state.orientation.act(force.bodyForce) + force.worldForce) / parameters.mass
    let gravityWorld = SIMD3<Double>(0, 0, -environment.effectiveGravity(defaultGravity: parameters.gravity))
    let expectedSpecificForce = state.orientation.inverse.act(accelerationWorld - gravityWorld)
    assertVectorClose(observables.angularVelocityBody, state.angularVelocity, tolerance: 1e-12)
    assertVectorClose(observables.specificForceBody, expectedSpecificForce, tolerance: 1e-12)
}

private func assertForceClose(
    _ lhs: QuadrotorGeneralizedForce,
    _ rhs: QuadrotorGeneralizedForce,
    tolerance: Double
) {
    assertVectorClose(lhs.bodyForce, rhs.bodyForce, tolerance: tolerance)
    assertVectorClose(lhs.bodyTorque, rhs.bodyTorque, tolerance: tolerance)
    assertVectorClose(lhs.worldForce, rhs.worldForce, tolerance: tolerance)
}

private func assertVectorClose(
    _ lhs: SIMD3<Double>,
    _ rhs: SIMD3<Double>,
    tolerance: Double
) {
    #expect(abs(lhs.x - rhs.x) < tolerance)
    #expect(abs(lhs.y - rhs.y) < tolerance)
    #expect(abs(lhs.z - rhs.z) < tolerance)
}

private func assertVector4Close(
    _ lhs: SIMD4<Double>,
    _ rhs: SIMD4<Double>,
    tolerance: Double
) {
    #expect(abs(lhs.x - rhs.x) < tolerance)
    #expect(abs(lhs.y - rhs.y) < tolerance)
    #expect(abs(lhs.z - rhs.z) < tolerance)
    #expect(abs(lhs.w - rhs.w) < tolerance)
}
