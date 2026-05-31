import simd
import KuyuCore
import KuyuPhysics
import Testing

@Test(.timeLimit(.minutes(1))) func singlePropPlantUsesCanonicalVerticalFidelity() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let initialState = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0.3, -0.2, 1.0),
        velocity: SIMD3<Double>(0.4, -0.5, 0.2),
        orientation: simd_quatd(angle: 0.4, axis: SIMD3<Double>(1, 0, 0)),
        angularVelocity: SIMD3<Double>(0.2, -0.1, 0.3)
    )
    let thrusts = try MotorThrusts(f1: parameters.mass * parameters.gravity * 1.15, f2: 0, f3: 0, f4: 0)
    let disturbances = DisturbanceState(
        torqueBody: SIMD3<Double>(0.1, 0.2, -0.1),
        forceWorld: SIMD3<Double>(1.0, -0.5, 0.25)
    )
    let timeStep = try TimeStep(delta: 0.01)
    let store = ReferenceQuadrotorWorldStore(
        state: initialState,
        motorThrusts: thrusts,
        disturbances: disturbances
    )
    var plant = SinglePropPlantEngine(
        parameters: parameters,
        store: store,
        timeStep: timeStep
    )

    try plant.integrate(time: try WorldTime(stepIndex: 1, time: timeStep.delta))

    let model = ReferenceQuadrotorPhysicsModel(
        parameters: parameters,
        mixer: ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient)
    )
    let expected = try ReferenceQuadrotorCanonicalIntegrator().step(
        state: initialState,
        model: model,
        motorThrusts: thrusts,
        disturbances: disturbances,
        fidelity: .singleProp,
        delta: timeStep.delta
    )

    assertStateApproximatelyEqual(store.state, expected, tolerance: 1e-12)
    #expect(abs(store.state.position.x) < 1e-12)
    #expect(abs(store.state.position.y) < 1e-12)
    #expect(abs(store.state.velocity.x) < 1e-12)
    #expect(abs(store.state.velocity.y) < 1e-12)
    #expect(abs(store.state.angularVelocity.x) < 1e-12)
    #expect(abs(store.state.angularVelocity.y) < 1e-12)
    #expect(abs(store.state.angularVelocity.z) < 1e-12)
}

@Test(.timeLimit(.minutes(1))) func refinementNestingMatchesProjectedHighFidelityTrajectory() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let model = ReferenceQuadrotorPhysicsModel(
        parameters: parameters,
        mixer: ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient)
    )
    let thrusts = try MotorThrusts.uniform(parameters.mass * parameters.gravity / 4.0 * 1.08)
    let disturbances = DisturbanceState.zero
    let delta = 0.005
    var lowState = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0, 0, 1.0),
        velocity: SIMD3<Double>(0, 0, 0.1),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
    var highState = lowState
    let low = ReferenceQuadrotorFidelity.singleProp
    let high = ReferenceQuadrotorFidelity.full
    let integrator = ReferenceQuadrotorCanonicalIntegrator()

    for _ in 0..<20 {
        lowState = try integrator.step(
            state: lowState,
            model: model,
            motorThrusts: thrusts,
            disturbances: disturbances,
            fidelity: low,
            delta: delta
        )
        highState = try integrator.step(
            state: highState,
            model: model,
            motorThrusts: thrusts,
            disturbances: disturbances,
            fidelity: high,
            delta: delta
        )
        let projectedHigh = QuadrotorConstraintProjection.verticalOnly.project(state: highState)
        assertStateApproximatelyEqual(lowState, projectedHigh, tolerance: 1e-12)
    }
}

@Test(.timeLimit(.minutes(1))) func residualTargetEqualsHighLowForceGap() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let environment = try WorldEnvironment(
        gravity: parameters.gravity,
        windVelocityWorld: Axis3(x: 1.0, y: -0.25, z: 0.1),
        airPressure: 92_000.0,
        airTemperature: 280.0,
        usage: .full
    )
    let model = ReferenceQuadrotorPhysicsModel(
        parameters: parameters,
        mixer: ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient),
        environment: environment
    )
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0.1, -0.2, 1.4),
        velocity: SIMD3<Double>(2.0, -1.0, 0.4),
        orientation: simd_quatd(angle: 0.2, axis: SIMD3<Double>(0, 1, 0)),
        angularVelocity: SIMD3<Double>(0.6, -0.4, 0.2)
    )
    let thrusts = try MotorThrusts(f1: 1.5, f2: 1.2, f3: 1.8, f4: 1.1)
    let disturbances = DisturbanceState(
        torqueBody: SIMD3<Double>(0.01, -0.02, 0.03),
        forceWorld: SIMD3<Double>(0.5, -0.25, 0.1)
    )
    let low = ReferenceQuadrotorFidelity.singleProp
    let high = ReferenceQuadrotorFidelity.full

    let lowForce = try model.generalizedForce(
        state: state,
        motorThrusts: thrusts,
        disturbances: disturbances,
        fidelity: low
    )
    let highForce = try model.generalizedForce(
        state: state,
        motorThrusts: thrusts,
        disturbances: disturbances,
        fidelity: high
    )
    let residual = try model.residualTarget(
        low: low,
        high: high,
        state: state,
        motorThrusts: thrusts,
        disturbances: disturbances
    )

    assertForceApproximatelyEqual(residual, highForce - lowForce, tolerance: 1e-12)
}

@Test(.timeLimit(.minutes(1))) func residualTargetExcludesIgnoredTerms() throws {
    let ignored = Set(QuadrotorForceTermID.allCases).subtracting([.gravity, .propulsion])
    let low = try ReferenceQuadrotorFidelity(
        active: [.gravity],
        worldModelTargets: [.propulsion],
        ignoredByNegligibilityPolicy: ignored,
        constraint: .free
    )

    #expect(low.residualTargetIDs(toward: .full) == [.propulsion])
}

@Test(.timeLimit(.minutes(1))) func canonicalIntegratorRejectsImplicitTermsUntilImplicitSolverExists() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let model = ReferenceQuadrotorPhysicsModel(
        parameters: parameters,
        mixer: ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient),
        terms: [
            AnyQuadrotorForceTerm(id: .gravity, stiffness: .implicit) { _ in .zero }
        ]
    )
    let fidelity = try ReferenceQuadrotorFidelity(
        active: [.gravity],
        worldModelTargets: [],
        ignoredByNegligibilityPolicy: Set(QuadrotorForceTermID.allCases).subtracting([.gravity]),
        constraint: .free
    )
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(repeating: 0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
    let thrusts = try MotorThrusts.uniform(0)

    #expect(throws: ReferenceQuadrotorCanonicalIntegrator.IntegrationError.unsupportedImplicitTerms([.gravity])) {
        _ = try ReferenceQuadrotorCanonicalIntegrator().step(
            state: state,
            model: model,
            motorThrusts: thrusts,
            disturbances: .zero,
            fidelity: fidelity,
            delta: 0.01
        )
    }
}

@Test(.timeLimit(.minutes(1))) func canonicalIntegratorRejectsMissingActiveForceTerms() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let model = ReferenceQuadrotorPhysicsModel(
        parameters: parameters,
        mixer: ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient),
        terms: []
    )
    let fidelity = try ReferenceQuadrotorFidelity(
        active: [.gravity],
        worldModelTargets: [],
        ignoredByNegligibilityPolicy: Set(QuadrotorForceTermID.allCases).subtracting([.gravity]),
        constraint: .free
    )
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(repeating: 0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
    let thrusts = try MotorThrusts.uniform(0)

    #expect(throws: ReferenceQuadrotorCanonicalIntegrator.IntegrationError.missingForceTerms([.gravity])) {
        _ = try ReferenceQuadrotorCanonicalIntegrator().step(
            state: state,
            model: model,
            motorThrusts: thrusts,
            disturbances: .zero,
            fidelity: fidelity,
            delta: 0.01
        )
    }
}

@Test(.timeLimit(.minutes(1))) func canonicalIntegratorRejectsInvalidTimeStep() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let model = ReferenceQuadrotorPhysicsModel(
        parameters: parameters,
        mixer: ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient)
    )
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(repeating: 0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )

    #expect(throws: ReferenceQuadrotorCanonicalIntegrator.IntegrationError.invalidTimeStep(0)) {
        _ = try ReferenceQuadrotorCanonicalIntegrator().step(
            state: state,
            model: model,
            motorThrusts: try MotorThrusts.uniform(0),
            disturbances: .zero,
            fidelity: .singleProp,
            delta: 0
        )
    }
}

@Test(.timeLimit(.minutes(1))) func canonicalIntegratorRejectsDuplicateActiveForceTerms() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let model = ReferenceQuadrotorPhysicsModel(
        parameters: parameters,
        mixer: ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient),
        terms: [
            AnyQuadrotorForceTerm(id: .gravity) { _ in .zero },
            AnyQuadrotorForceTerm(id: .gravity) { _ in .zero },
        ]
    )
    let fidelity = try ReferenceQuadrotorFidelity(
        active: [.gravity],
        worldModelTargets: [],
        ignoredByNegligibilityPolicy: Set(QuadrotorForceTermID.allCases).subtracting([.gravity]),
        constraint: .free
    )
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(repeating: 0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )

    #expect(throws: ReferenceQuadrotorCanonicalIntegrator.IntegrationError.duplicateForceTerms([.gravity])) {
        _ = try ReferenceQuadrotorCanonicalIntegrator().step(
            state: state,
            model: model,
            motorThrusts: try MotorThrusts.uniform(0),
            disturbances: .zero,
            fidelity: fidelity,
            delta: 0.01
        )
    }
}

@Test(.timeLimit(.minutes(1))) func physicsModelRejectsMissingActiveForceTerms() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let model = ReferenceQuadrotorPhysicsModel(
        parameters: parameters,
        mixer: ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient),
        terms: []
    )
    let fidelity = try ReferenceQuadrotorFidelity(
        active: [.gravity],
        worldModelTargets: [],
        ignoredByNegligibilityPolicy: Set(QuadrotorForceTermID.allCases).subtracting([.gravity]),
        constraint: .free
    )
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(repeating: 0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )

    #expect(throws: ReferenceQuadrotorPhysicsModel.ModelError.missingForceTerms([.gravity])) {
        _ = try model.derivative(
            state: state,
            motorThrusts: try MotorThrusts.uniform(0),
            disturbances: .zero,
            fidelity: fidelity
        )
    }
}

@Test(.timeLimit(.minutes(1))) func physicsModelRejectsDuplicateActiveForceTerms() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let model = ReferenceQuadrotorPhysicsModel(
        parameters: parameters,
        mixer: ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient),
        terms: [
            AnyQuadrotorForceTerm(id: .gravity) { _ in .zero },
            AnyQuadrotorForceTerm(id: .gravity) { _ in .zero },
        ]
    )
    let fidelity = try ReferenceQuadrotorFidelity(
        active: [.gravity],
        worldModelTargets: [],
        ignoredByNegligibilityPolicy: Set(QuadrotorForceTermID.allCases).subtracting([.gravity]),
        constraint: .free
    )
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(repeating: 0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )

    #expect(throws: ReferenceQuadrotorPhysicsModel.ModelError.duplicateForceTerms([.gravity])) {
        _ = try model.derivative(
            state: state,
            motorThrusts: try MotorThrusts.uniform(0),
            disturbances: .zero,
            fidelity: fidelity
        )
    }
}

@Test(.timeLimit(.minutes(1))) func singlePropDerivativeUsesInjectedForceTerms() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let model = ReferenceQuadrotorPhysicsModel(
        parameters: parameters,
        mixer: ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient),
        terms: [
            AnyQuadrotorForceTerm(id: .gravity) { _ in .zero },
            AnyQuadrotorForceTerm(id: .propulsion) { _ in
                QuadrotorGeneralizedForce(bodyForce: SIMD3<Double>(0, 0, 2.0))
            },
            AnyQuadrotorForceTerm(id: .disturbance) { _ in .zero },
        ]
    )
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(repeating: 0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
    let derivative = try model.derivative(
        state: state,
        motorThrusts: try MotorThrusts.uniform(0),
        disturbances: .zero,
        fidelity: .singleProp
    )

    #expect(abs(derivative.velocity.z - (2.0 / parameters.mass)) < 1e-12)
}

@Test(.timeLimit(.minutes(1))) func fullPlantEngineMatchesLegacyRK4WithoutAtmosphere() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let mixer = ReferenceQuadrotorMixer(armLength: parameters.armLength, yawCoefficient: parameters.yawCoefficient)
    let initialState = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0.2, -0.1, 1.1),
        velocity: SIMD3<Double>(0.05, -0.02, 0.1),
        orientation: simd_quatd(angle: 0.1, axis: SIMD3<Double>(0, 1, 0)),
        angularVelocity: SIMD3<Double>(0.02, -0.03, 0.04)
    )
    let thrusts = try MotorThrusts(f1: 1.1, f2: 1.3, f3: 1.2, f4: 1.4)
    let disturbances = DisturbanceState(
        torqueBody: SIMD3<Double>(0.01, -0.02, 0.03),
        forceWorld: SIMD3<Double>(0.2, -0.1, 0.05)
    )
    let timeStep = try TimeStep(delta: 0.01)
    let store = ReferenceQuadrotorWorldStore(
        state: initialState,
        motorThrusts: thrusts,
        disturbances: disturbances
    )
    var plant = ReferenceQuadrotorPlantEngine(
        parameters: parameters,
        mixer: mixer,
        store: store,
        timeStep: timeStep
    )

    try plant.integrate(time: try WorldTime(stepIndex: 1, time: timeStep.delta))

    let mix = mixer.mix(thrusts: thrusts)
    let expected = ReferenceQuadrotorDynamics.integrateRK4(
        state: initialState,
        input: ReferenceQuadrotorInput(
            bodyForce: mix.forceBody,
            bodyTorque: mix.torqueBody + disturbances.torqueBody,
            worldForce: disturbances.forceWorld
        ),
        parameters: parameters,
        gravity: parameters.gravity,
        delta: timeStep.delta
    )
    assertStateApproximatelyEqual(store.state, expected, tolerance: 1e-12)
}

private func assertStateApproximatelyEqual(
    _ lhs: ReferenceQuadrotorState,
    _ rhs: ReferenceQuadrotorState,
    tolerance: Double
) {
    assertVectorApproximatelyEqual(lhs.position, rhs.position, tolerance: tolerance)
    assertVectorApproximatelyEqual(lhs.velocity, rhs.velocity, tolerance: tolerance)
    assertVectorApproximatelyEqual(lhs.angularVelocity, rhs.angularVelocity, tolerance: tolerance)
    assertQuaternionApproximatelyEqual(lhs.orientation, rhs.orientation, tolerance: tolerance)
}

private func assertForceApproximatelyEqual(
    _ lhs: QuadrotorGeneralizedForce,
    _ rhs: QuadrotorGeneralizedForce,
    tolerance: Double
) {
    assertVectorApproximatelyEqual(lhs.bodyForce, rhs.bodyForce, tolerance: tolerance)
    assertVectorApproximatelyEqual(lhs.bodyTorque, rhs.bodyTorque, tolerance: tolerance)
    assertVectorApproximatelyEqual(lhs.worldForce, rhs.worldForce, tolerance: tolerance)
}

private func assertVectorApproximatelyEqual(
    _ lhs: SIMD3<Double>,
    _ rhs: SIMD3<Double>,
    tolerance: Double
) {
    #expect(abs(lhs.x - rhs.x) < tolerance)
    #expect(abs(lhs.y - rhs.y) < tolerance)
    #expect(abs(lhs.z - rhs.z) < tolerance)
}

private func assertQuaternionApproximatelyEqual(
    _ lhs: simd_quatd,
    _ rhs: simd_quatd,
    tolerance: Double
) {
    #expect(abs(lhs.vector.x - rhs.vector.x) < tolerance)
    #expect(abs(lhs.vector.y - rhs.vector.y) < tolerance)
    #expect(abs(lhs.vector.z - rhs.vector.z) < tolerance)
    #expect(abs(lhs.vector.w - rhs.vector.w) < tolerance)
}
