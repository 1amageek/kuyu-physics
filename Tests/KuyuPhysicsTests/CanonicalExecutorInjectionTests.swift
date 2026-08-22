import KuyuCore
import KuyuPhysics
import simd
import Synchronization
import Testing

@Suite("Canonical executor injection")
struct CanonicalExecutorInjectionTests {
    @Test(.timeLimit(.minutes(1)))
    func fullPlantAndIMUUseTheInjectedExecutor() throws {
        let executor = RecordingCanonicalExecutor()
        let parameters = ReferenceQuadrotorParameters.baseline
        let mixer = ReferenceQuadrotorMixer(
            armLength: parameters.armLength,
            yawCoefficient: parameters.yawCoefficient
        )
        let store = ReferenceQuadrotorWorldStore(
            state: try zeroState(),
            motorThrusts: try MotorThrusts.uniform(
                parameters.mass * parameters.gravity / 4
            )
        )
        let timeStep = try TimeStep(delta: 0.0025)
        var plant = try ReferenceQuadrotorPlantEngine(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: timeStep,
            canonicalExecutor: executor
        )
        var sensor = try IMU6SensorField(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: timeStep,
            noiseSeed: 7,
            gyroNoiseStdDev: 0,
            gyroBias: 0,
            gyroRandomWalkSigma: 0,
            accelNoiseStdDev: 0,
            accelBias: 0,
            accelRandomWalkSigma: 0,
            canonicalExecutor: executor
        )
        let time = try WorldTime(stepIndex: 1, time: timeStep.delta)

        try plant.integrate(time: time)
        _ = try sensor.sample(time: time)

        #expect(executor.generalizedForceCount > 0)
        #expect(executor.derivativeCount > 0)
        #expect(executor.observablesCount == 1)
    }

    @Test(.timeLimit(.minutes(1)))
    func singlePropPlantUsesTheInjectedExecutor() throws {
        let executor = RecordingCanonicalExecutor()
        let parameters = ReferenceQuadrotorParameters.baseline
        let store = ReferenceQuadrotorWorldStore(
            state: try zeroState(),
            motorThrusts: try MotorThrusts(
                f1: parameters.mass * parameters.gravity,
                f2: 0,
                f3: 0,
                f4: 0
            )
        )
        let timeStep = try TimeStep(delta: 0.0025)
        var plant = try SinglePropPlantEngine(
            parameters: parameters,
            store: store,
            timeStep: timeStep,
            canonicalExecutor: executor
        )

        try plant.integrate(
            time: WorldTime(stepIndex: 1, time: timeStep.delta)
        )

        #expect(executor.generalizedForceCount > 0)
        #expect(executor.derivativeCount > 0)
        #expect(executor.observablesCount == 0)
    }

    @Test(.timeLimit(.minutes(1)))
    func injectedExecutorFailurePropagatesWithoutScalarFallback() throws {
        let parameters = ReferenceQuadrotorParameters.baseline
        let mixer = ReferenceQuadrotorMixer(
            armLength: parameters.armLength,
            yawCoefficient: parameters.yawCoefficient
        )
        let store = ReferenceQuadrotorWorldStore(
            state: try zeroState(),
            motorThrusts: try MotorThrusts.uniform(
                parameters.mass * parameters.gravity / 4
            )
        )
        let timeStep = try TimeStep(delta: 0.0025)
        var plant = try ReferenceQuadrotorPlantEngine(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: timeStep,
            canonicalExecutor: FailingCanonicalExecutor()
        )
        var sensor = try IMU6SensorField(
            parameters: parameters,
            mixer: mixer,
            store: store,
            timeStep: timeStep,
            noiseSeed: 7,
            gyroNoiseStdDev: 0,
            gyroBias: 0,
            gyroRandomWalkSigma: 0,
            accelNoiseStdDev: 0,
            accelBias: 0,
            accelRandomWalkSigma: 0,
            canonicalExecutor: FailingCanonicalExecutor()
        )
        let time = try WorldTime(stepIndex: 1, time: timeStep.delta)

        #expect(throws: CanonicalExecutorFixtureError.forcedFailure) {
            try plant.integrate(time: time)
        }
        #expect(throws: CanonicalExecutorFixtureError.forcedFailure) {
            _ = try sensor.sample(time: time)
        }
    }

    private func zeroState() throws -> ReferenceQuadrotorState {
        try ReferenceQuadrotorState(
            position: .zero,
            velocity: .zero,
            orientation: simd_quatd(
                angle: 0,
                axis: SIMD3<Double>(0, 0, 1)
            ),
            angularVelocity: .zero
        )
    }
}

private enum CanonicalExecutorFixtureError: Error, Equatable {
    case forcedFailure
}

private struct FailingCanonicalExecutor:
    ReferenceQuadrotorCanonicalExecuting,
    Sendable
{
    let executorVersion = "failing-fixture-v1"

    func generalizedForce(
        program _: CanonicalDynamicsProgram,
        state _: ReferenceQuadrotorState,
        parameters _: ReferenceQuadrotorParameters,
        mixer _: ReferenceQuadrotorMixer,
        motorThrusts _: MotorThrusts,
        disturbances _: DisturbanceState,
        environment _: WorldEnvironment,
        activeTerms _: Set<QuadrotorForceTermID>
    ) throws -> QuadrotorGeneralizedForce {
        throw CanonicalExecutorFixtureError.forcedFailure
    }

    func derivative(
        program _: CanonicalDynamicsProgram,
        state _: ReferenceQuadrotorState,
        parameters _: ReferenceQuadrotorParameters,
        force _: QuadrotorGeneralizedForce
    ) throws -> ReferenceQuadrotorStateDerivative {
        throw CanonicalExecutorFixtureError.forcedFailure
    }

    func observables(
        program _: CanonicalDynamicsProgram,
        state _: ReferenceQuadrotorState,
        parameters _: ReferenceQuadrotorParameters,
        environment _: WorldEnvironment,
        force _: QuadrotorGeneralizedForce
    ) throws -> ReferenceQuadrotorCanonicalObservables {
        throw CanonicalExecutorFixtureError.forcedFailure
    }
}

private final class RecordingCanonicalExecutor:
    ReferenceQuadrotorCanonicalExecuting,
    Sendable
{
    private struct InvocationCounts: Sendable {
        var generalizedForce = 0
        var derivative = 0
        var observables = 0
    }

    let executorVersion = "recording-scalar-v1"
    var generalizedForceCount: Int {
        counts.withLock { $0.generalizedForce }
    }
    var derivativeCount: Int {
        counts.withLock { $0.derivative }
    }
    var observablesCount: Int {
        counts.withLock { $0.observables }
    }

    private let base = ReferenceQuadrotorScalarDynamicsExecutor()
    private let counts = Mutex(InvocationCounts())

    func generalizedForce(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState,
        environment: WorldEnvironment,
        activeTerms: Set<QuadrotorForceTermID>
    ) throws -> QuadrotorGeneralizedForce {
        counts.withLock { $0.generalizedForce += 1 }
        return try base.generalizedForce(
            program: program,
            state: state,
            parameters: parameters,
            mixer: mixer,
            motorThrusts: motorThrusts,
            disturbances: disturbances,
            environment: environment,
            activeTerms: activeTerms
        )
    }

    func derivative(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        force: QuadrotorGeneralizedForce
    ) throws -> ReferenceQuadrotorStateDerivative {
        counts.withLock { $0.derivative += 1 }
        return try base.derivative(
            program: program,
            state: state,
            parameters: parameters,
            force: force
        )
    }

    func observables(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        environment: WorldEnvironment,
        force: QuadrotorGeneralizedForce
    ) throws -> ReferenceQuadrotorCanonicalObservables {
        counts.withLock { $0.observables += 1 }
        return try base.observables(
            program: program,
            state: state,
            parameters: parameters,
            environment: environment,
            force: force
        )
    }
}
