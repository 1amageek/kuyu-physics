import simd
import KuyuCore
import KuyuPhysics
import Testing

// MARK: - Sensor bandwidth (low-pass) behavior

@Test(.timeLimit(.minutes(1))) func swappableSensorBandwidthLowPassesStateChannel() async throws {
    let fixture = try makeBandwidthFixture(altitude: 2.0)
    let bandwidth = try SensorSwapEvent(
        kind: .bandwidthChange,
        startTime: 0.0,
        duration: 1.0,
        targetChannels: [6],
        gainScale: 1.0,
        biasShift: 0.0,
        noiseScale: 1.0,
        dropoutProbability: 0.0,
        delayShiftSteps: 0,
        bandwidthScale: 0.5
    )
    var sensor = SwappableSensorField(
        base: fixture.baseSensor,
        swapEvents: [.sensor(bandwidth)],
        hfEvents: [],
        baseNoise: fixture.baseNoise,
        seed: 7,
        stateChannelStore: fixture.store
    )

    // Step 1 primes the filter at altitude 2.0.
    let first = try sensor.sample(time: try WorldTime(stepIndex: 1, time: fixture.timeStep.delta))
    #expect(abs(try sampleValue(first, channelIndex: 6) - 2.0) < 1e-9)

    // Step a hard altitude change to 4.0 — a 0.5 bandwidth must lag halfway to 3.0.
    fixture.store.state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0, 0, 4.0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
    let second = try sensor.sample(time: try WorldTime(stepIndex: 2, time: fixture.timeStep.delta * 2))
    #expect(abs(try sampleValue(second, channelIndex: 6) - 3.0) < 1e-9)
}

@Test(.timeLimit(.minutes(1))) func swappableSensorBandwidthUnityIsPassthrough() async throws {
    let fixture = try makeBandwidthFixture(altitude: 2.0)
    var sensor = SwappableSensorField(
        base: fixture.baseSensor,
        swapEvents: [],
        hfEvents: [],
        baseNoise: fixture.baseNoise,
        seed: 7,
        stateChannelStore: fixture.store
    )
    _ = try sensor.sample(time: try WorldTime(stepIndex: 1, time: fixture.timeStep.delta))
    fixture.store.state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0, 0, 4.0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
    let second = try sensor.sample(time: try WorldTime(stepIndex: 2, time: fixture.timeStep.delta * 2))
    // No swap → no filtering → channel reads the raw stepped altitude.
    #expect(abs(try sampleValue(second, channelIndex: 6) - 4.0) < 1e-9)
}

// MARK: - Actuator rate limit + asymmetry behavior

@Test(.timeLimit(.minutes(1))) func swappableActuatorRateLimitClampsPerStepDelta() async throws {
    let recorder = RecordingActuatorEngine()
    let baseMax = try MotorMaxThrusts.uniform(10.0)
    let rateLimit = try ActuatorSwapEvent(
        kind: .rateLimitShift,
        startTime: 0.0,
        duration: 10.0,
        motorIndex: 0,
        gainScale: 1.0,
        lagScale: 1.0,
        maxOutputScale: 1.0,
        deadzoneShift: 0.0,
        rateLimitScale: 0.1,
        asymmetryScale: 1.0
    )
    var engine = SwappableActuatorEngine(
        engine: recorder,
        baseMaxThrusts: baseMax,
        swapEvents: [.actuator(rateLimit)],
        hfEvents: []
    )

    let time = try WorldTime(stepIndex: 1, time: 0.001)
    // Prime the baseline at 0 (the first command has no previous reference to limit against).
    try engine.apply(values: [try ActuatorValue(index: ActuatorIndex(0), value: 0.0)], time: time)
    #expect(abs((engine.engine.lastApplied.first?.value ?? -1) - 0.0) < 1e-9)

    // Command full scale: per-step delta must clamp to 10.0 * 0.1 = 1.0.
    try engine.apply(values: [try ActuatorValue(index: ActuatorIndex(0), value: 10.0)], time: time)
    #expect(abs((engine.engine.lastApplied.first?.value ?? -1) - 1.0) < 1e-9)

    // Next step continues to ramp by at most 1.0 per step.
    try engine.apply(values: [try ActuatorValue(index: ActuatorIndex(0), value: 10.0)], time: time)
    #expect(abs((engine.engine.lastApplied.first?.value ?? -1) - 2.0) < 1e-9)
}

@Test(.timeLimit(.minutes(1))) func swappableActuatorAsymmetrySlowsRisingEdge() async throws {
    let recorder = RecordingActuatorEngine()
    let baseMax = try MotorMaxThrusts.uniform(10.0)
    let asymmetry = try ActuatorSwapEvent(
        kind: .asymmetryShift,
        startTime: 0.0,
        duration: 10.0,
        motorIndex: 0,
        gainScale: 1.0,
        lagScale: 1.0,
        maxOutputScale: 1.0,
        deadzoneShift: 0.0,
        rateLimitScale: 1.0,
        asymmetryScale: 0.5
    )
    var engine = SwappableActuatorEngine(
        engine: recorder,
        baseMaxThrusts: baseMax,
        swapEvents: [.actuator(asymmetry)],
        hfEvents: []
    )

    let time = try WorldTime(stepIndex: 1, time: 0.001)
    // Prime baseline at 2.0.
    try engine.apply(values: [try ActuatorValue(index: ActuatorIndex(0), value: 2.0)], time: time)
    // Rising command 2.0 -> 6.0; asymmetry 0.5 → 2.0 + 0.5*(6.0-2.0) = 4.0.
    try engine.apply(values: [try ActuatorValue(index: ActuatorIndex(0), value: 6.0)], time: time)
    #expect(abs((engine.engine.lastApplied.first?.value ?? -1) - 4.0) < 1e-9)
    // Falling command is unaffected: 4.0 -> 1.0 passes through.
    try engine.apply(values: [try ActuatorValue(index: ActuatorIndex(0), value: 1.0)], time: time)
    #expect(abs((engine.engine.lastApplied.first?.value ?? -1) - 1.0) < 1e-9)
}

// MARK: - Fixtures

private final class RecordingActuatorEngine: ActuatorEngine {
    var lastApplied: [ActuatorValue] = []
    func update(time: WorldTime) throws {}
    func apply(values: [ActuatorValue], time: WorldTime) throws { lastApplied = values }
    func telemetrySnapshot() -> ActuatorTelemetrySnapshot { ActuatorTelemetrySnapshot(channels: []) }
}

private struct BandwidthFixture {
    let baseSensor: IMU6SensorField
    let baseNoise: IMU6NoiseConfig
    let store: ReferenceQuadrotorWorldStore
    let timeStep: TimeStep
}

private func makeBandwidthFixture(altitude: Double) throws -> BandwidthFixture {
    let params = ReferenceQuadrotorParameters.baseline
    let mixer = ReferenceQuadrotorMixer(armLength: params.armLength, yawCoefficient: params.yawCoefficient)
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0, 0, altitude),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
    let hoverThrust = params.mass * params.gravity / 4.0
    let store = ReferenceQuadrotorWorldStore(state: state, motorThrusts: try MotorThrusts.uniform(hoverThrust))
    let timeStep = try TimeStep(delta: 0.001)
    let baseNoise = try IMU6NoiseConfig(
        gyroNoiseStdDev: 0, gyroBias: 0, gyroRandomWalkSigma: 0,
        accelNoiseStdDev: 0, accelBias: 0, accelRandomWalkSigma: 0, delaySteps: 0
    )
    let baseSensor = try IMU6SensorField(
        parameters: params,
        mixer: mixer,
        store: store,
        timeStep: timeStep,
        noiseSeed: 42,
        gyroNoiseStdDev: baseNoise.gyroNoiseStdDev,
        gyroBias: baseNoise.gyroBias,
        gyroRandomWalkSigma: baseNoise.gyroRandomWalkSigma,
        accelNoiseStdDev: baseNoise.accelNoiseStdDev,
        accelBias: baseNoise.accelBias,
        accelRandomWalkSigma: baseNoise.accelRandomWalkSigma,
        delaySteps: baseNoise.delaySteps
    )
    return BandwidthFixture(baseSensor: baseSensor, baseNoise: baseNoise, store: store, timeStep: timeStep)
}

private enum ModifierTestError: Error {
    case missingSample(UInt32)
}

private func sampleValue(_ samples: [ChannelSample], channelIndex: UInt32) throws -> Double {
    for sample in samples where sample.channelIndex == channelIndex {
        return sample.value
    }
    throw ModifierTestError.missingSample(channelIndex)
}
