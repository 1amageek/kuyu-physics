import simd
import KuyuCore
import KuyuPhysics
import Testing

@Test(.timeLimit(.minutes(1))) func swappableSensorAppliesAllStateChannelSwapModifiers() async throws {
    let fixture = try makeStateChannelSensorFixture(
        altitude: 2.25,
        verticalVelocity: -0.5,
        accelNoiseStdDev: 0.25
    )
    let swap = try SensorSwapEvent(
        kind: .driftChange,
        startTime: 0.0,
        duration: 1.0,
        targetChannels: [6],
        gainScale: 2.0,
        biasShift: 0.5,
        noiseScale: 2.0,
        dropoutProbability: 0.0,
        delayShiftSteps: 0
    )
    var sensor = SwappableSensorField(
        base: fixture.baseSensor,
        swapEvents: [.sensor(swap)],
        hfEvents: [],
        baseNoise: fixture.baseNoise,
        seed: 99,
        stateChannelStore: fixture.store
    )

    let samples = try sensor.sample(time: try WorldTime(stepIndex: 1, time: fixture.timeStep.delta))
    let altitude = try sampleValue(samples, channelIndex: 6)
    let verticalVelocity = try sampleValue(samples, channelIndex: 7)

    #expect(abs(altitude - 5.0) > 1e-9)
    #expect(verticalVelocity == -0.5)
}

@Test(.timeLimit(.minutes(1))) func swappableSensorAppliesStateChannelDropoutAndLatency() async throws {
    let fixture = try makeStateChannelSensorFixture(altitude: 2.25, verticalVelocity: -0.5)
    let latency = try SensorSwapEvent(
        kind: .latencyChange,
        startTime: 0.0,
        duration: 1.0,
        targetChannels: [6],
        gainScale: 1.0,
        biasShift: 0.0,
        noiseScale: 1.0,
        dropoutProbability: 0.0,
        delayShiftSteps: 1
    )
    let dropout = try SensorSwapEvent(
        kind: .dropoutBurst,
        startTime: 0.0,
        duration: 1.0,
        targetChannels: [7],
        gainScale: 1.0,
        biasShift: 0.0,
        noiseScale: 1.0,
        dropoutProbability: 1.0,
        delayShiftSteps: 0
    )
    var sensor = SwappableSensorField(
        base: fixture.baseSensor,
        swapEvents: [.sensor(latency), .sensor(dropout)],
        hfEvents: [],
        baseNoise: fixture.baseNoise,
        seed: 99,
        stateChannelStore: fixture.store
    )

    let first = try sensor.sample(time: try WorldTime(stepIndex: 1, time: fixture.timeStep.delta))
    fixture.store.state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0, 0, 3.5),
        velocity: SIMD3<Double>(0, 0, 0.25),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
    let second = try sensor.sample(time: try WorldTime(stepIndex: 2, time: fixture.timeStep.delta * 2))

    #expect(first.isEmpty)
    #expect(try sampleValue(second, channelIndex: 6) == 2.25)
    #expect(!second.contains { $0.channelIndex == 7 })
}

@Test(.timeLimit(.minutes(1))) func swappableSensorAppliesHFStateChannelGlitchAndLatency() async throws {
    let fixture = try makeStateChannelSensorFixture(altitude: 2.25, verticalVelocity: -0.5)
    let glitch = try HFStressEvent(
        kind: .sensorGlitch,
        startTime: 0.0,
        duration: 1.0,
        magnitude: 0.75
    )
    let latency = try HFStressEvent(
        kind: .latencySpike,
        startTime: 0.0,
        duration: 1.0,
        magnitude: 1.0
    )
    var sensor = SwappableSensorField(
        base: fixture.baseSensor,
        swapEvents: [],
        hfEvents: [glitch, latency],
        baseNoise: fixture.baseNoise,
        seed: 99,
        stateChannelStore: fixture.store
    )

    let first = try sensor.sample(time: try WorldTime(stepIndex: 1, time: fixture.timeStep.delta))
    let second = try sensor.sample(time: try WorldTime(stepIndex: 2, time: fixture.timeStep.delta * 2))
    let altitude = try sampleValue(second, channelIndex: 6)
    let verticalVelocity = try sampleValue(second, channelIndex: 7)

    #expect(first.isEmpty)
    #expect(abs(abs(altitude - 2.25) - 0.75) < 1e-9)
    #expect(abs(abs(verticalVelocity + 0.5) - 0.75) < 1e-9)
}

private struct StateChannelSensorFixture {
    let baseSensor: IMU6SensorField
    let baseNoise: IMU6NoiseConfig
    let store: ReferenceQuadrotorWorldStore
    let timeStep: TimeStep
}

private func makeStateChannelSensorFixture(
    altitude: Double,
    verticalVelocity: Double,
    accelNoiseStdDev: Double = 0
) throws -> StateChannelSensorFixture {
    let params = ReferenceQuadrotorParameters.baseline
    let mixer = ReferenceQuadrotorMixer(armLength: params.armLength, yawCoefficient: params.yawCoefficient)
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0, 0, altitude),
        velocity: SIMD3<Double>(0, 0, verticalVelocity),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
    let hoverThrust = params.mass * params.gravity / 4.0
    let store = ReferenceQuadrotorWorldStore(state: state, motorThrusts: try MotorThrusts.uniform(hoverThrust))
    let timeStep = try TimeStep(delta: 0.001)
    let baseNoise = try IMU6NoiseConfig(
        gyroNoiseStdDev: 0,
        gyroBias: 0,
        gyroRandomWalkSigma: 0,
        accelNoiseStdDev: accelNoiseStdDev,
        accelBias: 0,
        accelRandomWalkSigma: 0,
        delaySteps: 0
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
    return StateChannelSensorFixture(
        baseSensor: baseSensor,
        baseNoise: baseNoise,
        store: store,
        timeStep: timeStep
    )
}

private enum SensorTestError: Error {
    case missingSample(UInt32)
}

private func sampleValue(_ samples: [ChannelSample], channelIndex: UInt32) throws -> Double {
    for sample in samples where sample.channelIndex == channelIndex {
        return sample.value
    }
    throw SensorTestError.missingSample(channelIndex)
}
