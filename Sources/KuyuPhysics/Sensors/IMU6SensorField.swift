import simd
import KuyuCore

public struct IMU6SensorField: SensorField {
    public enum ValidationError: Error, Equatable {
        case nonFinite(String)
        case negative(String)
    }

    public var parameters: ReferenceQuadrotorParameters {
        didSet { rebuildModel() }
    }
    public var mixer: ReferenceQuadrotorMixer {
        didSet { rebuildModel() }
    }
    public var store: ReferenceQuadrotorWorldStore
    public var timeStep: TimeStep
    public var environment: WorldEnvironment {
        didSet { rebuildModel() }
    }

    private var model: ReferenceQuadrotorPhysicsModel
    private var gyroNoise: [AxisNoiseModel]
    private var accelNoise: [AxisNoiseModel]
    private var delayBuffer: SampleDelayBuffer

    public init(
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer,
        store: ReferenceQuadrotorWorldStore,
        timeStep: TimeStep,
        environment: WorldEnvironment = .standard,
        noiseSeed: UInt64,
        gyroNoiseStdDev: Double,
        gyroBias: Double,
        gyroRandomWalkSigma: Double,
        accelNoiseStdDev: Double,
        accelBias: Double,
        accelRandomWalkSigma: Double,
        delaySteps: UInt64 = 0,
        canonicalExecutor: any ReferenceQuadrotorCanonicalExecuting =
            ReferenceQuadrotorScalarDynamicsExecutor()
    ) throws {
        guard gyroNoiseStdDev.isFinite else { throw ValidationError.nonFinite("gyroNoiseStdDev") }
        guard gyroBias.isFinite else { throw ValidationError.nonFinite("gyroBias") }
        guard gyroRandomWalkSigma.isFinite else { throw ValidationError.nonFinite("gyroRandomWalkSigma") }
        guard accelNoiseStdDev.isFinite else { throw ValidationError.nonFinite("accelNoiseStdDev") }
        guard accelBias.isFinite else { throw ValidationError.nonFinite("accelBias") }
        guard accelRandomWalkSigma.isFinite else { throw ValidationError.nonFinite("accelRandomWalkSigma") }

        guard gyroNoiseStdDev >= 0 else { throw ValidationError.negative("gyroNoiseStdDev") }
        guard gyroRandomWalkSigma >= 0 else { throw ValidationError.negative("gyroRandomWalkSigma") }
        guard accelNoiseStdDev >= 0 else { throw ValidationError.negative("accelNoiseStdDev") }
        guard accelRandomWalkSigma >= 0 else { throw ValidationError.negative("accelRandomWalkSigma") }

        let program = try ReferenceQuadrotorCanonicalProgram.make()
        self.parameters = parameters
        self.mixer = mixer
        self.store = store
        self.timeStep = timeStep
        self.environment = environment
        self.model = ReferenceQuadrotorPhysicsModel(
            parameters: parameters,
            mixer: mixer,
            environment: environment,
            program: program,
            executor: canonicalExecutor
        )
        self.gyroNoise = [
            AxisNoiseModel(bias: gyroBias, noiseStdDev: gyroNoiseStdDev, randomWalkSigma: gyroRandomWalkSigma, seed: noiseSeed &+ 1),
            AxisNoiseModel(bias: gyroBias, noiseStdDev: gyroNoiseStdDev, randomWalkSigma: gyroRandomWalkSigma, seed: noiseSeed &+ 2),
            AxisNoiseModel(bias: gyroBias, noiseStdDev: gyroNoiseStdDev, randomWalkSigma: gyroRandomWalkSigma, seed: noiseSeed &+ 3)
        ]
        self.accelNoise = [
            AxisNoiseModel(bias: accelBias, noiseStdDev: accelNoiseStdDev, randomWalkSigma: accelRandomWalkSigma, seed: noiseSeed &+ 4),
            AxisNoiseModel(bias: accelBias, noiseStdDev: accelNoiseStdDev, randomWalkSigma: accelRandomWalkSigma, seed: noiseSeed &+ 5),
            AxisNoiseModel(bias: accelBias, noiseStdDev: accelNoiseStdDev, randomWalkSigma: accelRandomWalkSigma, seed: noiseSeed &+ 6)
        ]
        self.delayBuffer = SampleDelayBuffer(delaySteps: delaySteps)
    }

    public mutating func sample(time: WorldTime) throws -> [ChannelSample] {
        let observables = try model.observables(
            state: store.state,
            motorThrusts: store.motorThrusts,
            disturbances: store.disturbances,
            fidelity: .full
        )
        let gyro = observables.angularVelocityBody
        let accel = observables.specificForceBody

        let dt = timeStep.delta

        let gyroSamples = SIMD3<Double>(
            gyro.x + gyroNoise[0].sample(delta: dt),
            gyro.y + gyroNoise[1].sample(delta: dt),
            gyro.z + gyroNoise[2].sample(delta: dt)
        )

        let accelSamples = SIMD3<Double>(
            accel.x + accelNoise[0].sample(delta: dt),
            accel.y + accelNoise[1].sample(delta: dt),
            accel.z + accelNoise[2].sample(delta: dt)
        )

        let timestamp = time.time
        let samples = [
            try ChannelSample(channelIndex: 0, value: gyroSamples.x, timestamp: timestamp),
            try ChannelSample(channelIndex: 1, value: gyroSamples.y, timestamp: timestamp),
            try ChannelSample(channelIndex: 2, value: gyroSamples.z, timestamp: timestamp),
            try ChannelSample(channelIndex: 3, value: accelSamples.x, timestamp: timestamp),
            try ChannelSample(channelIndex: 4, value: accelSamples.y, timestamp: timestamp),
            try ChannelSample(channelIndex: 5, value: accelSamples.z, timestamp: timestamp)
        ]

        return delayBuffer.push(samples)
    }

    private mutating func rebuildModel() {
        model = ReferenceQuadrotorPhysicsModel(
            parameters: parameters,
            mixer: mixer,
            environment: environment,
            program: model.program,
            executor: model.executor
        )
    }
}
