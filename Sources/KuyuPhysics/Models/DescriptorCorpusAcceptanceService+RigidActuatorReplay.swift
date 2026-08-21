import EmbodimentContract
import Foundation
import KuyuCore
import simd

extension DescriptorCorpusAcceptanceService {
    func rigidActuatorReplayLog(entry: DescriptorCorpusEntry) throws -> SimulationLog {
        try validateRigidReplayWorld(entry: entry)
        let stepCount = try exactStepCount(duration: entry.duration, timeStep: entry.timeStep.delta)
        let parameters = try rigidReplayParameters(entry: entry)
        let initialState = try ReferenceQuadrotorState(
            position: SIMD3<Double>(0.0, 0.0, 1.0),
            velocity: SIMD3<Double>(repeating: 0.0),
            orientation: simd_quatd(angle: 0.0, axis: SIMD3<Double>(0.0, 0.0, 1.0)),
            angularVelocity: SIMD3<Double>(repeating: 0.0)
        )
        let store = ReferenceQuadrotorWorldStore(
            state: initialState,
            motorThrusts: .zero
        )
        let environment = try worldEnvironment(world: entry.world)
        var motorNerve = try MotorNerveChain(contract: entry.embodiment)
        let actuatorCount = entry.embodiment.signals.actuator.count
        let driveCount = entry.embodiment.control.driveChannels.count
        let hover = min(max(parameters.mass * parameters.gravity / Double(max(driveCount, 1)), 0.0), parameters.maxThrust)
        let amplitude = min(parameters.maxThrust * 0.05, max(0.0, parameters.maxThrust - hover), hover)
        var logs: [WorldStepLog] = []
        logs.reserveCapacity(stepCount)

        if driveCount == 1 {
            var actuator = SinglePropActuatorEngine(
                maxThrust: parameters.maxThrust,
                motorTimeConstant: parameters.motorTimeConstant,
                store: store,
                timeStep: entry.timeStep
            )
            var plant = try SinglePropPlantEngine(
                parameters: parameters,
                store: store,
                timeStep: entry.timeStep,
                environment: environment
            )
            for step in 0..<stepCount {
                let log = try rigidActuatorStepLog(
                    step: step,
                    stepCount: stepCount,
                    entry: entry,
                    hover: hover,
                    amplitude: amplitude,
                    motorNerve: &motorNerve,
                    actuatorTelemetry: actuator.telemetrySnapshot(),
                    actuatorApply: { values, time in
                        try actuator.apply(values: values, time: time)
                        try actuator.update(time: time)
                    },
                    plantIntegrate: { time in
                        try plant.integrate(time: time)
                    },
                    plantSnapshot: {
                        plant.snapshot()
                    },
                    safetyTrace: {
                        try plant.safetyTrace()
                    },
                    updatedActuatorTelemetry: {
                        actuator.telemetrySnapshot()
                    }
                )
                logs.append(log)
            }
        } else if driveCount == 4, actuatorCount == 4 {
            var actuator = ReferenceQuadrotorActuatorEngine(
                parameters: parameters,
                store: store,
                timeStep: entry.timeStep
            )
            var plant = try ReferenceQuadrotorPlantEngine(
                parameters: parameters,
                mixer: ReferenceQuadrotorMixer(
                    armLength: parameters.armLength,
                    yawCoefficient: parameters.yawCoefficient
                ),
                store: store,
                timeStep: entry.timeStep,
                environment: environment
            )
            for step in 0..<stepCount {
                let log = try rigidActuatorStepLog(
                    step: step,
                    stepCount: stepCount,
                    entry: entry,
                    hover: hover,
                    amplitude: amplitude,
                    motorNerve: &motorNerve,
                    actuatorTelemetry: actuator.telemetrySnapshot(),
                    actuatorApply: { values, time in
                        try actuator.apply(values: values, time: time)
                        try actuator.update(time: time)
                    },
                    plantIntegrate: { time in
                        try plant.integrate(time: time)
                    },
                    plantSnapshot: {
                        plant.snapshot()
                    },
                    safetyTrace: {
                        try plant.safetyTrace()
                    },
                    updatedActuatorTelemetry: {
                        actuator.telemetrySnapshot()
                    }
                )
                logs.append(log)
            }
        } else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("rigid-actuator-channel-count")
        }

        return SimulationLog(
            scenarioId: try ScenarioID("\(entry.body.bodyID.uppercased())-RIGID-DYN-1"),
            seed: entry.seed,
            timeStep: entry.timeStep,
            determinism: entry.determinism,
            configHash: try rigidReplayConfigHash(entry: entry, parameters: parameters),
            events: logs
        )
    }

    func rigidActuatorStepLog(
        step: Int,
        stepCount: Int,
        entry: DescriptorCorpusEntry,
        hover: Double,
        amplitude: Double,
        motorNerve: inout MotorNerveChain,
        actuatorTelemetry: ActuatorTelemetrySnapshot,
        actuatorApply: ([ActuatorValue], WorldTime) throws -> Void,
        plantIntegrate: (WorldTime) throws -> Void,
        plantSnapshot: () -> PlantStateSnapshot,
        safetyTrace: () throws -> SafetyTrace,
        updatedActuatorTelemetry: () -> ActuatorTelemetrySnapshot
    ) throws -> WorldStepLog {
        let time = try WorldTime(stepIndex: UInt64(step), time: Double(step) * entry.timeStep.delta)
        let drives = try rigidDriveIntents(
            time: time.time,
            stepCount: stepCount,
            driveCount: entry.embodiment.control.driveChannels.count,
            hover: hover,
            amplitude: amplitude
        )
        let actuatorValues = try motorNerve.update(
            input: drives,
            corrections: [],
            telemetry: MotorNerveTelemetry(actuatorTelemetry: actuatorTelemetry),
            time: time
        )
        try actuatorApply(actuatorValues, time)
        try plantIntegrate(time)
        let snapshot = plantSnapshot()
        return WorldStepLog(
            time: time,
            events: [
                .timeAdvance,
                .actuatorUpdate,
                .plantIntegrate,
                .sensorSample,
                .cutUpdate,
                .motorNerveUpdate,
                .applyCommands,
                .logging,
                .replayCheck
            ],
            sensorSamples: try rigidSensorSamples(entry: entry, snapshot: snapshot, timestamp: time.time),
            driveIntents: drives,
            reflexCorrections: [],
            actuatorValues: actuatorValues,
            actuatorTelemetry: updatedActuatorTelemetry(),
            motorNerveTrace: motorNerve.lastTrace,
            safetyTrace: try safetyTrace(),
            plantState: snapshot,
            disturbances: DisturbanceSnapshot(
                forceWorld: Axis3(x: 0.0, y: 0.0, z: 0.0),
                torqueBody: Axis3(x: 0.0, y: 0.0, z: 0.0)
            )
        )
    }

    func rigidDriveIntents(
        time: Double,
        stepCount: Int,
        driveCount: Int,
        hover: Double,
        amplitude: Double
    ) throws -> [DriveIntent] {
        try (0..<driveCount).map { index in
            let phase = Double(index) / Double(max(driveCount, 1))
            let period = max(Double(stepCount), 1.0)
            let value = hover + amplitude * sin((2.0 * Double.pi * time / period) + phase)
            return try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
        }
    }

    func rigidSensorSamples(
        entry: DescriptorCorpusEntry,
        snapshot: PlantStateSnapshot,
        timestamp: Double
    ) throws -> [ChannelSample] {
        try entry.embodiment.signals.sensor.sorted { $0.index < $1.index }.map { signal in
            let index = signal.index
            let value: Double
            switch index {
            case 0:
                value = snapshot.root.angularVelocity.x
            case 1:
                value = snapshot.root.angularVelocity.y
            case 2:
                value = snapshot.root.angularVelocity.z
            case 3:
                value = snapshot.root.velocity.x
            case 4:
                value = snapshot.root.velocity.y
            default:
                value = snapshot.root.velocity.z
            }
            return try ChannelSample(channelIndex: UInt32(index), value: value, timestamp: timestamp)
        }
    }

    func rigidReplayParameters(entry: DescriptorCorpusEntry) throws -> ReferenceQuadrotorParameters {
        let dynamicLinks = entry.body.links.filter { $0.mass > 0 }
        let mass = dynamicLinks.reduce(0.0) { $0 + $1.mass }
        guard mass > 0 else {
            throw KuyuModelValidationError.empty("rigidReplay.body.links.mass")
        }
        let inertia = dynamicLinks.reduce(Axis3(x: 0.0, y: 0.0, z: 0.0)) { partial, link in
            Axis3(
                x: partial.x + link.inertia.ixx,
                y: partial.y + link.inertia.iyy,
                z: partial.z + link.inertia.izz
            )
        }
        let maxThrust = entry.embodiment.actuators.map(\.limits.max).max() ?? ReferenceQuadrotorParameters.baseline.maxThrust
        let motorTimeConstant = entry.embodiment.actuators.compactMap(\.dynamics?.timeConstantSeconds).max()
            ?? ReferenceQuadrotorParameters.baseline.motorTimeConstant
        return try ReferenceQuadrotorParameters(
            mass: mass,
            inertia: inertia,
            armLength: ReferenceQuadrotorParameters.baseline.armLength,
            motorTimeConstant: motorTimeConstant,
            maxThrust: maxThrust,
            yawCoefficient: ReferenceQuadrotorParameters.baseline.yawCoefficient,
            gravity: worldGravityMagnitude(entry.world),
            aerodynamics: ReferenceQuadrotorParameters.baseline.aerodynamics
        )
    }

    func validateRigidReplayWorld(entry: DescriptorCorpusEntry) throws {
        guard entry.duration.isFinite, entry.duration > 0 else {
            throw DescriptorCorpusAcceptanceError.invalidDuration(entryID: entry.entryID, duration: entry.duration)
        }
        guard abs(entry.timeStep.delta - entry.world.time.fixedStepSeconds) <= 1e-12 else {
            throw ArticulatedRigidBodySimulator.SimulationError.timeStepMismatch(
                request: entry.timeStep.delta,
                world: entry.world.time.fixedStepSeconds
            )
        }
        guard entry.world.integrator.kind == .semiImplicitEuler else {
            throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld(
                "integrator.\(entry.world.integrator.kind.rawValue)"
            )
        }
        guard entry.world.contact.mode == .disabled, entry.world.solver.kind == .disabledContact else {
            throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld(
                "rigidActuator.contact.\(entry.world.contact.mode.rawValue)"
            )
        }
    }

    func exactStepCount(duration: Double, timeStep: Double) throws -> Int {
        let rawStepCount = duration / timeStep
        let roundedStepCount = rawStepCount.rounded()
        guard abs(rawStepCount - roundedStepCount) <= 1e-9 else {
            throw ArticulatedRigidBodySimulator.SimulationError.durationStepMismatch(
                duration: duration,
                timeStep: timeStep
            )
        }
        let stepCount = Int(roundedStepCount)
        guard stepCount > 0 else {
            throw DescriptorCorpusAcceptanceError.invalidDuration(entryID: "rigid-actuator", duration: duration)
        }
        return stepCount
    }

    func worldEnvironment(world: KuyuWorldModel) throws -> WorldEnvironment {
        let temperature = world.atmosphere.temperatureKelvin ?? WorldEnvironment.standard.airTemperature
        let density = world.atmosphere.airDensity ?? WorldEnvironment.seaLevelDensity
        let pressure = density * WorldEnvironment.dryAirGasConstant * temperature
        return try WorldEnvironment(
            gravity: worldGravityMagnitude(world),
            windVelocityWorld: Axis3(
                x: world.wind.velocityWorld.x,
                y: world.wind.velocityWorld.y,
                z: world.wind.velocityWorld.z
            ),
            airPressure: pressure,
            airTemperature: temperature,
            usage: .full
        )
    }

    func worldGravityMagnitude(_ world: KuyuWorldModel) -> Double {
        let vector = SIMD3<Double>(
            world.gravity.acceleration.x,
            world.gravity.acceleration.y,
            world.gravity.acceleration.z
        )
        return max(simd_length(vector), 1e-9)
    }

    func rigidReplayConfigHash(
        entry: DescriptorCorpusEntry,
        parameters: ReferenceQuadrotorParameters
    ) throws -> String {
        try ConfigHash.hash(RigidActuatorReplayConfigEnvelope(
            schemaVersion: "kuyu.rigid-actuator.simulation-config.v1",
            simulatorVersion: "rigid-actuator-dynamic-v1",
            body: entry.body,
            world: entry.world,
            embodiment: entry.embodiment,
            compatibilityReport: entry.compatibilityReport,
            determinism: entry.determinism,
            readinessLevel: entry.requiredReadiness,
            duration: entry.duration,
            timeStep: entry.timeStep,
            seed: entry.seed,
            parameters: parameters
        ))
    }
}

private struct RigidActuatorReplayConfigEnvelope: Encodable {
    let schemaVersion: String
    let simulatorVersion: String
    let body: KuyuBodyModel
    let world: KuyuWorldModel
    let embodiment: EmbodimentContract
    let compatibilityReport: CompatibilityReport?
    let determinism: DeterminismConfig
    let readinessLevel: ReadinessLevel
    let duration: Double
    let timeStep: TimeStep
    let seed: ScenarioSeed
    let parameters: ReferenceQuadrotorParameters
}
