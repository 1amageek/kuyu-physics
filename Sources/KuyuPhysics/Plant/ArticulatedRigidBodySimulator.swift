import EmbodimentContract
import Foundation
import KuyuCore
import simd

public struct ArticulatedRigidBodySimulationRequest: Sendable, Equatable {
    public let body: KuyuBodyModel
    public let world: KuyuWorldModel
    public let embodiment: EmbodimentContract
    public let compatibilityReport: CompatibilityReport?
    public let determinism: DeterminismConfig
    public let readinessLevel: ReadinessLevel
    public let duration: Double
    public let timeStep: TimeStep
    public let seed: ScenarioSeed

    public init(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        embodiment: EmbodimentContract,
        compatibilityReport: CompatibilityReport? = nil,
        determinism: DeterminismConfig,
        readinessLevel: ReadinessLevel = .dynamicSimulation,
        duration: Double = 6.0,
        timeStep: TimeStep,
        seed: ScenarioSeed = ScenarioSeed(0)
    ) {
        self.body = body
        self.world = world
        self.embodiment = embodiment
        self.compatibilityReport = compatibilityReport
        self.determinism = determinism
        self.readinessLevel = readinessLevel
        self.duration = duration
        self.timeStep = timeStep
        self.seed = seed
    }
}

public struct ArticulatedRigidBodySimulator: Sendable {
    public enum SimulationError: Error, Equatable {
        case invalidDuration(Double)
        case durationStepMismatch(duration: Double, timeStep: Double)
        case timeStepMismatch(request: Double, world: Double)
        case invalidBody(String)
        case invalidDriveProviderOutput(String)
        case missingJointRange(String)
        case nonFiniteState(String)
        case readinessFailed(String)
        case unstableTimeStep(substep: Double, timeConstant: Double, actuator: String)
        case unresolvedContact(maxPenetration: Double, tolerance: Double)
        case unsupportedWorld(String)
    }

    public init() {}

    public func run(
        request: ArticulatedRigidBodySimulationRequest,
        control: SimulationControl? = nil,
        telemetry: WorldStepTelemetry? = nil
    ) async throws -> SimulationLog {
        return try await run(
            request: request,
            driveProvider: ArticulatedSineDriveProvider(),
            control: control,
            telemetry: telemetry
        )
    }

    public func run<DriveProvider: ArticulatedRigidBodyDriveProvider>(
        request: ArticulatedRigidBodySimulationRequest,
        driveProvider: DriveProvider,
        control: SimulationControl? = nil,
        telemetry: WorldStepTelemetry? = nil
    ) async throws -> SimulationLog {
        guard request.duration.isFinite, request.duration > 0 else {
            throw SimulationError.invalidDuration(request.duration)
        }
        let stepCount = try exactStepCount(duration: request.duration, timeStep: request.timeStep.delta)
        guard abs(request.timeStep.delta - request.world.time.fixedStepSeconds) <= 1e-12 else {
            throw SimulationError.timeStepMismatch(
                request: request.timeStep.delta,
                world: request.world.time.fixedStepSeconds
            )
        }
        try validateSupportedWorld(request.world)
        do {
            _ = try ReadinessGate().validate(
                body: request.body,
                world: request.world,
                embodiment: request.embodiment,
                report: request.compatibilityReport,
                requiredLevel: request.readinessLevel
            )
        } catch {
            throw SimulationError.readinessFailed(String(describing: error))
        }

        let movableJoints = request.body.joints.filter { joint in
            joint.mimic == nil && (joint.kind == .revolute || joint.kind == .continuous || joint.kind == .prismatic)
        }
        guard !movableJoints.isEmpty else {
            throw SimulationError.invalidBody("movable-joints")
        }
        let driveSignals = try orderedDriveSignals(from: request.embodiment)
        let actuatorSignals = request.embodiment.signals.actuator.sorted { $0.index < $1.index }
        guard driveSignals.count == movableJoints.count else {
            throw SimulationError.invalidBody("drive-joint-count")
        }

        var motorNerve = try MotorNerveChain(contract: request.embodiment)
        let jointBindings = try bindings(
            joints: movableJoints,
            body: request.body,
            embodiment: request.embodiment,
            actuatorSignals: actuatorSignals
        )
        let jointRanges = try ranges(bindings: jointBindings)
        try validateNumericalStability(
            bindings: jointBindings,
            substepDelta: request.timeStep.delta / Double(max(request.world.time.substeps, 1))
        )
        let jointIDs = jointBindings.map(\.joint.id)
        let driveSignalIDs = driveSignals.map(\.id)
        let actuatorSignalIDs = actuatorSignals.map(\.id)
        var provider = driveProvider
        let initialPositions = try initialPositions(bindings: jointBindings, ranges: jointRanges)
        try provider.reset(context: ArticulatedRigidBodyDriveProviderResetContext(
            seed: request.seed,
            jointIDs: jointIDs,
            driveSignalIDs: driveSignalIDs,
            jointRanges: jointRanges
        ))
        let stateModel = ArticulatedStateModel(body: request.body, world: request.world)
        let jointDynamics = try stateModel.dynamics(for: jointBindings)
        let snapshotTopology = try snapshotTopology(body: request.body)
        let contactSolver = try makeContactSolver(
            body: request.body,
            world: request.world,
            bindings: jointBindings,
            dynamics: jointDynamics,
            topology: snapshotTopology
        )
        try validateContactNumericalStability(
            world: request.world,
            dynamics: jointDynamics,
            substepDelta: request.timeStep.delta / Double(max(request.world.time.substeps, 1))
        )
        var state = ArticulatedState(
            position: initialPositions,
            velocity: Array(repeating: 0, count: jointBindings.count),
            torque: Array(repeating: 0, count: jointBindings.count)
        )
        var contactMetrics = ArticulatedContactMetrics.disabled
        var targets = initialPositions
        var logs: [WorldStepLog] = []
        logs.reserveCapacity(stepCount)

        for step in 0..<stepCount {
            if let control = control {
                try await control.checkpoint()
            }

            let time = try WorldTime(stepIndex: UInt64(step), time: Double(step) * request.timeStep.delta)
            let providerContext = ArticulatedRigidBodyDriveContext(
                time: time,
                jointIDs: jointIDs,
                driveSignalIDs: driveSignalIDs,
                actuatorSignalIDs: actuatorSignalIDs,
                jointRanges: jointRanges,
                positions: state.position,
                velocities: state.velocity,
                targets: targets,
                torques: state.torque
            )
            let drives = try provider.driveIntents(context: providerContext)
            try validateProviderDrives(drives, expectedCount: jointBindings.count)
            let actuatorValues = try motorNerve.update(
                input: drives,
                corrections: [],
                telemetry: MotorNerveTelemetry(actuatorTelemetry: try actuatorTelemetry(
                    state: state,
                    bindings: jointBindings,
                    actuatorSignals: actuatorSignals
                )),
                time: time
            )
            targets = try jointTargets(values: actuatorValues, bindings: jointBindings, actuatorSignals: actuatorSignals)
            let stepResult = try stateModel.step(
                state: state,
                targets: targets,
                dynamics: jointDynamics,
                deltaTime: request.timeStep.delta,
                contactSolver: contactSolver
            )
            state = stepResult.state
            contactMetrics = stepResult.contactMetrics

            let log = try makeStepLog(
                body: request.body,
                world: request.world,
                embodiment: request.embodiment,
                time: time,
                snapshotTopology: snapshotTopology,
                bindings: jointBindings,
                state: state,
                targets: targets,
                contactMetrics: contactMetrics,
                drives: drives,
                actuatorValues: actuatorValues,
                actuatorSignals: actuatorSignals,
                motorNerveTrace: motorNerve.lastTrace
            )
            telemetry?(log)
            logs.append(log)

            if (step % 20) == 0 {
                await Task.yield()
            }
        }

        return SimulationLog(
            scenarioId: try ScenarioID("\(request.body.bodyID.uppercased())-DYN-1"),
            seed: request.seed,
            timeStep: request.timeStep,
            determinism: request.determinism,
            configHash: try configHash(request: request, providerID: provider.providerID),
            events: logs
        )
    }

    private func configHash(
        request: ArticulatedRigidBodySimulationRequest,
        providerID: String
    ) throws -> String {
        try ConfigHash.hash(ArticulatedSimulationConfigEnvelope(
            schemaVersion: "kuyu.articulated.simulation-config.v1",
            simulatorVersion: "articulated-dynamic-v3",
            body: request.body,
            world: request.world,
            embodiment: request.embodiment,
            compatibilityReport: request.compatibilityReport,
            determinism: request.determinism,
            readinessLevel: request.readinessLevel,
            duration: request.duration,
            timeStep: request.timeStep,
            seed: request.seed,
            driveProviderID: providerID
        ))
    }

    private func orderedDriveSignals(from contract: EmbodimentContract) throws -> [SignalDefinition] {
        let byID = Dictionary(uniqueKeysWithValues: contract.signals.drive.map { ($0.id, $0) })
        return try contract.control.driveChannels.map { id in
            guard let signal = byID[id] else {
                throw SimulationError.invalidBody("driveChannels.\(id)")
            }
            return signal
        }
    }

    private func exactStepCount(duration: Double, timeStep: Double) throws -> Int {
        let rawStepCount = duration / timeStep
        let roundedStepCount = rawStepCount.rounded()
        guard abs(rawStepCount - roundedStepCount) <= 1e-9 else {
            throw SimulationError.durationStepMismatch(duration: duration, timeStep: timeStep)
        }
        let stepCount = Int(roundedStepCount)
        guard stepCount > 0 else {
            throw SimulationError.invalidDuration(duration)
        }
        return stepCount
    }

    private func validateSupportedWorld(_ world: KuyuWorldModel) throws {
        guard world.integrator.kind == .semiImplicitEuler else {
            throw SimulationError.unsupportedWorld("integrator.\(world.integrator.kind.rawValue)")
        }
        switch world.contact.mode {
        case .disabled:
            guard world.solver.kind == .disabledContact else {
                throw SimulationError.unsupportedWorld(
                    "contact.disabled.solver.\(world.solver.kind.rawValue)"
                )
            }
        case .penalty, .constraint:
            guard world.solver.kind == .deterministicConstraint else {
                throw SimulationError.unsupportedWorld(
                    "contact.\(world.contact.mode.rawValue).solver.\(world.solver.kind.rawValue)"
                )
            }
        }
    }

    private func makeContactSolver(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        bindings: [ArticulatedJointBinding],
        dynamics: [ArticulatedJointDynamics],
        topology: ArticulatedSnapshotTopology
    ) throws -> ArticulatedContactSolver? {
        guard world.contact.mode != .disabled else { return nil }
        return try ArticulatedContactSolver(
            body: body,
            world: world,
            bindings: bindings,
            dynamics: dynamics,
            topology: topology
        )
    }

    private func validateContactNumericalStability(
        world: KuyuWorldModel,
        dynamics: [ArticulatedJointDynamics],
        substepDelta: Double
    ) throws {
        guard world.contact.mode == .penalty else { return }
        guard let stiffness = world.contact.stiffness else {
            throw SimulationError.unsupportedWorld("contact.penalty.stiffness")
        }
        let minimumInertia = dynamics.map(\.effectiveInertia).min() ?? 1e-6
        let contactTimeConstant = sqrt(max(minimumInertia, 1e-9) / stiffness)
        guard substepDelta <= contactTimeConstant * 0.25 + 1e-12 else {
            throw SimulationError.unstableTimeStep(
                substep: substepDelta,
                timeConstant: contactTimeConstant * 0.25,
                actuator: "contact.penalty"
            )
        }
    }

    private func bindings(
        joints: [JointDefinition],
        body: KuyuBodyModel,
        embodiment: EmbodimentContract,
        actuatorSignals: [SignalDefinition]
    ) throws -> [ArticulatedJointBinding] {
        let jointsByID = Dictionary(uniqueKeysWithValues: joints.map { ($0.id, $0) })
        let attachmentsByActuator = Dictionary(uniqueKeysWithValues: body.actuatorAttachments.map { ($0.actuatorID, $0) })
        let actuatorsByID = Dictionary(uniqueKeysWithValues: embodiment.actuators.map { ($0.id, $0) })
        let actuatorsBySignalID = try actuatorMapBySignalID(embodiment.actuators)

        let bindings = try actuatorSignals.map { signal in
            guard let actuatorID = actuatorsBySignalID[signal.id],
                  let actuator = actuatorsByID[actuatorID] else {
                throw SimulationError.invalidBody("actuator.signal.\(signal.id)")
            }
            guard actuator.channels.count == 1 else {
                throw SimulationError.invalidBody("actuator.channels.\(actuator.id)")
            }
            guard let attachment = attachmentsByActuator[actuator.id] else {
                throw SimulationError.invalidBody("attachment.\(actuator.id)")
            }
            guard let joint = jointsByID[attachment.jointID] else {
                throw SimulationError.invalidBody("attachment.joint.\(attachment.jointID)")
            }
            return ArticulatedJointBinding(
                joint: joint,
                attachment: attachment,
                actuator: actuator,
                actuatorSignal: signal
            )
        }
        let boundJointIDs = Set(bindings.map(\.joint.id))
        let movableJointIDs = Set(joints.map(\.id))
        guard boundJointIDs == movableJointIDs else {
            throw SimulationError.invalidBody("attachment.coverage")
        }
        return bindings
    }

    private func actuatorMapBySignalID(_ actuators: [ActuatorDefinition]) throws -> [String: String] {
        var actuatorBySignalID: [String: String] = [:]
        for actuator in actuators {
            for channelID in actuator.channels {
                if actuatorBySignalID[channelID] != nil {
                    throw SimulationError.invalidBody("actuator.signal.duplicate.\(channelID)")
                }
                actuatorBySignalID[channelID] = actuator.id
            }
        }
        return actuatorBySignalID
    }

    private func jointTargets(
        values: [ActuatorValue],
        bindings: [ArticulatedJointBinding],
        actuatorSignals: [SignalDefinition]
    ) throws -> [Double] {
        guard values.count == bindings.count, actuatorSignals.count == bindings.count else {
            throw SimulationError.invalidBody("actuator.value.count")
        }
        var targets = Array(repeating: 0.0, count: bindings.count)
        var seen = Array(repeating: false, count: bindings.count)
        for value in values {
            let index = Int(value.index.rawValue)
            guard index >= 0, index < bindings.count else {
                throw SimulationError.invalidBody("actuator.index.\(value.index.rawValue)")
            }
            guard !seen[index] else {
                throw SimulationError.invalidBody("actuator.index.duplicate.\(value.index.rawValue)")
            }
            guard bindings[index].actuatorSignal.id == actuatorSignals[index].id else {
                throw SimulationError.invalidBody("actuator.binding.order.\(value.index.rawValue)")
            }
            guard value.value.isFinite else {
                throw SimulationError.nonFiniteState("actuator.value[\(index)]")
            }
            let target = ArticulatedActuatorMapping.jointPosition(
                actuatorPosition: value.value,
                attachment: bindings[index].attachment
            )
            guard target.isFinite else {
                throw SimulationError.nonFiniteState("joint.target[\(index)]")
            }
            targets[index] = target
            seen[index] = true
        }
        guard seen.allSatisfy({ $0 }) else {
            throw SimulationError.invalidBody("actuator.value.coverage")
        }
        return targets
    }

    private func ranges(bindings: [ArticulatedJointBinding]) throws -> [ClosedRange<Double>] {
        try bindings.map { binding in
            let actuatorRange = ArticulatedActuatorMapping.jointRange(
                actuatorLimits: binding.actuator.limits,
                attachment: binding.attachment
            )
            let lower = max(
                actuatorRange.lowerBound,
                binding.joint.softLowerLimit ?? binding.joint.lowerLimit ?? actuatorRange.lowerBound
            )
            let upper = min(
                actuatorRange.upperBound,
                binding.joint.softUpperLimit ?? binding.joint.upperLimit ?? actuatorRange.upperBound
            )
            if lower > upper {
                guard lower - upper <= 1e-9 else {
                    throw SimulationError.missingJointRange(binding.actuatorSignal.id)
                }
                let collapsed = (lower + upper) * 0.5
                return collapsed...collapsed
            }
            guard lower.isFinite, upper.isFinite else {
                throw SimulationError.missingJointRange(binding.actuatorSignal.id)
            }
            return lower...upper
        }
    }

    private func validateNumericalStability(
        bindings: [ArticulatedJointBinding],
        substepDelta: Double
    ) throws {
        guard substepDelta.isFinite, substepDelta > 0 else {
            throw SimulationError.nonFiniteState("substepDelta")
        }
        for binding in bindings {
            let timeConstant = binding.actuator.dynamics?.timeConstantSeconds ?? 0.001
            guard substepDelta <= timeConstant + 1e-12 else {
                throw SimulationError.unstableTimeStep(
                    substep: substepDelta,
                    timeConstant: timeConstant,
                    actuator: binding.actuator.id
                )
            }
        }
    }

    private func initialPositions(
        bindings: [ArticulatedJointBinding],
        ranges: [ClosedRange<Double>]
    ) throws -> [Double] {
        guard bindings.count == ranges.count else {
            throw SimulationError.invalidBody("joint-range-count")
        }
        return try bindings.enumerated().map { index, binding in
            let range = ranges[index]
            let candidate = binding.joint.homePosition ?? 0.0
            guard candidate.isFinite else {
                throw SimulationError.nonFiniteState("homePosition.\(binding.joint.id)")
            }
            return min(max(candidate, range.lowerBound), range.upperBound)
        }
    }

    private func validateProviderDrives(_ drives: [DriveIntent], expectedCount: Int) throws {
        guard drives.count == expectedCount else {
            throw SimulationError.invalidDriveProviderOutput(
                "drive-count expected=\(expectedCount) actual=\(drives.count)"
            )
        }
        for (index, drive) in drives.enumerated() {
            guard drive.index.rawValue == UInt32(index) else {
                throw SimulationError.invalidDriveProviderOutput(
                    "drive-index expected=\(index) actual=\(drive.index.rawValue)"
                )
            }
            guard drive.activation.isFinite else {
                throw SimulationError.nonFiniteState("drive[\(index)]")
            }
        }
    }

    private func makeStepLog(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        embodiment: EmbodimentContract,
        time: WorldTime,
        snapshotTopology: ArticulatedSnapshotTopology,
        bindings: [ArticulatedJointBinding],
        state: ArticulatedState,
        targets: [Double],
        contactMetrics: ArticulatedContactMetrics,
        drives: [DriveIntent],
        actuatorValues: [ActuatorValue],
        actuatorSignals: [SignalDefinition],
        motorNerveTrace: MotorNerveTrace?
    ) throws -> WorldStepLog {
        let root = RigidBodySnapshot(
            id: "\(body.bodyID)-base",
            position: Axis3(x: 0.0, y: 0.0, z: 0.0),
            velocity: Axis3(x: 0.0, y: 0.0, z: 0.0),
            orientation: QuaternionSnapshot(w: 1.0, x: 0.0, y: 0.0, z: 0.0),
            angularVelocity: Axis3(x: 0.0, y: 0.0, z: 0.0)
        )
        let scalars = try scalarState(
            body: body,
            bindings: bindings,
            state: state,
            targets: targets,
            contactMetrics: contactMetrics
        )
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
            sensorSamples: try jointSamples(values: state.position, timestamp: time.time),
            driveIntents: drives,
            reflexCorrections: [],
            actuatorValues: actuatorValues,
            actuatorTelemetry: try actuatorTelemetry(state: state, bindings: bindings, actuatorSignals: actuatorSignals),
            motorNerveTrace: motorNerveTrace,
            safetyTrace: try SafetyTrace(omegaMagnitude: state.velocity.map(abs).max() ?? 0.0, tiltRadians: 0.0),
            plantState: PlantStateSnapshot(
                root: root,
                bodies: try linkSnapshots(topology: snapshotTopology, scalars: scalars),
                scalars: scalars
            ),
            disturbances: DisturbanceSnapshot(
                forceWorld: Axis3(x: 0.0, y: 0.0, z: 0.0),
                torqueBody: Axis3(
                    x: world.gravity.acceleration.x,
                    y: world.gravity.acceleration.y,
                    z: world.gravity.acceleration.z
                )
            )
        )
    }

    private func jointSamples(values: [Double], timestamp: Double) throws -> [ChannelSample] {
        try values.enumerated().map { index, value in
            try ChannelSample(channelIndex: UInt32(index), value: value, timestamp: timestamp)
        }
    }

    private func actuatorTelemetry(
        state: ArticulatedState,
        bindings: [ArticulatedJointBinding],
        actuatorSignals: [SignalDefinition]
    ) throws -> ActuatorTelemetrySnapshot {
        guard actuatorSignals.count == bindings.count, state.position.count == bindings.count else {
            throw SimulationError.invalidBody("actuator.telemetry.count")
        }
        let channels = try actuatorSignals.enumerated().map { index, signal in
            guard bindings[index].actuatorSignal.id == signal.id else {
                throw SimulationError.invalidBody("actuator.telemetry.order.\(signal.id)")
            }
            let actuatorPosition = ArticulatedActuatorMapping.actuatorPosition(
                jointPosition: state.position[index],
                attachment: bindings[index].attachment
            )
            guard actuatorPosition.isFinite else {
                throw SimulationError.nonFiniteState("actuator.telemetry[\(index)]")
            }
            return ActuatorChannelSnapshot(id: signal.id, value: actuatorPosition, units: signal.units)
        }
        return ActuatorTelemetrySnapshot(channels: channels)
    }

    private func scalarState(
        body: KuyuBodyModel,
        bindings: [ArticulatedJointBinding],
        state: ArticulatedState,
        targets: [Double],
        contactMetrics: ArticulatedContactMetrics
    ) throws -> [String: Double] {
        var scalars: [String: Double] = [:]
        var positionsByJointID: [String: Double] = [:]
        var velocitiesByJointID: [String: Double] = [:]
        var targetsByJointID: [String: Double] = [:]
        var torquesByJointID: [String: Double] = [:]
        for index in state.position.indices {
            let binding = bindings[index]
            let signalID = binding.actuatorSignal.id
            let target = targets.indices.contains(index) ? targets[index] : 0
            let actuatorPosition = ArticulatedActuatorMapping.actuatorPosition(
                jointPosition: state.position[index],
                attachment: binding.attachment
            )
            let actuatorTarget = ArticulatedActuatorMapping.actuatorPosition(
                jointPosition: target,
                attachment: binding.attachment
            )
            let actuatorVelocity = ArticulatedActuatorMapping.actuatorVelocity(
                jointVelocity: state.velocity[index],
                attachment: binding.attachment
            )
            let actuatorTorque = ArticulatedActuatorMapping.actuatorTorque(
                jointTorque: state.torque[index],
                attachment: binding.attachment
            )
            guard actuatorPosition.isFinite,
                  actuatorTarget.isFinite,
                  actuatorVelocity.isFinite,
                  actuatorTorque.isFinite else {
                throw SimulationError.nonFiniteState("actuator.scalar[\(index)]")
            }
            positionsByJointID[binding.joint.id] = state.position[index]
            velocitiesByJointID[binding.joint.id] = state.velocity[index]
            targetsByJointID[binding.joint.id] = target
            torquesByJointID[binding.joint.id] = state.torque[index]
            scalars[signalID] = actuatorPosition
            scalars[binding.joint.id] = state.position[index]
            scalars["target_\(signalID)"] = actuatorTarget
            scalars["target_\(binding.joint.id)"] = target
            scalars["velocity_\(signalID)"] = actuatorVelocity
            scalars["velocity_\(binding.joint.id)"] = state.velocity[index]
            scalars["torque_\(signalID)"] = actuatorTorque
            scalars["torque_\(binding.joint.id)"] = state.torque[index]
        }

        var unresolvedMimics = body.joints.filter { $0.mimic != nil }
        while !unresolvedMimics.isEmpty {
            var remaining: [JointDefinition] = []
            var resolvedCount = 0

            for joint in unresolvedMimics {
                guard let mimic = joint.mimic,
                      let masterPosition = positionsByJointID[mimic.jointID] else {
                    remaining.append(joint)
                    continue
                }

                let masterVelocity = velocitiesByJointID[mimic.jointID] ?? 0
                let masterTarget = targetsByJointID[mimic.jointID] ?? masterPosition
                let masterTorque = torquesByJointID[mimic.jointID] ?? 0
                let position = masterPosition * mimic.multiplier + mimic.offset
                let velocity = masterVelocity * mimic.multiplier
                let target = masterTarget * mimic.multiplier + mimic.offset
                let torque = masterTorque * mimic.multiplier

                positionsByJointID[joint.id] = position
                velocitiesByJointID[joint.id] = velocity
                targetsByJointID[joint.id] = target
                torquesByJointID[joint.id] = torque
                scalars[joint.id] = position
                scalars["target_\(joint.id)"] = target
                scalars["velocity_\(joint.id)"] = velocity
                scalars["torque_\(joint.id)"] = torque
                resolvedCount += 1
            }

            if resolvedCount == 0 {
                let ids = remaining.map(\.id).joined(separator: ",")
                throw SimulationError.invalidBody("mimic.\(ids)")
            }
            unresolvedMimics = remaining
        }
        scalars["contact.active.count"] = Double(contactMetrics.activeContacts)
        scalars["contact.penetration.max"] = contactMetrics.maxPenetration
        scalars["contact.normalImpulse.max"] = contactMetrics.maxNormalImpulse
        scalars["contact.normalForce.max"] = contactMetrics.maxNormalForce
        scalars["contact.solver.iterations"] = Double(contactMetrics.solverIterations)
        return scalars
    }

    private func snapshotTopology(body: KuyuBodyModel) throws -> ArticulatedSnapshotTopology {
        let childLinkIDs = Set(body.joints.map(\.childLinkID))
        let rootLinks = body.links.filter { !childLinkIDs.contains($0.id) }
        guard !rootLinks.isEmpty else {
            throw SimulationError.invalidBody("root-link")
        }
        var jointsByParentLinkID: [String: [JointDefinition]] = [:]
        for joint in body.joints {
            jointsByParentLinkID[joint.parentLinkID, default: []].append(joint)
        }
        var orderedJoints: [JointDefinition] = []
        orderedJoints.reserveCapacity(body.joints.count)
        var visitedJointIDs: Set<String> = []
        var linkStack = rootLinks.map(\.id)

        while !linkStack.isEmpty {
            let parentLinkID = linkStack.removeFirst()
            for joint in jointsByParentLinkID[parentLinkID] ?? [] {
                guard visitedJointIDs.insert(joint.id).inserted else {
                    throw SimulationError.invalidBody("joint-topology.duplicate.\(joint.id)")
                }
                orderedJoints.append(joint)
                linkStack.append(joint.childLinkID)
            }
        }
        guard orderedJoints.count == body.joints.count else {
            let resolvedIDs = Set(orderedJoints.map(\.id))
            let unresolvedIDs = body.joints
                .filter { !resolvedIDs.contains($0.id) }
                .map(\.id)
                .joined(separator: ",")
            throw SimulationError.invalidBody("joint-topology.\(unresolvedIDs)")
        }
        return ArticulatedSnapshotTopology(rootLinks: rootLinks, orderedJoints: orderedJoints)
    }

    private func linkSnapshots(
        topology: ArticulatedSnapshotTopology,
        scalars: [String: Double]
    ) throws -> [RigidBodySnapshot] {
        try ArticulatedKinematics(topology: topology).snapshots(scalars: scalars)
    }

}

private struct ArticulatedSimulationConfigEnvelope: Sendable, Encodable, Equatable {
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
    let driveProviderID: String
}
