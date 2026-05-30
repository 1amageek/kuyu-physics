import EmbodimentContract
import Foundation
import KuyuCore

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
        case invalidBody(String)
        case missingJointRange(String)
        case nonFiniteState(String)
        case readinessFailed(String)
    }

    public init() {}

    public func run(
        request: ArticulatedRigidBodySimulationRequest,
        control: SimulationControl? = nil,
        telemetry: WorldStepTelemetry? = nil
    ) async throws -> SimulationLog {
        guard request.duration.isFinite, request.duration > 0 else {
            throw SimulationError.invalidDuration(request.duration)
        }
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
        guard driveSignals.count == movableJoints.count else {
            throw SimulationError.invalidBody("drive-joint-count")
        }

        var motorNerve = try MotorNerveChain(contract: request.embodiment)
        let actuatorSignals = request.embodiment.signals.actuator.sorted { $0.index < $1.index }
        let jointBindings = try bindings(
            joints: movableJoints,
            body: request.body,
            embodiment: request.embodiment,
            actuatorSignals: actuatorSignals
        )
        let jointRanges = try ranges(from: driveSignals, bindings: jointBindings)
        let stateModel = ArticulatedStateModel(body: request.body, world: request.world)
        var state = ArticulatedState(
            position: Array(repeating: 0, count: jointBindings.count),
            velocity: Array(repeating: 0, count: jointBindings.count),
            torque: Array(repeating: 0, count: jointBindings.count)
        )
        let stepCount = Int((request.duration / request.timeStep.delta).rounded(.down))
        var logs: [WorldStepLog] = []
        logs.reserveCapacity(stepCount)

        for step in 0..<stepCount {
            if let control {
                try await control.checkpoint()
            }

            let time = try WorldTime(stepIndex: UInt64(step), time: Double(step) * request.timeStep.delta)
            let drives = try trajectoryTargets(
                time: time.time,
                ranges: jointRanges
            ).enumerated().map { index, value in
                try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
            }
            let actuatorValues = try motorNerve.update(
                input: drives,
                corrections: [],
                telemetry: MotorNerveTelemetry(actuatorTelemetry: actuatorTelemetry(
                    state: state,
                    bindings: jointBindings,
                    actuatorSignals: actuatorSignals
                )),
                time: time
            )
            let targets = try jointTargets(values: actuatorValues, bindings: jointBindings, actuatorSignals: actuatorSignals)
            state = try stateModel.step(
                state: state,
                targets: targets,
                bindings: jointBindings,
                deltaTime: request.timeStep.delta
            )

            let log = try makeStepLog(
                body: request.body,
                world: request.world,
                embodiment: request.embodiment,
                time: time,
                bindings: jointBindings,
                state: state,
                targets: targets,
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
            configHash: "articulated-dynamic-v1-\(request.body.bodyID)-duration-\(request.duration)-dt-\(request.timeStep.delta)",
            events: logs
        )
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

    private func bindings(
        joints: [JointDefinition],
        body: KuyuBodyModel,
        embodiment: EmbodimentContract,
        actuatorSignals: [SignalDefinition]
    ) throws -> [ArticulatedJointBinding] {
        let attachmentsByJoint = Dictionary(uniqueKeysWithValues: body.actuatorAttachments.map { ($0.jointID, $0) })
        let actuatorsByID = Dictionary(uniqueKeysWithValues: embodiment.actuators.map { ($0.id, $0) })
        let actuatorSignalsByID = Dictionary(uniqueKeysWithValues: actuatorSignals.map { ($0.id, $0) })

        return try joints.map { joint in
            guard let attachment = attachmentsByJoint[joint.id] else {
                throw SimulationError.invalidBody("attachment.\(joint.id)")
            }
            guard let actuator = actuatorsByID[attachment.actuatorID] else {
                throw SimulationError.invalidBody("actuator.\(attachment.actuatorID)")
            }
            guard actuator.channels.count == 1, let channelID = actuator.channels.first else {
                throw SimulationError.invalidBody("actuator.channels.\(actuator.id)")
            }
            guard let signal = actuatorSignalsByID[channelID] else {
                throw SimulationError.invalidBody("actuator.signal.\(channelID)")
            }
            return ArticulatedJointBinding(
                joint: joint,
                attachment: attachment,
                actuator: actuator,
                actuatorSignal: signal
            )
        }
    }

    private func jointTargets(
        values: [ActuatorValue],
        bindings: [ArticulatedJointBinding],
        actuatorSignals: [SignalDefinition]
    ) throws -> [Double] {
        let signalsByIndex = Dictionary(uniqueKeysWithValues: actuatorSignals.map { (UInt32($0.index), $0.id) })
        var valuesBySignalID: [String: Double] = [:]
        for value in values {
            guard let signalID = signalsByIndex[value.index.rawValue] else {
                throw SimulationError.invalidBody("actuator.index.\(value.index.rawValue)")
            }
            valuesBySignalID[signalID] = value.value
        }
        return try bindings.map { binding in
            guard let value = valuesBySignalID[binding.actuatorSignal.id] else {
                throw SimulationError.invalidBody("actuator.value.\(binding.actuatorSignal.id)")
            }
            return ((value - binding.attachment.actuatorZeroOffset) / binding.attachment.transmissionRatio)
                + binding.attachment.jointZeroOffset
        }
    }

    private func ranges(
        from signals: [SignalDefinition],
        bindings: [ArticulatedJointBinding]
    ) throws -> [ClosedRange<Double>] {
        try signals.enumerated().map { index, signal in
            guard let range = signal.range else {
                throw SimulationError.missingJointRange(signal.id)
            }
            guard bindings.indices.contains(index) else {
                return range.min...range.max
            }
            let joint = bindings[index].joint
            let lower = max(range.min, joint.softLowerLimit ?? joint.lowerLimit ?? range.min)
            let upper = min(range.max, joint.softUpperLimit ?? joint.upperLimit ?? range.max)
            guard lower <= upper else {
                throw SimulationError.missingJointRange(signal.id)
            }
            return lower...upper
        }
    }

    private func trajectoryTargets(
        time: Double,
        ranges: [ClosedRange<Double>]
    ) -> [Double] {
        let frequency = 0.20
        return ranges.enumerated().map { index, range in
            let center = (range.lowerBound + range.upperBound) * 0.5
            let amplitude = (range.upperBound - range.lowerBound) * 0.5 * 0.70
            let phase = Double(index) * 0.7
            return center + amplitude * sin((2.0 * Double.pi * frequency * time) + phase)
        }
    }

    private func makeStepLog(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        embodiment: EmbodimentContract,
        time: WorldTime,
        bindings: [ArticulatedJointBinding],
        state: ArticulatedState,
        targets: [Double],
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
        let scalars = scalarState(
            bindings: bindings,
            state: state,
            targets: targets
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
            actuatorTelemetry: actuatorTelemetry(state: state, bindings: bindings, actuatorSignals: actuatorSignals),
            motorNerveTrace: motorNerveTrace,
            safetyTrace: try SafetyTrace(omegaMagnitude: state.velocity.map(abs).max() ?? 0.0, tiltRadians: 0.0),
            plantState: PlantStateSnapshot(root: root, bodies: linkSnapshots(bindings: bindings, state: state), scalars: scalars),
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
    ) -> ActuatorTelemetrySnapshot {
        var valuesBySignalID: [String: Double] = [:]
        for index in bindings.indices {
            valuesBySignalID[bindings[index].actuatorSignal.id] = state.position[index]
        }
        return ActuatorTelemetrySnapshot(
            channels: actuatorSignals.map { signal in
                let value = valuesBySignalID[signal.id] ?? 0.0
                return ActuatorChannelSnapshot(id: signal.id, value: value, units: signal.units)
            }
        )
    }

    private func scalarState(
        bindings: [ArticulatedJointBinding],
        state: ArticulatedState,
        targets: [Double]
    ) -> [String: Double] {
        var scalars: [String: Double] = [:]
        for index in state.position.indices {
            let binding = bindings[index]
            let signalID = binding.actuatorSignal.id
            scalars[signalID] = state.position[index]
            scalars[binding.joint.id] = state.position[index]
            scalars["target_\(signalID)"] = targets.indices.contains(index) ? targets[index] : 0
            scalars["velocity_\(signalID)"] = state.velocity[index]
            scalars["torque_\(signalID)"] = state.torque[index]
        }
        return scalars
    }

    private func linkSnapshots(
        bindings: [ArticulatedJointBinding],
        state: ArticulatedState
    ) -> [RigidBodySnapshot] {
        var cursor = Axis3(x: 0, y: 0, z: 0)
        return bindings.enumerated().map { index, binding in
            let joint = binding.joint
            cursor = Axis3(
                x: cursor.x + joint.origin.xyz.x,
                y: cursor.y + joint.origin.xyz.y,
                z: cursor.z + joint.origin.xyz.z
            )
            return RigidBodySnapshot(
                id: joint.childLinkID,
                position: cursor,
                velocity: Axis3(x: 0, y: 0, z: 0),
                orientation: approximateOrientation(axis: joint.axis, angle: state.position[index]),
                angularVelocity: Axis3(
                    x: joint.axis.x * state.velocity[index],
                    y: joint.axis.y * state.velocity[index],
                    z: joint.axis.z * state.velocity[index]
                )
            )
        }
    }

    private func approximateOrientation(axis: KuyuVector3, angle: Double) -> QuaternionSnapshot {
        let half = angle / 2.0
        let sinHalf = sin(half)
        return QuaternionSnapshot(
            w: cos(half),
            x: axis.x * sinHalf,
            y: axis.y * sinHalf,
            z: axis.z * sinHalf
        )
    }
}

private struct ArticulatedState: Sendable, Equatable {
    var position: [Double]
    var velocity: [Double]
    var torque: [Double]
}

private struct ArticulatedJointBinding: Sendable, Equatable {
    let joint: JointDefinition
    let attachment: ActuatorAttachment
    let actuator: ActuatorDefinition
    let actuatorSignal: SignalDefinition
}

private struct ArticulatedStateModel: Sendable {
    private let world: KuyuWorldModel
    private let linkByID: [String: LinkDefinition]
    private let childLinksByParent: [String: [String]]

    init(body: KuyuBodyModel, world: KuyuWorldModel) {
        self.world = world
        self.linkByID = Dictionary(uniqueKeysWithValues: body.links.map { ($0.id, $0) })
        var children: [String: [String]] = [:]
        for joint in body.joints {
            children[joint.parentLinkID, default: []].append(joint.childLinkID)
        }
        self.childLinksByParent = children
    }

    func step(
        state: ArticulatedState,
        targets: [Double],
        bindings: [ArticulatedJointBinding],
        deltaTime: Double
    ) throws -> ArticulatedState {
        var next = state
        try ensureFinite(deltaTime, "deltaTime")
        guard deltaTime > 0 else {
            throw ArticulatedRigidBodySimulator.SimulationError.nonFiniteState("deltaTime")
        }
        let substeps = max(world.time.substeps, 1)
        let substepDelta = deltaTime / Double(substeps)
        for _ in 0..<substeps {
            next = try integrateSubstep(
                state: next,
                targets: targets,
                bindings: bindings,
                deltaTime: substepDelta
            )
        }
        return next
    }

    private func integrateSubstep(
        state: ArticulatedState,
        targets: [Double],
        bindings: [ArticulatedJointBinding],
        deltaTime: Double
    ) throws -> ArticulatedState {
        var next = state
        for index in bindings.indices {
            let binding = bindings[index]
            let joint = binding.joint
            let current = state.position[index]
            let velocity = state.velocity[index]
            let target = targets.indices.contains(index) ? targets[index] : current
            let effectiveInertia = inertia(for: binding)
            let effortLimit = effortLimit(for: binding)
            try ensureFinite(current, "position[\(index)]")
            try ensureFinite(velocity, "velocity[\(index)]")
            try ensureFinite(target, "target[\(index)]")
            try ensureFinite(effectiveInertia, "inertia[\(index)]")
            try ensureFinite(effortLimit, "effortLimit[\(index)]")
            let dynamics = binding.actuator.dynamics
            let timeConstant = max(dynamics?.timeConstantSeconds ?? 0.001, 0.001)
            let stiffness = effectiveInertia / (timeConstant * timeConstant)
            let servoDamping = 2.0 * sqrt(max(stiffness * effectiveInertia, 0.0))
            let gravityTorque = gravityTorque(for: joint, position: current)
            let damping = joint.damping + (dynamics?.damping ?? 0)
            let friction = (joint.coulombFriction + (dynamics?.coulombFriction ?? 0)) * sign(velocity)
            let rawTorque = stiffness * (target - current)
                - servoDamping * velocity
                - damping * velocity
                - friction
                + gravityTorque
            let torque = clamp(rawTorque, min: -effortLimit, max: effortLimit)
            let acceleration = torque / effectiveInertia
            let limitedVelocity = velocityLimit(for: joint, proposed: velocity + acceleration * deltaTime)
            var proposedPosition = current + limitedVelocity * deltaTime
            var proposedVelocity = limitedVelocity
            if let lower = joint.lowerLimit, proposedPosition < lower {
                proposedPosition = lower
                proposedVelocity = 0
            }
            if let upper = joint.upperLimit, proposedPosition > upper {
                proposedPosition = upper
                proposedVelocity = 0
            }
            try ensureFinite(torque, "torque[\(index)]")
            try ensureFinite(acceleration, "acceleration[\(index)]")
            try ensureFinite(proposedPosition, "position.next[\(index)]")
            try ensureFinite(proposedVelocity, "velocity.next[\(index)]")
            next.position[index] = proposedPosition
            next.velocity[index] = proposedVelocity
            next.torque[index] = torque
        }
        return next
    }

    private func inertia(for joint: JointDefinition) -> Double {
        let axis = joint.axis
        let descendants = descendantLinks(from: joint.childLinkID)
        let total = descendants.reduce(0.0) { partial, link in
            let rotational = abs(axis.x) * link.inertia.ixx
                + abs(axis.y) * link.inertia.iyy
                + abs(axis.z) * link.inertia.izz
            let leverVector = KuyuVector3(
                x: joint.origin.xyz.x + link.centerOfMass.x,
                y: joint.origin.xyz.y + link.centerOfMass.y,
                z: joint.origin.xyz.z + link.centerOfMass.z
            )
            let lever = perpendicularLever(axis: axis, vector: leverVector)
            return partial + rotational + link.mass * lever * lever
        }
        return max(total, 1e-6)
    }

    private func inertia(for binding: ArticulatedJointBinding) -> Double {
        inertia(for: binding.joint) + (binding.attachment.reflectedInertia ?? 0)
    }

    private func effortLimit(for binding: ArticulatedJointBinding) -> Double {
        let joint = binding.joint
        let jointLimit = joint.effortLimit ?? .greatestFiniteMagnitude
        let attachmentLimit = binding.attachment.torqueLimit
        let actuatorLimit = binding.actuator.dynamics?.torqueLimit ?? .greatestFiniteMagnitude
        let transmissionLimit = actuatorLimit
            * binding.attachment.mechanicalReductionRatio
            * (binding.attachment.efficiency ?? 1.0)
        return max(min(jointLimit, attachmentLimit, transmissionLimit), 1e-6)
    }

    private func gravityTorque(for joint: JointDefinition, position: Double) -> Double {
        let g = abs(world.gravity.acceleration.z)
        let massLever = descendantLinks(from: joint.childLinkID).reduce(0.0) { partial, link in
            let lever = max(abs(joint.origin.xyz.x + link.centerOfMass.x), 0.01)
            return partial + link.mass * lever
        }
        if abs(joint.axis.y) > 0 || abs(joint.axis.x) > 0 {
            return -massLever * g * sin(position)
        }
        return 0
    }

    private func descendantLinks(from rootLinkID: String) -> [LinkDefinition] {
        guard let root = linkByID[rootLinkID] else { return [] }
        var result = [root]
        for childID in childLinksByParent[rootLinkID] ?? [] {
            result.append(contentsOf: descendantLinks(from: childID))
        }
        return result
    }

    private func perpendicularLever(axis: KuyuVector3, vector: KuyuVector3) -> Double {
        let axisLength = sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z)
        guard axisLength > 0 else { return 0.01 }
        let ax = axis.x / axisLength
        let ay = axis.y / axisLength
        let az = axis.z / axisLength
        let dot = vector.x * ax + vector.y * ay + vector.z * az
        let px = vector.x - dot * ax
        let py = vector.y - dot * ay
        let pz = vector.z - dot * az
        return max(sqrt(px * px + py * py + pz * pz), 0.01)
    }

    private func velocityLimit(for joint: JointDefinition, proposed: Double) -> Double {
        guard let limit = joint.velocityLimit else { return proposed }
        return clamp(proposed, min: -limit, max: limit)
    }

    private func sign(_ value: Double) -> Double {
        if value > 0 { return 1 }
        if value < 0 { return -1 }
        return 0
    }

    private func clamp(_ value: Double, min lower: Double, max upper: Double) -> Double {
        Swift.min(Swift.max(value, lower), upper)
    }

    private func ensureFinite(_ value: Double, _ field: String) throws {
        if !value.isFinite {
            throw ArticulatedRigidBodySimulator.SimulationError.nonFiniteState(field)
        }
    }
}
