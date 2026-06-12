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
        case invalidBody(String)
        case invalidDriveProviderOutput(String)
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
        let jointIDs = jointBindings.map(\.joint.id)
        let driveSignalIDs = driveSignals.map(\.id)
        let actuatorSignalIDs = actuatorSignals.map(\.id)
        var provider = driveProvider
        try provider.reset(context: ArticulatedRigidBodyDriveProviderResetContext(
            seed: request.seed,
            jointIDs: jointIDs,
            driveSignalIDs: driveSignalIDs,
            jointRanges: jointRanges
        ))
        let stateModel = ArticulatedStateModel(body: request.body, world: request.world)
        var state = ArticulatedState(
            position: Array(repeating: 0, count: jointBindings.count),
            velocity: Array(repeating: 0, count: jointBindings.count),
            torque: Array(repeating: 0, count: jointBindings.count)
        )
        var targets = state.position
        let stepCount = Int((request.duration / request.timeStep.delta).rounded(.down))
        var logs: [WorldStepLog] = []
        logs.reserveCapacity(stepCount)

        for step in 0..<stepCount {
            if let control {
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
                telemetry: MotorNerveTelemetry(actuatorTelemetry: actuatorTelemetry(
                    state: state,
                    bindings: jointBindings,
                    actuatorSignals: actuatorSignals
                )),
                time: time
            )
            targets = try jointTargets(values: actuatorValues, bindings: jointBindings, actuatorSignals: actuatorSignals)
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
            configHash: "articulated-dynamic-v2-\(request.body.bodyID)-duration-\(request.duration)-dt-\(request.timeStep.delta)-drive-\(provider.providerID)",
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
        let scalars = try scalarState(
            body: body,
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
            plantState: PlantStateSnapshot(root: root, bodies: try linkSnapshots(body: body, scalars: scalars), scalars: scalars),
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
        body: KuyuBodyModel,
        bindings: [ArticulatedJointBinding],
        state: ArticulatedState,
        targets: [Double]
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
            positionsByJointID[binding.joint.id] = state.position[index]
            velocitiesByJointID[binding.joint.id] = state.velocity[index]
            targetsByJointID[binding.joint.id] = target
            torquesByJointID[binding.joint.id] = state.torque[index]
            scalars[signalID] = state.position[index]
            scalars[binding.joint.id] = state.position[index]
            scalars["target_\(signalID)"] = target
            scalars["target_\(binding.joint.id)"] = target
            scalars["velocity_\(signalID)"] = state.velocity[index]
            scalars["velocity_\(binding.joint.id)"] = state.velocity[index]
            scalars["torque_\(signalID)"] = state.torque[index]
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
        return scalars
    }

    private func linkSnapshots(
        body: KuyuBodyModel,
        scalars: [String: Double]
    ) throws -> [RigidBodySnapshot] {
        let childLinkIDs = Set(body.joints.map(\.childLinkID))
        let rootLinks = body.links.filter { !childLinkIDs.contains($0.id) }
        guard !rootLinks.isEmpty else {
            throw SimulationError.invalidBody("root-link")
        }
        var statesByLinkID = Dictionary(
            uniqueKeysWithValues: rootLinks.map { ($0.id, LinkKinematicState.identity) }
        )
        var snapshotsByChildLinkID: [String: RigidBodySnapshot] = [:]
        snapshotsByChildLinkID.reserveCapacity(body.joints.count)
        var unresolved = body.joints

        while !unresolved.isEmpty {
            var remaining: [JointDefinition] = []
            var resolvedCount = 0

            for joint in unresolved {
                guard let parentState = statesByLinkID[joint.parentLinkID] else {
                    remaining.append(joint)
                    continue
                }
                let scalar = scalars[joint.id] ?? joint.homePosition ?? 0
                let velocity = scalars["velocity_\(joint.id)"] ?? 0
                let childState = try childLinkState(
                    parent: parentState,
                    joint: joint,
                    scalar: scalar,
                    velocity: velocity
                )
                statesByLinkID[joint.childLinkID] = childState
                snapshotsByChildLinkID[joint.childLinkID] = RigidBodySnapshot(
                    id: joint.childLinkID,
                    position: axis3(childState.position),
                    velocity: axis3(childState.velocity),
                    orientation: QuaternionSnapshot(orientation: childState.orientation),
                    angularVelocity: axis3(childState.angularVelocity)
                )
                resolvedCount += 1
            }

            if resolvedCount == 0 {
                let ids = remaining.map(\.id).joined(separator: ",")
                throw SimulationError.invalidBody("joint-topology.\(ids)")
            }
            unresolved = remaining
        }

        return body.joints.compactMap { snapshotsByChildLinkID[$0.childLinkID] }
    }

    private func childLinkState(
        parent: LinkKinematicState,
        joint: JointDefinition,
        scalar: Double,
        velocity: Double
    ) throws -> LinkKinematicState {
        let originTranslation = simdVector(joint.origin.xyz)
        let originWorldOffset = parent.orientation.act(originTranslation)
        let originPosition = parent.position + originWorldOffset
        let originVelocity = parent.velocity + simd_cross(parent.angularVelocity, originWorldOffset)
        let originOrientation = (parent.orientation * orientation(fromRPY: joint.origin.rpy)).normalizedQuat

        switch joint.kind {
        case .fixed:
            return LinkKinematicState(
                position: originPosition,
                velocity: originVelocity,
                orientation: originOrientation,
                angularVelocity: parent.angularVelocity
            )
        case .revolute, .continuous:
            let axis = try normalizedAxis(joint)
            let worldAxis = originOrientation.act(axis)
            return LinkKinematicState(
                position: originPosition,
                velocity: originVelocity,
                orientation: (originOrientation * simd_quatd(angle: scalar, axis: axis)).normalizedQuat,
                angularVelocity: parent.angularVelocity + worldAxis * velocity
            )
        case .prismatic:
            let axis = try normalizedAxis(joint)
            let worldAxis = originOrientation.act(axis)
            let displacement = worldAxis * scalar
            return LinkKinematicState(
                position: originPosition + displacement,
                velocity: originVelocity + simd_cross(parent.angularVelocity, displacement) + worldAxis * velocity,
                orientation: originOrientation,
                angularVelocity: parent.angularVelocity
            )
        }
    }

    private func normalizedAxis(_ joint: JointDefinition) throws -> SIMD3<Double> {
        let axis = simdVector(joint.axis)
        let length = simd_length(axis)
        guard length > 0 else {
            throw SimulationError.invalidBody("joint-axis.\(joint.id)")
        }
        return axis / length
    }

    private func orientation(fromRPY rpy: KuyuVector3) -> simd_quatd {
        let roll = simd_quatd(angle: rpy.x, axis: SIMD3<Double>(1, 0, 0))
        let pitch = simd_quatd(angle: rpy.y, axis: SIMD3<Double>(0, 1, 0))
        let yaw = simd_quatd(angle: rpy.z, axis: SIMD3<Double>(0, 0, 1))
        return (yaw * pitch * roll).normalizedQuat
    }

    private func simdVector(_ vector: KuyuVector3) -> SIMD3<Double> {
        SIMD3<Double>(vector.x, vector.y, vector.z)
    }

    private func axis3(_ vector: SIMD3<Double>) -> Axis3 {
        Axis3(x: vector.x, y: vector.y, z: vector.z)
    }

}

private struct LinkKinematicState: Sendable, Equatable {
    var position: SIMD3<Double>
    var velocity: SIMD3<Double>
    var orientation: simd_quatd
    var angularVelocity: SIMD3<Double>

    static let identity = LinkKinematicState(
        position: SIMD3<Double>(repeating: 0),
        velocity: SIMD3<Double>(repeating: 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(repeating: 0)
    )
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
