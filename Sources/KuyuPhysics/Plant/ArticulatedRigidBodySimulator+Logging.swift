import EmbodimentContract
import KuyuCore

extension ArticulatedRigidBodySimulator {
    func makeStepLog(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
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

    func jointSamples(values: [Double], timestamp: Double) throws -> [ChannelSample] {
        try values.enumerated().map { index, value in
            try ChannelSample(channelIndex: UInt32(index), value: value, timestamp: timestamp)
        }
    }

    func actuatorTelemetry(
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

    func scalarState(
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
}
