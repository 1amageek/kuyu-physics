import EmbodimentContract
import KuyuCore

extension ArticulatedRigidBodySimulator {
    func orderedDriveSignals(from contract: EmbodimentContract) throws -> [SignalDefinition] {
        let byID = Dictionary(uniqueKeysWithValues: contract.signals.drive.map { ($0.id, $0) })
        return try contract.control.driveChannels.map { id in
            guard let signal = byID[id] else {
                throw SimulationError.invalidBody("driveChannels.\(id)")
            }
            return signal
        }
    }

    func bindings(
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

    func actuatorMapBySignalID(_ actuators: [ActuatorDefinition]) throws -> [String: String] {
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

    func jointTargets(
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

    func ranges(bindings: [ArticulatedJointBinding]) throws -> [ClosedRange<Double>] {
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

    func initialPositions(
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
}
