import EmbodimentContract

public struct ReadinessGate: Sendable {
    public init() {}

    public func validate(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        embodiment: EmbodimentContract,
        report: CompatibilityReport?,
        requiredLevel: ReadinessLevel,
        hardwareReport: HardwareCalibrationReport? = nil
    ) throws -> ReadinessLevel {
        try body.validate()
        try world.validate()
        try embodiment.validate()

        if let report {
            if report.hasUnsupportedMappings {
                throw KuyuModelValidationError.unsupportedReadiness("compatibility.unsupported")
            }
            let compatibilityRequiredLevel = requiredLevel == .hardwareParity ? .dynamicSimulation : requiredLevel
            if report.readinessLevel < compatibilityRequiredLevel {
                throw KuyuModelValidationError.unsupportedReadiness("compatibility.\(report.readinessLevel.rawValue)")
            }
        }

        switch requiredLevel {
        case .visualPreview:
            return .visualPreview
        case .kinematicPreview:
            try requireKinematic(body: body, embodiment: embodiment)
            return .kinematicPreview
        case .dynamicSimulation:
            try requireDynamic(body: body, world: world, embodiment: embodiment)
            return .dynamicSimulation
        case .contactTraining:
            try requireDynamic(body: body, world: world, embodiment: embodiment)
            try requireContactTraining(body: body, world: world)
            return .contactTraining
        case .hardwareParity:
            try requireDynamic(body: body, world: world, embodiment: embodiment)
            guard let hardwareReport else {
                throw KuyuModelValidationError.empty("hardwareParity.report")
            }
            do {
                try hardwareReport.validateHardwareParity(body: body, embodiment: embodiment)
            } catch {
                throw KuyuModelValidationError.unsupportedReadiness("hardwareParity.\(String(describing: error))")
            }
            return .hardwareParity
        }
    }

    private func requireKinematic(
        body: KuyuBodyModel,
        embodiment: EmbodimentContract
    ) throws {
        let jointIDs = Set(body.joints.map(\.id))
        for attachment in body.actuatorAttachments where !jointIDs.contains(attachment.jointID) {
            throw KuyuModelValidationError.unknownReference("readiness.kinematic.actuatorAttachment")
        }
        try requireBodyEmbodimentBinding(body: body, embodiment: embodiment)
        if embodiment.control.driveChannels.isEmpty || embodiment.signals.actuator.isEmpty {
            throw KuyuModelValidationError.empty("readiness.kinematic.channels")
        }
    }

    private func requireDynamic(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        embodiment: EmbodimentContract
    ) throws {
        try requireKinematic(body: body, embodiment: embodiment)
        if world.gravity.kind == .none {
            throw KuyuModelValidationError.unsupportedReadiness("dynamicSimulation.gravity")
        }
        for link in body.links {
            try validatePositive(link.mass, "readiness.dynamic.links.\(link.id).mass")
            try validateInertia(link.inertia, field: "readiness.dynamic.links.\(link.id).inertia")
        }
        let movableJointIDs = Set(body.joints.filter { joint in
            joint.mimic == nil && (joint.kind == .revolute || joint.kind == .continuous || joint.kind == .prismatic)
        }.map(\.id))
        try requireDynamicActuators(embodiment: embodiment)
        if body.actuatorAttachments.isEmpty {
            guard movableJointIDs.isEmpty else {
                throw KuyuModelValidationError.empty("readiness.dynamic.actuatorAttachments")
            }
            try requireRigidActuatorPath(embodiment: embodiment)
            return
        }
        let attachedJointIDs = Set(body.actuatorAttachments.map(\.jointID))
        if movableJointIDs != attachedJointIDs {
            throw KuyuModelValidationError.unknownReference("readiness.dynamic.actuatorAttachments.coverage")
        }
        try requireArticulatedActuatorPath(embodiment: embodiment)
        let actuatorsByID = Dictionary(uniqueKeysWithValues: embodiment.actuators.map { ($0.id, $0) })
        for attachment in body.actuatorAttachments {
            guard let actuator = actuatorsByID[attachment.actuatorID] else {
                throw KuyuModelValidationError.unknownReference("readiness.dynamic.actuatorAttachments.\(attachment.actuatorID)")
            }
            if attachment.mountFrameID == nil {
                throw KuyuModelValidationError.empty("readiness.dynamic.actuatorAttachments.\(attachment.actuatorID).mountFrameID")
            }
            if !body.actuatorMounts.contains(where: { $0.actuatorID == attachment.actuatorID }) {
                throw KuyuModelValidationError.empty("readiness.dynamic.actuatorMounts.\(attachment.actuatorID)")
            }
            let joint = body.joints.first { $0.id == attachment.jointID }
            if joint?.kind != .continuous,
               (joint?.lowerLimit == nil || joint?.upperLimit == nil) {
                throw KuyuModelValidationError.empty("readiness.dynamic.joints.\(attachment.jointID).limits")
            }
            if let joint {
                try requireActuatorLimitsWithinJointEnvelope(
                    actuator: actuator,
                    attachment: attachment,
                    joint: joint
                )
            }
        }
    }

    private func requireDynamicActuators(embodiment: EmbodimentContract) throws {
        guard !embodiment.actuators.isEmpty else {
            throw KuyuModelValidationError.empty("readiness.dynamic.actuators")
        }
        for actuator in embodiment.actuators {
            guard let dynamics = actuator.dynamics else {
                throw KuyuModelValidationError.empty("readiness.dynamic.actuators.\(actuator.id).dynamics")
            }
            try validatePositive(dynamics.timeConstantSeconds, "readiness.dynamic.actuators.\(actuator.id).timeConstantSeconds")
            try validateOptionalPositive(dynamics.torqueLimit, "readiness.dynamic.actuators.\(actuator.id).torqueLimit")
        }
    }

    private func requireArticulatedActuatorPath(embodiment: EmbodimentContract) throws {
        for actuator in embodiment.actuators {
            if actuator.channels.count != 1 {
                throw KuyuModelValidationError.invalidRange("readiness.dynamic.actuators.\(actuator.id).channels")
            }
        }
    }

    private func requireRigidActuatorPath(embodiment: EmbodimentContract) throws {
        guard !embodiment.control.driveChannels.isEmpty else {
            throw KuyuModelValidationError.empty("readiness.dynamic.rigid.driveChannels")
        }
        let actuatorSignalCount = embodiment.signals.actuator.count
        guard embodiment.control.driveChannels.count == actuatorSignalCount,
              embodiment.actuators.count == actuatorSignalCount,
              actuatorSignalCount == 1 || actuatorSignalCount == 4 else {
            throw KuyuModelValidationError.invalidRange("readiness.dynamic.rigid.channelCount")
        }
        try requireArticulatedActuatorPath(embodiment: embodiment)
        let actuatorSignalIDs = Set(embodiment.signals.actuator.map(\.id))
        for actuator in embodiment.actuators {
            guard let channel = actuator.channels.first, actuatorSignalIDs.contains(channel) else {
                throw KuyuModelValidationError.unknownReference("readiness.dynamic.rigid.actuator.\(actuator.id)")
            }
        }
    }

    private func requireActuatorLimitsWithinJointEnvelope(
        actuator: ActuatorDefinition,
        attachment: ActuatorAttachment,
        joint: JointDefinition
    ) throws {
        guard joint.kind != .continuous,
              let lowerLimit = joint.lowerLimit,
              let upperLimit = joint.upperLimit else {
            return
        }
        let jointRange = ArticulatedActuatorMapping.jointRange(
            actuatorLimits: actuator.limits,
            attachment: attachment
        )
        guard jointRange.lowerBound.isFinite, jointRange.upperBound.isFinite else {
            throw KuyuModelValidationError.nonFinite("readiness.dynamic.actuators.\(actuator.id).limits")
        }
        let tolerance = 1e-9
        if jointRange.lowerBound < lowerLimit - tolerance || jointRange.upperBound > upperLimit + tolerance {
            throw KuyuModelValidationError.invalidRange("readiness.dynamic.actuators.\(actuator.id).limits")
        }
    }

    private func requireContactTraining(
        body: KuyuBodyModel,
        world: KuyuWorldModel
    ) throws {
        guard world.contact.mode != .disabled else {
            throw KuyuModelValidationError.unsupportedReadiness("contactTraining.contact.disabled")
        }
        if world.materials.isEmpty {
            throw KuyuModelValidationError.empty("contactTraining.world.materials")
        }
        let bodyMaterialsByID = Dictionary(uniqueKeysWithValues: body.materials.map { ($0.id, $0) })
        for link in body.links where link.collisions.isEmpty {
            throw KuyuModelValidationError.empty("contactTraining.links.\(link.id).collisions")
        }
        for link in body.links {
            guard let materialID = link.materialID else {
                throw KuyuModelValidationError.empty("contactTraining.links.\(link.id).materialID")
            }
            guard let material = bodyMaterialsByID[materialID] else {
                throw KuyuModelValidationError.unknownReference("contactTraining.links.\(link.id).materialID")
            }
            if material.staticFriction == nil {
                throw KuyuModelValidationError.empty("contactTraining.body.materials.\(materialID).staticFriction")
            }
            if material.dynamicFriction == nil {
                throw KuyuModelValidationError.empty("contactTraining.body.materials.\(materialID).dynamicFriction")
            }
            if material.restitution == nil {
                throw KuyuModelValidationError.empty("contactTraining.body.materials.\(materialID).restitution")
            }
        }
    }

    private func requireBodyEmbodimentBinding(
        body: KuyuBodyModel,
        embodiment: EmbodimentContract
    ) throws {
        if body.bodyID != embodiment.bodyID {
            throw KuyuModelValidationError.unknownReference("readiness.bodyID.\(embodiment.bodyID)")
        }

        let actuatorIDs = Set(embodiment.actuators.map(\.id))
        for attachment in body.actuatorAttachments where !actuatorIDs.contains(attachment.actuatorID) {
            throw KuyuModelValidationError.unknownReference("readiness.actuatorAttachments.\(attachment.actuatorID)")
        }

        let sensorIDs = Set(embodiment.sensors.map(\.id))
        for mount in body.sensorMounts where !sensorIDs.contains(mount.sensorID) {
            throw KuyuModelValidationError.unknownReference("readiness.sensorMounts.\(mount.sensorID)")
        }
    }
}
