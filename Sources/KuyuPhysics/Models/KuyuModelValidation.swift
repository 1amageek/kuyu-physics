import EmbodimentContract
import Foundation

public enum KuyuModelValidationError: Error, Equatable {
    case empty(String)
    case duplicate(String)
    case unknownReference(String)
    case invalidRange(String)
    case nonFinite(String)
    case nonPositive(String)
    case unsupportedReadiness(String)
}

public extension KuyuBodyModel {
    func validate() throws {
        if schemaVersion.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("body.schemaVersion")
        }
        if bodyID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("body.bodyID")
        }
        if links.isEmpty {
            throw KuyuModelValidationError.empty("body.links")
        }

        var linkIDs: Set<String> = []
        let frameIDs = Set(frames.map(\.id))
        let materialIDs = Set(materials.map(\.id))
        for link in links {
            if link.id.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw KuyuModelValidationError.empty("body.links.id")
            }
            if !linkIDs.insert(link.id).inserted {
                throw KuyuModelValidationError.duplicate("body.links.\(link.id)")
            }
            try validatePositive(link.mass, "body.links.\(link.id).mass")
            try validateFinite(link.centerOfMass.x, "body.links.\(link.id).centerOfMass.x")
            try validateFinite(link.centerOfMass.y, "body.links.\(link.id).centerOfMass.y")
            try validateFinite(link.centerOfMass.z, "body.links.\(link.id).centerOfMass.z")
            try validateInertia(link.inertia, field: "body.links.\(link.id).inertia")
            if let materialID = link.materialID, !materialIDs.contains(materialID) {
                throw KuyuModelValidationError.unknownReference("body.links.\(link.id).materialID")
            }
            for collision in link.collisions {
                try validateGeometry(collision, field: "body.links.\(link.id).collisions")
            }
            for visual in link.visuals {
                try validateGeometry(visual, field: "body.links.\(link.id).visuals")
            }
        }

        var jointIDs: Set<String> = []
        for joint in joints {
            if joint.id.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw KuyuModelValidationError.empty("body.joints.id")
            }
            if !jointIDs.insert(joint.id).inserted {
                throw KuyuModelValidationError.duplicate("body.joints.\(joint.id)")
            }
            if !linkIDs.contains(joint.parentLinkID) {
                throw KuyuModelValidationError.unknownReference("body.joints.\(joint.id).parentLinkID")
            }
            if !linkIDs.contains(joint.childLinkID) {
                throw KuyuModelValidationError.unknownReference("body.joints.\(joint.id).childLinkID")
            }
            try validateFinite(joint.axis.x, "body.joints.\(joint.id).axis.x")
            try validateFinite(joint.axis.y, "body.joints.\(joint.id).axis.y")
            try validateFinite(joint.axis.z, "body.joints.\(joint.id).axis.z")
            let axisMagnitude = abs(joint.axis.x) + abs(joint.axis.y) + abs(joint.axis.z)
            if joint.kind != .fixed, axisMagnitude <= 0 {
                throw KuyuModelValidationError.invalidRange("body.joints.\(joint.id).axis")
            }
            if let lower = joint.lowerLimit, let upper = joint.upperLimit, lower > upper {
                throw KuyuModelValidationError.invalidRange("body.joints.\(joint.id).limits")
            }
            try validateOptionalNonNegative(joint.effortLimit, "body.joints.\(joint.id).effortLimit")
            try validateOptionalNonNegative(joint.velocityLimit, "body.joints.\(joint.id).velocityLimit")
            try validateNonNegative(joint.damping, "body.joints.\(joint.id).damping")
            try validateNonNegative(joint.coulombFriction, "body.joints.\(joint.id).coulombFriction")
            try validateNonNegative(joint.stiction, "body.joints.\(joint.id).stiction")
            try validateNonNegative(joint.backlash, "body.joints.\(joint.id).backlash")
        }

        var attachmentJointIDs: Set<String> = []
        var attachmentActuatorIDs: Set<String> = []
        for attachment in actuatorAttachments {
            if !jointIDs.contains(attachment.jointID) {
                throw KuyuModelValidationError.unknownReference("body.actuatorAttachments.\(attachment.jointID)")
            }
            if attachment.actuatorID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw KuyuModelValidationError.empty("body.actuatorAttachments.actuatorID")
            }
            if !attachmentJointIDs.insert(attachment.jointID).inserted {
                throw KuyuModelValidationError.duplicate("body.actuatorAttachments.jointID.\(attachment.jointID)")
            }
            if !attachmentActuatorIDs.insert(attachment.actuatorID).inserted {
                throw KuyuModelValidationError.duplicate("body.actuatorAttachments.actuatorID.\(attachment.actuatorID)")
            }
            try validatePositive(attachment.transmissionRatio, "body.actuatorAttachments.transmissionRatio")
            try validatePositive(attachment.torqueLimit, "body.actuatorAttachments.torqueLimit")
        }

        let bodyFrameIDs = frameIDs.union(linkIDs)
        for mount in sensorMounts {
            if mount.sensorID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw KuyuModelValidationError.empty("body.sensorMounts.sensorID")
            }
            if !bodyFrameIDs.contains(mount.frameID) {
                throw KuyuModelValidationError.unknownReference("body.sensorMounts.\(mount.sensorID).frameID")
            }
        }
    }
}

public extension KuyuWorldModel {
    func validate() throws {
        if schemaVersion.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("world.schemaVersion")
        }
        if worldID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("world.worldID")
        }
        try validatePositive(time.fixedStepSeconds, "world.time.fixedStepSeconds")
        if time.substeps <= 0 {
            throw KuyuModelValidationError.invalidRange("world.time.substeps")
        }
        if solver.iterations < 0 {
            throw KuyuModelValidationError.invalidRange("world.solver.iterations")
        }
        try validateNonNegative(solver.tolerance, "world.solver.tolerance")
        try validateFinite(gravity.acceleration.x, "world.gravity.acceleration.x")
        try validateFinite(gravity.acceleration.y, "world.gravity.acceleration.y")
        try validateFinite(gravity.acceleration.z, "world.gravity.acceleration.z")
        try validateNonNegative(nap.forceAbsoluteThreshold, "world.nap.forceAbsoluteThreshold")
        try validateNonNegative(nap.forceRelativeThreshold, "world.nap.forceRelativeThreshold")
        try validateNonNegative(nap.torqueAbsoluteThreshold, "world.nap.torqueAbsoluteThreshold")
        try validateNonNegative(nap.torqueRelativeThreshold, "world.nap.torqueRelativeThreshold")
    }
}

public extension KuyuRobotManifest {
    func validate() throws {
        if schemaVersion.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("manifest.schemaVersion")
        }
        if robotID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("manifest.robotID")
        }
        if bodyModel.path.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("manifest.bodyModel.path")
        }
        if embodimentContract.path.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("manifest.embodimentContract.path")
        }
    }
}

public struct ReadinessGate: Sendable {
    public init() {}

    public func validate(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        embodiment: EmbodimentContract,
        report: CompatibilityReport?,
        requiredLevel: ReadinessLevel
    ) throws -> ReadinessLevel {
        try body.validate()
        try world.validate()
        try embodiment.validate()

        if let report {
            if report.hasUnsupportedMappings {
                throw KuyuModelValidationError.unsupportedReadiness("compatibility.unsupported")
            }
            if report.readinessLevel < requiredLevel {
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
            throw KuyuModelValidationError.unsupportedReadiness("hardwareParity")
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
        if body.actuatorAttachments.isEmpty {
            throw KuyuModelValidationError.empty("readiness.dynamic.actuatorAttachments")
        }
        let movableJointIDs = Set(body.joints.filter { $0.kind == .revolute || $0.kind == .continuous || $0.kind == .prismatic }.map(\.id))
        let attachedJointIDs = Set(body.actuatorAttachments.map(\.jointID))
        if movableJointIDs != attachedJointIDs {
            throw KuyuModelValidationError.unknownReference("readiness.dynamic.actuatorAttachments.coverage")
        }
        for actuator in embodiment.actuators {
            guard let dynamics = actuator.dynamics else {
                throw KuyuModelValidationError.empty("readiness.dynamic.actuators.\(actuator.id).dynamics")
            }
            try validatePositive(dynamics.timeConstantSeconds, "readiness.dynamic.actuators.\(actuator.id).timeConstantSeconds")
            try validateOptionalPositive(dynamics.torqueLimit, "readiness.dynamic.actuators.\(actuator.id).torqueLimit")
        }
        let actuatorsByID = Dictionary(uniqueKeysWithValues: embodiment.actuators.map { ($0.id, $0) })
        for attachment in body.actuatorAttachments {
            guard let actuator = actuatorsByID[attachment.actuatorID] else {
                throw KuyuModelValidationError.unknownReference("readiness.dynamic.actuatorAttachments.\(attachment.actuatorID)")
            }
            if actuator.channels.count != 1 {
                throw KuyuModelValidationError.invalidRange("readiness.dynamic.actuators.\(actuator.id).channels")
            }
            let joint = body.joints.first { $0.id == attachment.jointID }
            if joint?.kind != .continuous,
               (joint?.lowerLimit == nil || joint?.upperLimit == nil) {
                throw KuyuModelValidationError.empty("readiness.dynamic.joints.\(attachment.jointID).limits")
            }
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
        for link in body.links where link.collisions.isEmpty {
            throw KuyuModelValidationError.empty("contactTraining.links.\(link.id).collisions")
        }
        if body.links.contains(where: { $0.materialID == nil }) {
            throw KuyuModelValidationError.empty("contactTraining.links.materialID")
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

private func validateGeometry(_ geometry: GeometryInstance, field: String) throws {
    switch geometry.kind {
    case .box:
        guard let size = geometry.size else {
            throw KuyuModelValidationError.empty("\(field).size")
        }
        try validatePositive(size.x, "\(field).size.x")
        try validatePositive(size.y, "\(field).size.y")
        try validatePositive(size.z, "\(field).size.z")
    case .cylinder:
        guard let radius = geometry.radius else {
            throw KuyuModelValidationError.empty("\(field).radius")
        }
        guard let length = geometry.length else {
            throw KuyuModelValidationError.empty("\(field).length")
        }
        try validatePositive(radius, "\(field).radius")
        try validatePositive(length, "\(field).length")
    case .sphere:
        guard let radius = geometry.radius else {
            throw KuyuModelValidationError.empty("\(field).radius")
        }
        try validatePositive(radius, "\(field).radius")
    case .mesh:
        if geometry.meshPath?.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty != false {
            throw KuyuModelValidationError.empty("\(field).meshPath")
        }
    }
}

private func validateInertia(_ inertia: KuyuInertiaTensor, field: String) throws {
    try validatePositive(inertia.ixx, "\(field).ixx")
    try validatePositive(inertia.iyy, "\(field).iyy")
    try validatePositive(inertia.izz, "\(field).izz")
    try validateFinite(inertia.ixy, "\(field).ixy")
    try validateFinite(inertia.ixz, "\(field).ixz")
    try validateFinite(inertia.iyz, "\(field).iyz")
}

private func validatePositive(_ value: Double, _ field: String) throws {
    try validateFinite(value, field)
    if value <= 0 {
        throw KuyuModelValidationError.nonPositive(field)
    }
}

private func validateOptionalPositive(_ value: Double?, _ field: String) throws {
    guard let value else { return }
    try validatePositive(value, field)
}

private func validateNonNegative(_ value: Double, _ field: String) throws {
    try validateFinite(value, field)
    if value < 0 {
        throw KuyuModelValidationError.invalidRange(field)
    }
}

private func validateOptionalNonNegative(_ value: Double?, _ field: String) throws {
    guard let value else { return }
    try validateNonNegative(value, field)
}

private func validateFinite(_ value: Double, _ field: String) throws {
    if !value.isFinite {
        throw KuyuModelValidationError.nonFinite(field)
    }
}
