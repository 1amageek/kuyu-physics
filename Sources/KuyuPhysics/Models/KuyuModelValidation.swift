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
        var frameIDs: Set<String> = []
        let materialIDs = Set(materials.map(\.id))
        for frame in frames {
            if frame.id.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw KuyuModelValidationError.empty("body.frames.id")
            }
            if !frameIDs.insert(frame.id).inserted {
                throw KuyuModelValidationError.duplicate("body.frames.\(frame.id)")
            }
            try validatePose(frame.pose, field: "body.frames.\(frame.id).pose")
        }
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
            if let softLower = joint.softLowerLimit, let softUpper = joint.softUpperLimit, softLower > softUpper {
                throw KuyuModelValidationError.invalidRange("body.joints.\(joint.id).softLimits")
            }
            if let lower = joint.lowerLimit, let softLower = joint.softLowerLimit, softLower < lower {
                throw KuyuModelValidationError.invalidRange("body.joints.\(joint.id).softLowerLimit")
            }
            if let upper = joint.upperLimit, let softUpper = joint.softUpperLimit, softUpper > upper {
                throw KuyuModelValidationError.invalidRange("body.joints.\(joint.id).softUpperLimit")
            }
            if let home = joint.homePosition {
                try validateFinite(home, "body.joints.\(joint.id).homePosition")
                if let lower = joint.lowerLimit, home < lower {
                    throw KuyuModelValidationError.invalidRange("body.joints.\(joint.id).homePosition")
                }
                if let upper = joint.upperLimit, home > upper {
                    throw KuyuModelValidationError.invalidRange("body.joints.\(joint.id).homePosition")
                }
            }
            try validateOptionalNonNegative(joint.effortLimit, "body.joints.\(joint.id).effortLimit")
            try validateOptionalNonNegative(joint.velocityLimit, "body.joints.\(joint.id).velocityLimit")
            try validateNonNegative(joint.damping, "body.joints.\(joint.id).damping")
            try validateNonNegative(joint.coulombFriction, "body.joints.\(joint.id).coulombFriction")
            try validateNonNegative(joint.stiction, "body.joints.\(joint.id).stiction")
            try validateNonNegative(joint.backlash, "body.joints.\(joint.id).backlash")
        }
        for joint in joints {
            guard let mimic = joint.mimic else { continue }
            if mimic.jointID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw KuyuModelValidationError.empty("body.joints.\(joint.id).mimic.jointID")
            }
            if mimic.jointID == joint.id {
                throw KuyuModelValidationError.invalidRange("body.joints.\(joint.id).mimic.self")
            }
            if !jointIDs.contains(mimic.jointID) {
                throw KuyuModelValidationError.unknownReference("body.joints.\(joint.id).mimic.\(mimic.jointID)")
            }
            try validateFinite(mimic.multiplier, "body.joints.\(joint.id).mimic.multiplier")
            try validateFinite(mimic.offset, "body.joints.\(joint.id).mimic.offset")
        }

        let bodyFrameIDs = frameIDs.union(linkIDs)
        for frame in frames {
            if let parentID = frame.parentID, !bodyFrameIDs.contains(parentID) {
                throw KuyuModelValidationError.unknownReference("body.frames.\(frame.id).parentID")
            }
        }

        var mountActuatorIDs: Set<String> = []
        var mountFrameIDs: Set<String> = []
        for mount in actuatorMounts {
            if mount.actuatorID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw KuyuModelValidationError.empty("body.actuatorMounts.actuatorID")
            }
            if mount.frameID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw KuyuModelValidationError.empty("body.actuatorMounts.frameID")
            }
            if !mountActuatorIDs.insert(mount.actuatorID).inserted {
                throw KuyuModelValidationError.duplicate("body.actuatorMounts.actuatorID.\(mount.actuatorID)")
            }
            if !mountFrameIDs.insert(mount.frameID).inserted {
                throw KuyuModelValidationError.duplicate("body.actuatorMounts.frameID.\(mount.frameID)")
            }
            if !linkIDs.contains(mount.parentLinkID) {
                throw KuyuModelValidationError.unknownReference("body.actuatorMounts.\(mount.actuatorID).parentLinkID")
            }
            if !bodyFrameIDs.contains(mount.frameID) {
                throw KuyuModelValidationError.unknownReference("body.actuatorMounts.\(mount.actuatorID).frameID")
            }
            try validatePose(mount.pose, field: "body.actuatorMounts.\(mount.actuatorID).pose")
            try validateFinite(mount.outputAxis.x, "body.actuatorMounts.\(mount.actuatorID).outputAxis.x")
            try validateFinite(mount.outputAxis.y, "body.actuatorMounts.\(mount.actuatorID).outputAxis.y")
            try validateFinite(mount.outputAxis.z, "body.actuatorMounts.\(mount.actuatorID).outputAxis.z")
            if vectorMagnitude(mount.outputAxis) <= 0 {
                throw KuyuModelValidationError.invalidRange("body.actuatorMounts.\(mount.actuatorID).outputAxis")
            }
            if let housing = mount.housing {
                try validateGeometry(housing, field: "body.actuatorMounts.\(mount.actuatorID).housing")
            }
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
            if let mountFrameID = attachment.mountFrameID, !bodyFrameIDs.contains(mountFrameID) {
                throw KuyuModelValidationError.unknownReference("body.actuatorAttachments.\(attachment.actuatorID).mountFrameID")
            }
            try validatePositive(attachment.mechanicalReductionRatio, "body.actuatorAttachments.mechanicalReductionRatio")
            try validateFinite(attachment.commandDirection, "body.actuatorAttachments.commandDirection")
            if attachment.commandDirection == 0 {
                throw KuyuModelValidationError.invalidRange("body.actuatorAttachments.commandDirection")
            }
            try validateFinite(attachment.actuatorZeroOffset, "body.actuatorAttachments.actuatorZeroOffset")
            try validateFinite(attachment.jointZeroOffset, "body.actuatorAttachments.jointZeroOffset")
            if let efficiency = attachment.efficiency {
                try validatePositive(efficiency, "body.actuatorAttachments.efficiency")
                if efficiency > 1 {
                    throw KuyuModelValidationError.invalidRange("body.actuatorAttachments.efficiency")
                }
            }
            try validateOptionalNonNegative(attachment.reflectedInertia, "body.actuatorAttachments.reflectedInertia")
        }

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
        if body.actuatorAttachments.isEmpty {
            throw KuyuModelValidationError.empty("readiness.dynamic.actuatorAttachments")
        }
        let movableJointIDs = Set(body.joints.filter { joint in
            joint.mimic == nil && (joint.kind == .revolute || joint.kind == .continuous || joint.kind == .prismatic)
        }.map(\.id))
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
            if attachment.mountFrameID == nil {
                throw KuyuModelValidationError.empty("readiness.dynamic.actuatorAttachments.\(attachment.actuatorID).mountFrameID")
            }
            if !body.actuatorMounts.contains(where: { $0.actuatorID == attachment.actuatorID }) {
                throw KuyuModelValidationError.empty("readiness.dynamic.actuatorMounts.\(attachment.actuatorID)")
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

private func validatePose(_ pose: KuyuPose, field: String) throws {
    try validateFinite(pose.xyz.x, "\(field).xyz.x")
    try validateFinite(pose.xyz.y, "\(field).xyz.y")
    try validateFinite(pose.xyz.z, "\(field).xyz.z")
    try validateFinite(pose.rpy.x, "\(field).rpy.x")
    try validateFinite(pose.rpy.y, "\(field).rpy.y")
    try validateFinite(pose.rpy.z, "\(field).rpy.z")
}

private func vectorMagnitude(_ vector: KuyuVector3) -> Double {
    sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
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
