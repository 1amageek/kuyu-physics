import Foundation

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
        var materialIDs: Set<String> = []
        for material in materials {
            if material.id.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                throw KuyuModelValidationError.empty("body.materials.id")
            }
            if !materialIDs.insert(material.id).inserted {
                throw KuyuModelValidationError.duplicate("body.materials.\(material.id)")
            }
            try validateOptionalPositive(material.density, "body.materials.\(material.id).density")
            try validateOptionalNonNegative(material.staticFriction, "body.materials.\(material.id).staticFriction")
            try validateOptionalNonNegative(material.dynamicFriction, "body.materials.\(material.id).dynamicFriction")
            if let staticFriction = material.staticFriction,
               let dynamicFriction = material.dynamicFriction,
               dynamicFriction > staticFriction {
                throw KuyuModelValidationError.invalidRange("body.materials.\(material.id).dynamicFriction")
            }
            try validateOptionalUnitInterval(material.restitution, "body.materials.\(material.id).restitution")
        }
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
            if let compliance = link.compliance {
                try validateCompliance(compliance, field: "body.links.\(link.id).compliance")
            }
        }

        var jointIDs: Set<String> = []
        var childLinkIDs: Set<String> = []
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
            if joint.parentLinkID == joint.childLinkID {
                throw KuyuModelValidationError.invalidRange("body.joints.\(joint.id).selfLink")
            }
            if !childLinkIDs.insert(joint.childLinkID).inserted {
                throw KuyuModelValidationError.duplicate("body.joints.childLinkID.\(joint.childLinkID)")
            }
            try validatePose(joint.origin, field: "body.joints.\(joint.id).origin")
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
            try validateOptionalFinite(joint.lowerLimit, "body.joints.\(joint.id).lowerLimit")
            try validateOptionalFinite(joint.upperLimit, "body.joints.\(joint.id).upperLimit")
            try validateOptionalFinite(joint.softLowerLimit, "body.joints.\(joint.id).softLowerLimit")
            try validateOptionalFinite(joint.softUpperLimit, "body.joints.\(joint.id).softUpperLimit")
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
            if let compliance = joint.compliance {
                try validateCompliance(compliance, field: "body.joints.\(joint.id).compliance")
            }
        }
        try validateJointTopology(joints: joints)
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
            if abs(abs(attachment.commandDirection) - 1.0) > 1e-12 {
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
            try validatePose(mount.pose, field: "body.sensorMounts.\(mount.sensorID).pose")
        }
    }
}
