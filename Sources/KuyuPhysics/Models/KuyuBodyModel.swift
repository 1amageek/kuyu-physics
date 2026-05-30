import Foundation

public struct KuyuBodyModel: Sendable, Codable, Equatable {
    public let schemaVersion: String
    public let bodyID: String
    public let name: String
    public let category: String
    public let provenance: [ModelProvenance]
    public let units: UnitDeclaration
    public let frames: [FrameDefinition]
    public let links: [LinkDefinition]
    public let joints: [JointDefinition]
    public let materials: [BodyMaterial]
    public let actuatorMounts: [ActuatorMount]
    public let actuatorAttachments: [ActuatorAttachment]
    public let sensorMounts: [SensorMount]

    enum CodingKeys: String, CodingKey {
        case schemaVersion
        case bodyID
        case name
        case category
        case provenance
        case units
        case frames
        case links
        case joints
        case materials
        case actuatorMounts
        case actuatorAttachments
        case sensorMounts
    }

    public init(
        schemaVersion: String,
        bodyID: String,
        name: String,
        category: String,
        provenance: [ModelProvenance] = [],
        units: UnitDeclaration = .si,
        frames: [FrameDefinition] = [],
        links: [LinkDefinition],
        joints: [JointDefinition],
        materials: [BodyMaterial] = [],
        actuatorMounts: [ActuatorMount] = [],
        actuatorAttachments: [ActuatorAttachment] = [],
        sensorMounts: [SensorMount] = []
    ) {
        self.schemaVersion = schemaVersion
        self.bodyID = bodyID
        self.name = name
        self.category = category
        self.provenance = provenance
        self.units = units
        self.frames = frames
        self.links = links
        self.joints = joints
        self.materials = materials
        self.actuatorMounts = actuatorMounts
        self.actuatorAttachments = actuatorAttachments
        self.sensorMounts = sensorMounts
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.schemaVersion = try container.decode(String.self, forKey: .schemaVersion)
        self.bodyID = try container.decode(String.self, forKey: .bodyID)
        self.name = try container.decode(String.self, forKey: .name)
        self.category = try container.decode(String.self, forKey: .category)
        self.provenance = try container.decodeIfPresent([ModelProvenance].self, forKey: .provenance) ?? []
        self.units = try container.decodeIfPresent(UnitDeclaration.self, forKey: .units) ?? .si
        self.frames = try container.decodeIfPresent([FrameDefinition].self, forKey: .frames) ?? []
        self.links = try container.decode([LinkDefinition].self, forKey: .links)
        self.joints = try container.decode([JointDefinition].self, forKey: .joints)
        self.materials = try container.decodeIfPresent([BodyMaterial].self, forKey: .materials) ?? []
        self.actuatorMounts = try container.decodeIfPresent([ActuatorMount].self, forKey: .actuatorMounts) ?? []
        self.actuatorAttachments = try container.decodeIfPresent(
            [ActuatorAttachment].self,
            forKey: .actuatorAttachments
        ) ?? []
        self.sensorMounts = try container.decodeIfPresent([SensorMount].self, forKey: .sensorMounts) ?? []
    }
}

public struct ModelProvenance: Sendable, Codable, Equatable {
    public let source: String
    public let format: String
    public let sha256: String?
    public let notes: String?

    public init(source: String, format: String, sha256: String? = nil, notes: String? = nil) {
        self.source = source
        self.format = format
        self.sha256 = sha256
        self.notes = notes
    }
}

public struct UnitDeclaration: Sendable, Codable, Equatable {
    public let length: String
    public let mass: String
    public let angle: String
    public let time: String
    public let force: String
    public let torque: String

    public init(
        length: String,
        mass: String,
        angle: String,
        time: String,
        force: String,
        torque: String
    ) {
        self.length = length
        self.mass = mass
        self.angle = angle
        self.time = time
        self.force = force
        self.torque = torque
    }

    public static let si = UnitDeclaration(
        length: "m",
        mass: "kg",
        angle: "rad",
        time: "s",
        force: "N",
        torque: "N*m"
    )
}

public struct FrameDefinition: Sendable, Codable, Equatable {
    public let id: String
    public let parentID: String?
    public let pose: KuyuPose

    public init(id: String, parentID: String? = nil, pose: KuyuPose = KuyuPose()) {
        self.id = id
        self.parentID = parentID
        self.pose = pose
    }
}

public struct LinkDefinition: Sendable, Codable, Equatable {
    public let id: String
    public let mass: Double
    public let centerOfMass: KuyuVector3
    public let inertia: KuyuInertiaTensor
    public let visuals: [GeometryInstance]
    public let collisions: [GeometryInstance]
    public let materialID: String?
    public let compliance: ComplianceModel?

    public init(
        id: String,
        mass: Double,
        centerOfMass: KuyuVector3,
        inertia: KuyuInertiaTensor,
        visuals: [GeometryInstance] = [],
        collisions: [GeometryInstance] = [],
        materialID: String? = nil,
        compliance: ComplianceModel? = nil
    ) {
        self.id = id
        self.mass = mass
        self.centerOfMass = centerOfMass
        self.inertia = inertia
        self.visuals = visuals
        self.collisions = collisions
        self.materialID = materialID
        self.compliance = compliance
    }
}

public enum GeometryKind: String, Sendable, Codable, Equatable {
    case box
    case cylinder
    case sphere
    case mesh
}

public struct GeometryInstance: Sendable, Codable, Equatable {
    public let id: String
    public let kind: GeometryKind
    public let pose: KuyuPose
    public let size: KuyuVector3?
    public let radius: Double?
    public let length: Double?
    public let meshPath: String?
    public let meshFormat: RenderMeshFormat?
    public let scale: KuyuVector3?

    public init(
        id: String,
        kind: GeometryKind,
        pose: KuyuPose = KuyuPose(),
        size: KuyuVector3? = nil,
        radius: Double? = nil,
        length: Double? = nil,
        meshPath: String? = nil,
        meshFormat: RenderMeshFormat? = nil,
        scale: KuyuVector3? = nil
    ) {
        self.id = id
        self.kind = kind
        self.pose = pose
        self.size = size
        self.radius = radius
        self.length = length
        self.meshPath = meshPath
        self.meshFormat = meshFormat
        self.scale = scale
    }
}

public struct BodyMaterial: Sendable, Codable, Equatable {
    public let id: String
    public let density: Double?
    public let staticFriction: Double?
    public let dynamicFriction: Double?
    public let restitution: Double?

    public init(
        id: String,
        density: Double? = nil,
        staticFriction: Double? = nil,
        dynamicFriction: Double? = nil,
        restitution: Double? = nil
    ) {
        self.id = id
        self.density = density
        self.staticFriction = staticFriction
        self.dynamicFriction = dynamicFriction
        self.restitution = restitution
    }
}

public struct ComplianceModel: Sendable, Codable, Equatable {
    public let stiffness: Double
    public let damping: Double

    public init(stiffness: Double, damping: Double) {
        self.stiffness = stiffness
        self.damping = damping
    }
}

public enum JointKind: String, Sendable, Codable, Equatable {
    case fixed
    case revolute
    case continuous
    case prismatic
}

public struct JointDefinition: Sendable, Codable, Equatable {
    public let id: String
    public let kind: JointKind
    public let parentLinkID: String
    public let childLinkID: String
    public let origin: KuyuPose
    public let axis: KuyuVector3
    public let lowerLimit: Double?
    public let upperLimit: Double?
    public let effortLimit: Double?
    public let velocityLimit: Double?
    public let softLowerLimit: Double?
    public let softUpperLimit: Double?
    public let homePosition: Double?
    public let damping: Double
    public let coulombFriction: Double
    public let stiction: Double
    public let backlash: Double
    public let compliance: ComplianceModel?
    public let mimic: JointMimic?

    public init(
        id: String,
        kind: JointKind,
        parentLinkID: String,
        childLinkID: String,
        origin: KuyuPose,
        axis: KuyuVector3,
        lowerLimit: Double? = nil,
        upperLimit: Double? = nil,
        effortLimit: Double? = nil,
        velocityLimit: Double? = nil,
        softLowerLimit: Double? = nil,
        softUpperLimit: Double? = nil,
        homePosition: Double? = nil,
        damping: Double = 0,
        coulombFriction: Double = 0,
        stiction: Double = 0,
        backlash: Double = 0,
        compliance: ComplianceModel? = nil,
        mimic: JointMimic? = nil
    ) {
        self.id = id
        self.kind = kind
        self.parentLinkID = parentLinkID
        self.childLinkID = childLinkID
        self.origin = origin
        self.axis = axis
        self.lowerLimit = lowerLimit
        self.upperLimit = upperLimit
        self.effortLimit = effortLimit
        self.velocityLimit = velocityLimit
        self.softLowerLimit = softLowerLimit
        self.softUpperLimit = softUpperLimit
        self.homePosition = homePosition
        self.damping = damping
        self.coulombFriction = coulombFriction
        self.stiction = stiction
        self.backlash = backlash
        self.compliance = compliance
        self.mimic = mimic
    }
}

public struct JointMimic: Sendable, Codable, Equatable {
    public let jointID: String
    public let multiplier: Double
    public let offset: Double

    public init(jointID: String, multiplier: Double = 1, offset: Double = 0) {
        self.jointID = jointID
        self.multiplier = multiplier
        self.offset = offset
    }
}

public struct ActuatorMount: Sendable, Codable, Equatable {
    public let actuatorID: String
    public let parentLinkID: String
    public let frameID: String
    public let pose: KuyuPose
    public let outputAxis: KuyuVector3
    public let housing: GeometryInstance?
    public let source: String?
    public let notes: String?

    public init(
        actuatorID: String,
        parentLinkID: String,
        frameID: String,
        pose: KuyuPose,
        outputAxis: KuyuVector3,
        housing: GeometryInstance? = nil,
        source: String? = nil,
        notes: String? = nil
    ) {
        self.actuatorID = actuatorID
        self.parentLinkID = parentLinkID
        self.frameID = frameID
        self.pose = pose
        self.outputAxis = outputAxis
        self.housing = housing
        self.source = source
        self.notes = notes
    }
}

public enum TransmissionKind: String, Sendable, Codable, Equatable {
    case direct
    case timingPulley
    case linkage
}

public struct ActuatorAttachment: Sendable, Codable, Equatable {
    public let actuatorID: String
    public let jointID: String
    public let transmissionRatio: Double
    public let torqueLimit: Double
    public let mountFrameID: String?
    public let transmissionKind: TransmissionKind
    public let mechanicalReductionRatio: Double
    public let commandDirection: Double
    public let actuatorZeroOffset: Double
    public let jointZeroOffset: Double
    public let efficiency: Double?
    public let reflectedInertia: Double?

    public init(
        actuatorID: String,
        jointID: String,
        transmissionRatio: Double = 1,
        torqueLimit: Double,
        mountFrameID: String? = nil,
        transmissionKind: TransmissionKind = .direct,
        mechanicalReductionRatio: Double = 1,
        commandDirection: Double = 1,
        actuatorZeroOffset: Double = 0,
        jointZeroOffset: Double = 0,
        efficiency: Double? = nil,
        reflectedInertia: Double? = nil
    ) {
        self.actuatorID = actuatorID
        self.jointID = jointID
        self.transmissionRatio = transmissionRatio
        self.torqueLimit = torqueLimit
        self.mountFrameID = mountFrameID
        self.transmissionKind = transmissionKind
        self.mechanicalReductionRatio = mechanicalReductionRatio
        self.commandDirection = commandDirection
        self.actuatorZeroOffset = actuatorZeroOffset
        self.jointZeroOffset = jointZeroOffset
        self.efficiency = efficiency
        self.reflectedInertia = reflectedInertia
    }
}

public struct SensorMount: Sendable, Codable, Equatable {
    public let sensorID: String
    public let frameID: String
    public let pose: KuyuPose

    public init(sensorID: String, frameID: String, pose: KuyuPose = KuyuPose()) {
        self.sensorID = sensorID
        self.frameID = frameID
        self.pose = pose
    }
}
