import Foundation

public struct KuyuWorldModel: Sendable, Codable, Equatable {
    public let schemaVersion: String
    public let worldID: String
    public let time: TimeModel
    public let integrator: IntegratorModel
    public let solver: SolverModel
    public let gravity: GravityModel
    public let atmosphere: AtmosphereModel
    public let wind: WindModel
    public let surfaces: [WorldSurface]
    public let materials: [WorldMaterial]
    public let contact: ContactModel
    public let nap: NegligibilityApproximationPolicy
    public let randomness: RandomnessModel

    public init(
        schemaVersion: String,
        worldID: String,
        time: TimeModel,
        integrator: IntegratorModel,
        solver: SolverModel,
        gravity: GravityModel,
        atmosphere: AtmosphereModel,
        wind: WindModel,
        surfaces: [WorldSurface] = [],
        materials: [WorldMaterial] = [],
        contact: ContactModel,
        nap: NegligibilityApproximationPolicy,
        randomness: RandomnessModel
    ) {
        self.schemaVersion = schemaVersion
        self.worldID = worldID
        self.time = time
        self.integrator = integrator
        self.solver = solver
        self.gravity = gravity
        self.atmosphere = atmosphere
        self.wind = wind
        self.surfaces = surfaces
        self.materials = materials
        self.contact = contact
        self.nap = nap
        self.randomness = randomness
    }
}

public struct TimeModel: Sendable, Codable, Equatable {
    public let fixedStepSeconds: Double
    public let substeps: Int

    public init(fixedStepSeconds: Double, substeps: Int = 1) {
        self.fixedStepSeconds = fixedStepSeconds
        self.substeps = substeps
    }
}

public enum IntegratorKind: String, Sendable, Codable, Equatable {
    case semiImplicitEuler
    case rk4
}

public struct IntegratorModel: Sendable, Codable, Equatable {
    public let kind: IntegratorKind

    public init(kind: IntegratorKind) {
        self.kind = kind
    }
}

public enum SolverKind: String, Sendable, Codable, Equatable {
    case deterministicConstraint
    case disabledContact
}

public struct SolverModel: Sendable, Codable, Equatable {
    public let kind: SolverKind
    public let iterations: Int
    public let tolerance: Double

    public init(kind: SolverKind, iterations: Int, tolerance: Double) {
        self.kind = kind
        self.iterations = iterations
        self.tolerance = tolerance
    }
}

public enum GravityKind: String, Sendable, Codable, Equatable {
    case uniform
    case none
}

public struct GravityModel: Sendable, Codable, Equatable {
    public let kind: GravityKind
    public let acceleration: KuyuVector3

    public init(kind: GravityKind, acceleration: KuyuVector3) {
        self.kind = kind
        self.acceleration = acceleration
    }

    public static let earthUniform = GravityModel(
        kind: .uniform,
        acceleration: KuyuVector3(x: 0, y: 0, z: -9.80665)
    )
}

public enum AtmosphereKind: String, Sendable, Codable, Equatable {
    case none
    case standard
}

public struct AtmosphereModel: Sendable, Codable, Equatable {
    public let kind: AtmosphereKind
    public let airDensity: Double?
    public let temperatureKelvin: Double?

    public init(kind: AtmosphereKind, airDensity: Double? = nil, temperatureKelvin: Double? = nil) {
        self.kind = kind
        self.airDensity = airDensity
        self.temperatureKelvin = temperatureKelvin
    }
}

public enum WindKind: String, Sendable, Codable, Equatable {
    case none
    case constant
}

public struct WindModel: Sendable, Codable, Equatable {
    public let kind: WindKind
    public let velocityWorld: KuyuVector3

    public init(kind: WindKind, velocityWorld: KuyuVector3 = KuyuVector3(x: 0, y: 0, z: 0)) {
        self.kind = kind
        self.velocityWorld = velocityWorld
    }
}

public struct WorldSurface: Sendable, Codable, Equatable {
    public let id: String
    public let frameID: String
    public let materialID: String
    public let pose: KuyuPose
    public let geometry: GeometryInstance

    public init(
        id: String,
        frameID: String,
        materialID: String,
        pose: KuyuPose,
        geometry: GeometryInstance
    ) {
        self.id = id
        self.frameID = frameID
        self.materialID = materialID
        self.pose = pose
        self.geometry = geometry
    }
}

public struct WorldMaterial: Sendable, Codable, Equatable {
    public let id: String
    public let staticFriction: Double
    public let dynamicFriction: Double
    public let restitution: Double

    public init(
        id: String,
        staticFriction: Double,
        dynamicFriction: Double,
        restitution: Double
    ) {
        self.id = id
        self.staticFriction = staticFriction
        self.dynamicFriction = dynamicFriction
        self.restitution = restitution
    }
}

public enum ContactMode: String, Sendable, Codable, Equatable {
    case disabled
    case penalty
    case constraint
}

public struct ContactModel: Sendable, Codable, Equatable {
    public let mode: ContactMode
    public let stiffness: Double?
    public let damping: Double?

    public init(mode: ContactMode, stiffness: Double? = nil, damping: Double? = nil) {
        self.mode = mode
        self.stiffness = stiffness
        self.damping = damping
    }
}

public struct NegligibilityApproximationPolicy: Sendable, Codable, Equatable {
    public let forceAbsoluteThreshold: Double
    public let forceRelativeThreshold: Double
    public let torqueAbsoluteThreshold: Double
    public let torqueRelativeThreshold: Double

    public init(
        forceAbsoluteThreshold: Double,
        forceRelativeThreshold: Double,
        torqueAbsoluteThreshold: Double,
        torqueRelativeThreshold: Double
    ) {
        self.forceAbsoluteThreshold = forceAbsoluteThreshold
        self.forceRelativeThreshold = forceRelativeThreshold
        self.torqueAbsoluteThreshold = torqueAbsoluteThreshold
        self.torqueRelativeThreshold = torqueRelativeThreshold
    }
}

public struct RandomnessModel: Sendable, Codable, Equatable {
    public let seed: UInt64
    public let deterministicReplay: Bool

    public init(seed: UInt64, deterministicReplay: Bool) {
        self.seed = seed
        self.deterministicReplay = deterministicReplay
    }
}
