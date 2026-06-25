import simd
import KuyuCore

public struct ReferenceQuadrotorParameters: Sendable, Equatable, Codable {
    public enum ValidationError: Error, Equatable {
        case nonFinite(String)
        case nonPositive(String)
    }

    public let mass: Double
    public let inertia: Axis3
    public let armLength: Double
    public let motorTimeConstant: Double
    public let maxThrust: Double
    public let yawCoefficient: Double
    public let gravity: Double
    public let aerodynamics: AerodynamicsParameters

    public init(
        mass: Double,
        inertia: Axis3,
        armLength: Double,
        motorTimeConstant: Double,
        maxThrust: Double,
        yawCoefficient: Double,
        gravity: Double,
        aerodynamics: AerodynamicsParameters = .zero
    ) throws {
        guard mass.isFinite else { throw ValidationError.nonFinite("mass") }
        guard armLength.isFinite else { throw ValidationError.nonFinite("armLength") }
        guard motorTimeConstant.isFinite else { throw ValidationError.nonFinite("motorTimeConstant") }
        guard maxThrust.isFinite else { throw ValidationError.nonFinite("maxThrust") }
        guard yawCoefficient.isFinite else { throw ValidationError.nonFinite("yawCoefficient") }
        guard gravity.isFinite else { throw ValidationError.nonFinite("gravity") }
        guard inertia.x.isFinite, inertia.y.isFinite, inertia.z.isFinite else {
            throw ValidationError.nonFinite("inertia")
        }

        guard mass > 0 else { throw ValidationError.nonPositive("mass") }
        guard armLength > 0 else { throw ValidationError.nonPositive("armLength") }
        guard motorTimeConstant > 0 else { throw ValidationError.nonPositive("motorTimeConstant") }
        guard maxThrust > 0 else { throw ValidationError.nonPositive("maxThrust") }
        guard yawCoefficient >= 0 else { throw ValidationError.nonPositive("yawCoefficient") }
        guard gravity > 0 else { throw ValidationError.nonPositive("gravity") }
        guard inertia.x > 0, inertia.y > 0, inertia.z > 0 else {
            throw ValidationError.nonPositive("inertia")
        }

        self.mass = mass
        self.inertia = inertia
        self.armLength = armLength
        self.motorTimeConstant = motorTimeConstant
        self.maxThrust = maxThrust
        self.yawCoefficient = yawCoefficient
        self.gravity = gravity
        self.aerodynamics = aerodynamics
    }

    public static let baseline = ReferenceQuadrotorParameters(
        uncheckedMass: 1.0,
        uncheckedInertia: Axis3(x: 0.005, y: 0.005, z: 0.009),
        uncheckedArmLength: 0.12,
        uncheckedMotorTimeConstant: 0.030,
        uncheckedMaxThrust: 6.0,
        uncheckedYawCoefficient: 0.020,
        uncheckedGravity: 9.80665,
        uncheckedAerodynamics: .baseline
    )

    public static func reference(
        from inertial: PlantInertialProperties,
        robotID: String
    ) throws -> ReferenceQuadrotorParameters {
        let base = ReferenceQuadrotorParameters.baseline
        let lowered = robotID.lowercased()
        let tunedMaxThrust = lowered.contains("singleprop") ? 12.0 : base.maxThrust
        return try ReferenceQuadrotorParameters(
            mass: inertial.mass,
            inertia: inertial.inertia,
            armLength: base.armLength,
            motorTimeConstant: base.motorTimeConstant,
            maxThrust: tunedMaxThrust,
            yawCoefficient: base.yawCoefficient,
            gravity: base.gravity,
            aerodynamics: base.aerodynamics
        )
    }

    public var inertiaSIMD: SIMD3<Double> {
        SIMD3<Double>(inertia.x, inertia.y, inertia.z)
    }

    private enum CodingKeys: String, CodingKey {
        case mass
        case inertia
        case armLength
        case motorTimeConstant
        case maxThrust
        case yawCoefficient
        case gravity
        case aerodynamics
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            mass: try container.decode(Double.self, forKey: .mass),
            inertia: try container.decode(Axis3.self, forKey: .inertia),
            armLength: try container.decode(Double.self, forKey: .armLength),
            motorTimeConstant: try container.decode(Double.self, forKey: .motorTimeConstant),
            maxThrust: try container.decode(Double.self, forKey: .maxThrust),
            yawCoefficient: try container.decode(Double.self, forKey: .yawCoefficient),
            gravity: try container.decode(Double.self, forKey: .gravity),
            aerodynamics: try container.decode(AerodynamicsParameters.self, forKey: .aerodynamics)
        )
    }

    private init(
        uncheckedMass mass: Double,
        uncheckedInertia inertia: Axis3,
        uncheckedArmLength armLength: Double,
        uncheckedMotorTimeConstant motorTimeConstant: Double,
        uncheckedMaxThrust maxThrust: Double,
        uncheckedYawCoefficient yawCoefficient: Double,
        uncheckedGravity gravity: Double,
        uncheckedAerodynamics aerodynamics: AerodynamicsParameters
    ) {
        self.mass = mass
        self.inertia = inertia
        self.armLength = armLength
        self.motorTimeConstant = motorTimeConstant
        self.maxThrust = maxThrust
        self.yawCoefficient = yawCoefficient
        self.gravity = gravity
        self.aerodynamics = aerodynamics
    }
}
