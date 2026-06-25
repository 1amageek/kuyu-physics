import KuyuCore

public struct MotorMaxThrusts: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case nonFinite(String)
        case negative(String)
    }

    public let f1: Double
    public let f2: Double
    public let f3: Double
    public let f4: Double

    public init(f1: Double, f2: Double, f3: Double, f4: Double) throws {
        guard f1.isFinite else { throw ValidationError.nonFinite("f1") }
        guard f2.isFinite else { throw ValidationError.nonFinite("f2") }
        guard f3.isFinite else { throw ValidationError.nonFinite("f3") }
        guard f4.isFinite else { throw ValidationError.nonFinite("f4") }

        guard f1 >= 0 else { throw ValidationError.negative("f1") }
        guard f2 >= 0 else { throw ValidationError.negative("f2") }
        guard f3 >= 0 else { throw ValidationError.negative("f3") }
        guard f4 >= 0 else { throw ValidationError.negative("f4") }

        self.f1 = f1
        self.f2 = f2
        self.f3 = f3
        self.f4 = f4
    }

    public static func uniform(_ value: Double) throws -> MotorMaxThrusts {
        try MotorMaxThrusts(f1: value, f2: value, f3: value, f4: value)
    }

    static func uncheckedUniform(_ value: Double) -> MotorMaxThrusts {
        MotorMaxThrusts(uncheckedF1: value, uncheckedF2: value, uncheckedF3: value, uncheckedF4: value)
    }

    public func max(forIndex index: UInt32) -> Double {
        switch index {
        case 0: return f1
        case 1: return f2
        case 2: return f3
        case 3: return f4
        default: return 0
        }
    }

    public func setting(index: UInt32, value: Double) throws -> MotorMaxThrusts {
        switch index {
        case 0:
            return try MotorMaxThrusts(f1: value, f2: f2, f3: f3, f4: f4)
        case 1:
            return try MotorMaxThrusts(f1: f1, f2: value, f3: f3, f4: f4)
        case 2:
            return try MotorMaxThrusts(f1: f1, f2: f2, f3: value, f4: f4)
        case 3:
            return try MotorMaxThrusts(f1: f1, f2: f2, f3: f3, f4: value)
        default:
            return self
        }
    }

    private enum CodingKeys: String, CodingKey {
        case f1
        case f2
        case f3
        case f4
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            f1: try container.decode(Double.self, forKey: .f1),
            f2: try container.decode(Double.self, forKey: .f2),
            f3: try container.decode(Double.self, forKey: .f3),
            f4: try container.decode(Double.self, forKey: .f4)
        )
    }

    private init(uncheckedF1 f1: Double, uncheckedF2 f2: Double, uncheckedF3 f3: Double, uncheckedF4 f4: Double) {
        self.f1 = f1
        self.f2 = f2
        self.f3 = f3
        self.f4 = f4
    }
}
