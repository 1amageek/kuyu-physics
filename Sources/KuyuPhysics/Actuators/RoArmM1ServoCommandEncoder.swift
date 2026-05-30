import Foundation
import KuyuCore

public struct RoArmM1ServoCommand: Sendable, Codable, Equatable {
    public enum CommandError: Error, Equatable {
        case invalidFieldCount(field: String, expected: Int, actual: Int)
    }

    public let t: Int
    public let p1: Int
    public let p2: Int
    public let p3: Int
    public let p4: Int
    public let p5: Int
    public let s1: Int
    public let s2: Int
    public let s3: Int
    public let s4: Int
    public let s5: Int
    public let a1: Int
    public let a2: Int
    public let a3: Int
    public let a4: Int
    public let a5: Int

    public init(positions: [Int], speeds: [Int], accelerations: [Int]) throws {
        guard positions.count == RoArmM1ServoCommandEncoder.jointCount else {
            throw CommandError.invalidFieldCount(
                field: "positions",
                expected: RoArmM1ServoCommandEncoder.jointCount,
                actual: positions.count
            )
        }
        guard speeds.count == RoArmM1ServoCommandEncoder.jointCount else {
            throw CommandError.invalidFieldCount(
                field: "speeds",
                expected: RoArmM1ServoCommandEncoder.jointCount,
                actual: speeds.count
            )
        }
        guard accelerations.count == RoArmM1ServoCommandEncoder.jointCount else {
            throw CommandError.invalidFieldCount(
                field: "accelerations",
                expected: RoArmM1ServoCommandEncoder.jointCount,
                actual: accelerations.count
            )
        }
        self.t = 3
        self.p1 = positions[0]
        self.p2 = positions[1]
        self.p3 = positions[2]
        self.p4 = positions[3]
        self.p5 = positions[4]
        self.s1 = speeds[0]
        self.s2 = speeds[1]
        self.s3 = speeds[2]
        self.s4 = speeds[3]
        self.s5 = speeds[4]
        self.a1 = accelerations[0]
        self.a2 = accelerations[1]
        self.a3 = accelerations[2]
        self.a4 = accelerations[3]
        self.a5 = accelerations[4]
    }

    public var positions: [Int] { [p1, p2, p3, p4, p5] }
    public var speeds: [Int] { [s1, s2, s3, s4, s5] }
    public var accelerations: [Int] { [a1, a2, a3, a4, a5] }

    enum CodingKeys: String, CodingKey {
        case t = "T"
        case p1 = "P1"
        case p2 = "P2"
        case p3 = "P3"
        case p4 = "P4"
        case p5 = "P5"
        case s1 = "S1"
        case s2 = "S2"
        case s3 = "S3"
        case s4 = "S4"
        case s5 = "S5"
        case a1 = "A1"
        case a2 = "A2"
        case a3 = "A3"
        case a4 = "A4"
        case a5 = "A5"
    }
}

public struct RoArmM1ServoCommandEncoder: Sendable {
    public enum EncodingError: Error, Equatable {
        case invalidJointCount(expected: Int, actual: Int)
        case invalidActuatorIndex(expected: Int, actual: Int)
        case nonFinite(String)
        case outOfRange(joint: Int, value: Double, min: Double, max: Double)
        case invalidSpeed(Int)
        case invalidAcceleration(Int)
        case pulseOutOfRange(joint: Int, pulse: Int)
    }

    public static let jointCount = 5
    public static let manufacturerJointLimits: [ClosedRange<Double>] = [
        -3.14...3.14,
        -1.0467...1.0467,
        -2.7...2.7,
        -2.1...2.1,
        -1.57...0
    ]
    public static let safeCommissioningJointLimits: [ClosedRange<Double>] = [
        -0.261799...0.261799,
        -0.261799...0.261799,
        -0.261799...0.261799,
        -0.261799...0.261799,
        -0.261799...0
    ]

    private static let wavesharePi = 3.1415926
    private static let servoCenter = 2047
    private static let servoScale = 2048.0
    private static let pulseRange = 0...4095
    public static let commandDirections = [-1.0, -1.0, -1.0, 1.0, -1.0]
    public static let mechanicalReductionRatios = [1.0, 3.0, 1.0, 1.0, 1.0]

    public let jointLimits: [ClosedRange<Double>]
    public let speed: Int
    public let acceleration: Int

    public init(
        jointLimits: [ClosedRange<Double>] = RoArmM1ServoCommandEncoder.safeCommissioningJointLimits,
        speed: Int = 0,
        acceleration: Int = 60
    ) throws {
        guard jointLimits.count == Self.jointCount else {
            throw EncodingError.invalidJointCount(expected: Self.jointCount, actual: jointLimits.count)
        }
        for (index, range) in jointLimits.enumerated() {
            guard range.lowerBound.isFinite, range.upperBound.isFinite else {
                throw EncodingError.nonFinite("jointLimits[\(index)]")
            }
        }
        guard (0...4095).contains(speed) else {
            throw EncodingError.invalidSpeed(speed)
        }
        guard (0...4095).contains(acceleration) else {
            throw EncodingError.invalidAcceleration(acceleration)
        }
        self.jointLimits = jointLimits
        self.speed = speed
        self.acceleration = acceleration
    }

    public func command(forRadians radians: [Double]) throws -> RoArmM1ServoCommand {
        guard radians.count == Self.jointCount else {
            throw EncodingError.invalidJointCount(expected: Self.jointCount, actual: radians.count)
        }

        var positions: [Int] = []
        positions.reserveCapacity(Self.jointCount)
        for (index, value) in radians.enumerated() {
            guard value.isFinite else {
                throw EncodingError.nonFinite("joint[\(index)]")
            }
            let limit = jointLimits[index]
            guard limit.contains(value) else {
                throw EncodingError.outOfRange(
                    joint: index + 1,
                    value: value,
                    min: limit.lowerBound,
                    max: limit.upperBound
                )
            }
            positions.append(try Self.pulse(forRadians: value, joint: index))
        }

        return try RoArmM1ServoCommand(
            positions: positions,
            speeds: Array(repeating: speed, count: Self.jointCount),
            accelerations: Array(repeating: acceleration, count: Self.jointCount)
        )
    }

    public func command(forActuatorValues values: [ActuatorValue]) throws -> RoArmM1ServoCommand {
        guard values.count == Self.jointCount else {
            throw EncodingError.invalidJointCount(expected: Self.jointCount, actual: values.count)
        }

        let sorted = values.sorted { $0.index.rawValue < $1.index.rawValue }
        for (expected, value) in sorted.enumerated() {
            let actual = Int(value.index.rawValue)
            guard actual == expected else {
                throw EncodingError.invalidActuatorIndex(expected: expected, actual: actual)
            }
        }
        return try command(forRadians: sorted.map(\.value))
    }

    public func commandData(forRadians radians: [Double]) throws -> Data {
        let command = try command(forRadians: radians)
        return try Self.jsonData(command)
    }

    public func commandData(forActuatorValues values: [ActuatorValue]) throws -> Data {
        let command = try command(forActuatorValues: values)
        return try Self.jsonData(command)
    }

    public static func pulse(forRadians radians: Double, joint: Int) throws -> Int {
        guard joint >= 0, joint < jointCount else {
            throw EncodingError.invalidActuatorIndex(expected: 0, actual: joint)
        }
        guard radians.isFinite else {
            throw EncodingError.nonFinite("joint[\(joint)]")
        }

        let pulse = Int(
            Double(servoCenter)
            + (
                commandDirections[joint]
                * radians
                / wavesharePi
                * servoScale
                * mechanicalReductionRatios[joint]
            )
            + 0.5
        )
        guard pulseRange.contains(pulse) else {
            throw EncodingError.pulseOutOfRange(joint: joint + 1, pulse: pulse)
        }
        return pulse
    }

    private static func jsonData(_ command: RoArmM1ServoCommand) throws -> Data {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.sortedKeys]
        return try encoder.encode(command)
    }
}
