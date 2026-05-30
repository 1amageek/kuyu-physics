import Foundation
import Testing
import KuyuCore
import KuyuPhysics

@Test func roArmM1EncoderEmitsNeutralWaveshareCommand() async throws {
    let encoder = try RoArmM1ServoCommandEncoder()
    let command = try encoder.command(forRadians: [0.0, 0.0, 0.0, 0.0, 0.0])

    #expect(command.t == 3)
    #expect(command.positions == [2047, 2047, 2047, 2047, 2047])
    #expect(command.speeds == [0, 0, 0, 0, 0])
    #expect(command.accelerations == [60, 60, 60, 60, 60])
}

@Test func roArmM1EncoderMatchesWaveshareJointDirectionTable() async throws {
    let encoder = try RoArmM1ServoCommandEncoder(
        jointLimits: Array(repeating: -1.0...1.0, count: RoArmM1ServoCommandEncoder.jointCount),
        speed: 12,
        acceleration: 34
    )

    let command = try encoder.command(forRadians: [
        Double.pi / 8.0,
        Double.pi / 8.0,
        Double.pi / 8.0,
        Double.pi / 8.0,
        Double.pi / 8.0
    ])

    #expect(command.positions == [1791, 1279, 1791, 2303, 1791])
    #expect(command.speeds == [12, 12, 12, 12, 12])
    #expect(command.accelerations == [34, 34, 34, 34, 34])
}

@Test func roArmM1EncoderRejectsUnsafeTargetsBeforeSerialEncoding() async throws {
    let encoder = try RoArmM1ServoCommandEncoder()

    do {
        _ = try encoder.command(forRadians: [0.0, 0.3, 0.0, 0.0, 0.0])
        #expect(Bool(false))
    } catch let error as RoArmM1ServoCommandEncoder.EncodingError {
        #expect(error == .outOfRange(joint: 2, value: 0.3, min: -0.261799, max: 0.261799))
    }
}

@Test func roArmM1EncoderRejectsInvalidShapeAndNonFiniteValues() async throws {
    let encoder = try RoArmM1ServoCommandEncoder()

    do {
        _ = try encoder.command(forRadians: [0.0, 0.0])
        #expect(Bool(false))
    } catch let error as RoArmM1ServoCommandEncoder.EncodingError {
        #expect(error == .invalidJointCount(expected: 5, actual: 2))
    }

    do {
        _ = try encoder.command(forRadians: [0.0, .nan, 0.0, 0.0, 0.0])
        #expect(Bool(false))
    } catch let error as RoArmM1ServoCommandEncoder.EncodingError {
        #expect(error == .nonFinite("joint[1]"))
    }
}

@Test func roArmM1EncoderAcceptsMotorNerveActuatorValuesInIndexOrder() async throws {
    let encoder = try RoArmM1ServoCommandEncoder()
    let command = try encoder.command(forActuatorValues: [
        try ActuatorValue(index: ActuatorIndex(2), value: 0.0),
        try ActuatorValue(index: ActuatorIndex(0), value: 0.0),
        try ActuatorValue(index: ActuatorIndex(4), value: 0.0),
        try ActuatorValue(index: ActuatorIndex(1), value: 0.0),
        try ActuatorValue(index: ActuatorIndex(3), value: 0.0)
    ])

    #expect(command.positions == [2047, 2047, 2047, 2047, 2047])
}

@Test func roArmM1EncoderProducesCodableJsonPayload() async throws {
    let encoder = try RoArmM1ServoCommandEncoder()
    let data = try encoder.commandData(forRadians: [0.0, 0.0, 0.0, 0.0, 0.0])
    let decoded = try JSONDecoder().decode(RoArmM1ServoCommand.self, from: data)

    #expect(decoded.t == 3)
    #expect(decoded.positions == [2047, 2047, 2047, 2047, 2047])
}
