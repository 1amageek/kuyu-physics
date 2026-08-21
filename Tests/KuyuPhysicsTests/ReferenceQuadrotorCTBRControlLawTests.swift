import KuyuCore
import KuyuPhysics
import Testing

@Test func ctbrCommandSaturatesFinitePolicyOutput() throws {
    let command = try ReferenceQuadrotorCTBRCommand(
        collectiveThrust: 1.2,
        normalizedBodyRate: Axis3(x: -1.3, y: 0.25, z: 2.0)
    )

    #expect(command.collectiveThrust == 1.0)
    #expect(command.normalizedBodyRate == Axis3(x: -1.0, y: 0.25, z: 1.0))
}

@Test func ctbrCommandRejectsNonFinitePolicyOutput() {
    #expect(throws: ReferenceQuadrotorCTBRCommand.ValidationError.nonFinite("collectiveThrust")) {
        _ = try ReferenceQuadrotorCTBRCommand(
            collectiveThrust: .nan,
            normalizedBodyRate: Axis3(x: 0, y: 0, z: 0)
        )
    }
}

@Test func ctbrControlLawMatchesCanonicalRollAllocation() throws {
    let command = try ReferenceQuadrotorCTBRCommand(
        collectiveThrust: 0.5,
        normalizedBodyRate: Axis3(x: 1.0, y: 0.0, z: 0.0)
    )
    let parameters = ReferenceQuadrotorParameters.baseline
    let mixer = ReferenceQuadrotorMixer(
        armLength: parameters.armLength,
        yawCoefficient: parameters.yawCoefficient
    )

    let output = try ReferenceQuadrotorCTBRControlLaw().motorCommand(
        for: command,
        currentAngularVelocity: Axis3(x: 0, y: 0, z: 0),
        parameters: parameters,
        mixer: mixer
    )

    #expect(abs(output.f1 - 0.5) < 1e-12)
    #expect(abs(output.f2 - 0.6458333333333334) < 1e-12)
    #expect(abs(output.f3 - 0.5) < 1e-12)
    #expect(abs(output.f4 - 0.3541666666666667) < 1e-12)
}

@Test func ctbrControlLawProducesCollectiveOnlyWhenRateTargetIsMet() throws {
    let command = try ReferenceQuadrotorCTBRCommand(
        collectiveThrust: 0.4,
        normalizedBodyRate: Axis3(x: 0.5, y: -0.25, z: 0.75)
    )
    let config = ReferenceQuadrotorCTBRControlConfig.canonical
    let currentRate = Axis3(
        x: command.normalizedBodyRate.x * config.bodyRateScale.x,
        y: command.normalizedBodyRate.y * config.bodyRateScale.y,
        z: command.normalizedBodyRate.z * config.bodyRateScale.z
    )
    let parameters = ReferenceQuadrotorParameters.baseline
    let mixer = ReferenceQuadrotorMixer(
        armLength: parameters.armLength,
        yawCoefficient: parameters.yawCoefficient
    )

    let output = try ReferenceQuadrotorCTBRControlLaw(config: config).motorCommand(
        for: command,
        currentAngularVelocity: currentRate,
        parameters: parameters,
        mixer: mixer
    )

    for value in output.values {
        #expect(abs(value - 0.4) < 1e-12)
    }
}

@Test func normalizedMotorNerveUsesCommonRateAndScalingSemantics() throws {
    let maximums = try MotorMaxThrusts.uniform(6.0)
    var nerve = FixedQuadNormalizedMotorNerve(
        config: FixedQuadNormalizedMotorNerve.Config(
            motorMaxThrusts: maximums,
            rateLimitPerSecond: 2.0,
            smoothingTimeConstant: nil
        )
    )
    let zero = try normalizedMotorDrives([0, 0, 0, 0])
    _ = try nerve.update(
        input: zero,
        corrections: [],
        telemetry: emptyQuadTelemetry(),
        time: WorldTime(stepIndex: 0, time: 0.0)
    )

    let output = try nerve.update(
        input: normalizedMotorDrives([1.0, 0.8, 0.6, 0.4]),
        corrections: [],
        telemetry: emptyQuadTelemetry(),
        time: WorldTime(stepIndex: 1, time: 0.1)
    )

    let trace = try #require(nerve.lastTrace)
    expectApproximatelyEqual(trace.uRaw, [1.0, 0.8, 0.6, 0.4])
    expectApproximatelyEqual(trace.uRate, [0.2, 0.2, 0.2, 0.2])
    expectApproximatelyEqual(output.map(\.value), [1.2, 1.2, 1.2, 1.2])
}

@Test func fixedQuadMotorNervePreservesLegacyMixerSemantics() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    var nerve = FixedQuadMotorNerve(
        config: FixedQuadMotorNerve.Config(
            mixer: ReferenceQuadrotorMixer(
                armLength: parameters.armLength,
                yawCoefficient: parameters.yawCoefficient
            ),
            motorMaxThrusts: try MotorMaxThrusts.uniform(parameters.maxThrust),
            rateLimitPerSecond: 1000,
            smoothingTimeConstant: nil
        )
    )

    let output = try nerve.update(
        input: normalizedMotorDrives([0.5, 0.1, -0.2, 0.05]),
        corrections: [],
        telemetry: emptyQuadTelemetry(),
        time: WorldTime(stepIndex: 0, time: 0.0)
    )

    let trace = try #require(nerve.lastTrace)
    expectApproximatelyEqual(trace.uRaw, [0.75, 0.55, 0.35, 0.35])
    expectApproximatelyEqual(output.map(\.value), [4.5, 3.3, 2.1, 2.1])
}

private func expectApproximatelyEqual(
    _ actual: [Double],
    _ expected: [Double],
    tolerance: Double = 1e-12
) {
    #expect(actual.count == expected.count)
    guard actual.count == expected.count else {
        return
    }

    for (actualValue, expectedValue) in zip(actual, expected) {
        #expect(abs(actualValue - expectedValue) <= tolerance)
    }
}

private func normalizedMotorDrives(_ values: [Double]) throws -> [DriveIntent] {
    try values.enumerated().map { index, value in
        try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
    }
}

private func emptyQuadTelemetry() -> MotorNerveTelemetry {
    MotorNerveTelemetry(actuatorTelemetry: ActuatorTelemetrySnapshot(channels: []))
}
