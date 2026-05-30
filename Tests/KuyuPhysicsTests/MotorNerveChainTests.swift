import KuyuCore
import KuyuPhysics
import Testing

@Test func motorNerveChainAppliesActuatorRateLimit() throws {
    var chain = try MotorNerveChain(contract: motorNerveContract(rateLimit: 1.0))
    _ = try chain.update(
        input: [DriveIntent(index: DriveIndex(0), activation: 0.0)],
        corrections: [],
        telemetry: emptyTelemetry(),
        time: WorldTime(stepIndex: 0, time: 0.0)
    )

    let values = try chain.update(
        input: [DriveIntent(index: DriveIndex(0), activation: 10.0)],
        corrections: [],
        telemetry: emptyTelemetry(),
        time: WorldTime(stepIndex: 1, time: 0.5)
    )

    #expect(values.map { $0.value } == [0.5])
    #expect(chain.lastTrace?.uSat == [10.0])
    #expect(chain.lastTrace?.uRate == [0.5])
}

@Test func motorNerveChainRejectsInvalidMixerSpinInsteadOfFallingBack() throws {
    var chain = try MotorNerveChain(contract: mixerContract(spin: "1,-1,nope,1"))

    #expect(throws: MotorNerveChain.ChainError.invalidMixerParameters("mixer")) {
        _ = try chain.update(
            input: [
                DriveIntent(index: DriveIndex(0), activation: 0.5),
                DriveIntent(index: DriveIndex(1), activation: 0.0),
                DriveIntent(index: DriveIndex(2), activation: 0.0),
                DriveIntent(index: DriveIndex(3), activation: 0.0)
            ],
            corrections: [],
            telemetry: emptyTelemetry(),
            time: WorldTime(stepIndex: 0, time: 0.0)
        )
    }
}

private func emptyTelemetry() -> MotorNerveTelemetry {
    MotorNerveTelemetry(actuatorTelemetry: ActuatorTelemetrySnapshot(channels: []))
}

private func motorNerveContract(rateLimit: Double) -> EmbodimentContract {
    EmbodimentContract(
        schemaVersion: "1.0",
        contractID: "motor-nerve-test",
        bodyID: "body",
        signals: SignalCatalog(
            sensor: [
                SignalDefinition(id: "sensor.joint", index: 0, name: "Joint", units: "rad")
            ],
            actuator: [
                SignalDefinition(id: "actuator.joint", index: 0, name: "Actuator", units: "rad")
            ],
            drive: [
                SignalDefinition(id: "drive.joint", index: 0, name: "Drive", units: "rad")
            ],
            reflex: [
                SignalDefinition(id: "reflex.joint", index: 0, name: "Reflex", units: "rad")
            ]
        ),
        sensors: [
            SensorDefinition(
                id: "sensor",
                type: "joint-state",
                channels: ["sensor.joint"],
                rateHz: 100,
                latencySeconds: 0
            )
        ],
        actuators: [
            ActuatorDefinition(
                id: "actuator",
                type: "servo",
                channels: ["actuator.joint"],
                limits: ActuatorLimits(min: -10, max: 10, rateLimitPerSecond: rateLimit)
            )
        ],
        control: ControlContract(
            driveChannels: ["drive.joint"],
            reflexChannels: ["reflex.joint"]
        ),
        motorNerve: MotorNerveContract(stages: [
            MotorNerveStageDefinition(
                id: "direct",
                type: .direct,
                inputs: ["drive.joint"],
                outputs: ["actuator.joint"]
            )
        ])
    )
}

private func mixerContract(spin: String) -> EmbodimentContract {
    let actuatorSignals = (0..<4).map {
        SignalDefinition(id: "actuator.\($0)", index: $0, name: "Actuator \($0)", units: "normalized")
    }
    let driveSignals = (0..<4).map {
        SignalDefinition(id: "drive.\($0)", index: $0, name: "Drive \($0)", units: "normalized")
    }
    return EmbodimentContract(
        schemaVersion: "1.0",
        contractID: "mixer-test",
        bodyID: "body",
        signals: SignalCatalog(
            sensor: [
                SignalDefinition(id: "sensor.state", index: 0, name: "State", units: "1")
            ],
            actuator: actuatorSignals,
            drive: driveSignals,
            reflex: [
                SignalDefinition(id: "reflex.state", index: 0, name: "Reflex", units: "1")
            ]
        ),
        sensors: [
            SensorDefinition(
                id: "sensor",
                type: "state",
                channels: ["sensor.state"],
                rateHz: 100,
                latencySeconds: 0
            )
        ],
        actuators: actuatorSignals.map {
            ActuatorDefinition(
                id: "actuator-\($0.index)",
                type: "motor",
                channels: [$0.id],
                limits: ActuatorLimits(min: 0, max: 1, rateLimitPerSecond: 100)
            )
        },
        control: ControlContract(
            driveChannels: driveSignals.map(\.id),
            reflexChannels: ["reflex.state"]
        ),
        motorNerve: MotorNerveContract(stages: [
            MotorNerveStageDefinition(
                id: "mixer",
                type: .mixer,
                inputs: driveSignals.map(\.id),
                outputs: actuatorSignals.map(\.id),
                parameters: ["spin": spin]
            )
        ])
    )
}
