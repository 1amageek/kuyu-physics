import Testing
import KuyuPhysics

@Test func hardwareParityRejectsMissingHardwareCalibrationReport() throws {
    #expect(throws: KuyuModelValidationError.empty("hardwareParity.report")) {
        _ = try ReadinessGate().validate(
            body: parityBody(),
            world: parityWorld(),
            embodiment: parityEmbodiment(),
            report: parityCompatibilityReport(),
            requiredLevel: .hardwareParity
        )
    }
}

@Test func hardwareCalibrationReportUnlocksHardwareParityWhenMeasuredCoverageIsComplete() throws {
    let body = parityBody()
    let embodiment = parityEmbodiment()
    let level = try ReadinessGate().validate(
        body: body,
        world: parityWorld(),
        embodiment: embodiment,
        report: parityCompatibilityReport(),
        requiredLevel: .hardwareParity,
        hardwareReport: parityHardwareReport(body: body, embodiment: embodiment)
    )

    #expect(level == .hardwareParity)
}

@Test func hardwareParityRejectsUnmeasuredCalibrationSamples() throws {
    let body = parityBody()
    let embodiment = parityEmbodiment()
    let report = HardwareCalibrationReport(
        schemaVersion: "kuyu.hardware-calibration-report.v1",
        reportID: "unmeasured",
        robotID: "fixture-robot",
        bodyID: body.bodyID,
        embodimentContractID: embodiment.contractID,
        readinessLevel: .hardwareParity,
        positionToleranceRadians: 0.05,
        source: HardwareCalibrationSource(measurementSystem: "fixture"),
        jointCalibrations: [
            JointHardwareCalibration(
                jointID: "joint",
                actuatorID: "actuator",
                commandDirection: 1,
                mechanicalReductionRatio: 1,
                identifiedDynamics: parityDynamics(meanError: 0.01, maxError: 0.02),
                samples: [
                    JointCalibrationSample(commandedPositionRadians: 0, commandTimeSeconds: 0),
                    JointCalibrationSample(commandedPositionRadians: 0.1, measuredPositionRadians: 0.1, commandTimeSeconds: 1),
                    JointCalibrationSample(commandedPositionRadians: -0.1, measuredPositionRadians: -0.1, commandTimeSeconds: 2),
                ]
            )
        ]
    )

    #expect(throws: KuyuModelValidationError.unsupportedReadiness(
        "hardwareParity.\(HardwareCalibrationValidationError.missingMeasuredEvidence("jointCalibrations.joint.measuredPositionRadians"))"
    )) {
        _ = try ReadinessGate().validate(
            body: body,
            world: parityWorld(),
            embodiment: embodiment,
            report: parityCompatibilityReport(),
            requiredLevel: .hardwareParity,
            hardwareReport: report
        )
    }
}

@Test func hardwareParityRejectsUntimestampedMeasuredSamples() throws {
    let body = parityBody()
    let embodiment = parityEmbodiment()
    let report = HardwareCalibrationReport(
        schemaVersion: "kuyu.hardware-calibration-report.v1",
        reportID: "untimestamped",
        robotID: "fixture-robot",
        bodyID: body.bodyID,
        embodimentContractID: embodiment.contractID,
        readinessLevel: .hardwareParity,
        positionToleranceRadians: 0.05,
        source: HardwareCalibrationSource(measurementSystem: "fixture"),
        jointCalibrations: [
            JointHardwareCalibration(
                jointID: "joint",
                actuatorID: "actuator",
                commandDirection: 1,
                mechanicalReductionRatio: 1,
                identifiedDynamics: parityDynamics(meanError: 0.01, maxError: 0.02),
                samples: [
                    JointCalibrationSample(
                        commandedPositionRadians: 0,
                        measuredPositionRadians: 0,
                        commandTimeSeconds: 0
                    ),
                    JointCalibrationSample(
                        commandedPositionRadians: 0.1,
                        measuredPositionRadians: 0.1,
                        commandTimeSeconds: 1,
                        observedTimeSeconds: 1.1
                    ),
                    JointCalibrationSample(
                        commandedPositionRadians: -0.1,
                        measuredPositionRadians: -0.1,
                        commandTimeSeconds: 2,
                        observedTimeSeconds: 2.1
                    ),
                ]
            )
        ]
    )

    #expect(throws: KuyuModelValidationError.unsupportedReadiness(
        "hardwareParity.\(HardwareCalibrationValidationError.missingMeasuredEvidence("jointCalibrations.joint.observedTimeSeconds"))"
    )) {
        _ = try ReadinessGate().validate(
            body: body,
            world: parityWorld(),
            embodiment: embodiment,
            report: parityCompatibilityReport(),
            requiredLevel: .hardwareParity,
            hardwareReport: report
        )
    }
}

@Test func hardwareParityAcceptsMeasuredSensorLatencyCoverage() throws {
    let body = sensorParityBody()
    let embodiment = sensorParityEmbodiment()
    let level = try ReadinessGate().validate(
        body: body,
        world: parityWorld(),
        embodiment: embodiment,
        report: sensorParityCompatibilityReport(),
        requiredLevel: .hardwareParity,
        hardwareReport: sensorParityHardwareReport(
            body: body,
            embodiment: embodiment,
            measuredLatencySeconds: 0.01,
            latencyToleranceSeconds: 0.002
        )
    )

    #expect(level == .hardwareParity)
}

@Test func hardwareParityRejectsMissingSensorLatencyCoverage() throws {
    let body = sensorParityBody()
    let embodiment = sensorParityEmbodiment()

    #expect(throws: KuyuModelValidationError.unsupportedReadiness(
        "hardwareParity.\(HardwareCalibrationValidationError.insufficientCoverage("sensorCalibrations.activeSensorCoverage"))"
    )) {
        _ = try ReadinessGate().validate(
            body: body,
            world: parityWorld(),
            embodiment: embodiment,
            report: sensorParityCompatibilityReport(),
            requiredLevel: .hardwareParity,
            hardwareReport: parityHardwareReport(body: body, embodiment: embodiment)
        )
    }
}

@Test func hardwareParityRejectsSensorLatencyOutsideDeclaredTolerance() throws {
    let body = sensorParityBody()
    let embodiment = sensorParityEmbodiment()

    #expect(throws: KuyuModelValidationError.unsupportedReadiness(
        "hardwareParity.\(HardwareCalibrationValidationError.invalidRange("sensorCalibrations.joint-sensor.measuredLatencySeconds"))"
    )) {
        _ = try ReadinessGate().validate(
            body: body,
            world: parityWorld(),
            embodiment: embodiment,
            report: sensorParityCompatibilityReport(),
            requiredLevel: .hardwareParity,
            hardwareReport: sensorParityHardwareReport(
                body: body,
                embodiment: embodiment,
                measuredLatencySeconds: 0.03,
                latencyToleranceSeconds: 0.002
            )
        )
    }
}

private func parityBody() -> KuyuBodyModel {
    KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "parity-body",
        name: "Parity Body",
        category: "manipulator",
        frames: [
            FrameDefinition(id: "actuator-output", parentID: "base", pose: KuyuPose())
        ],
        links: [
            LinkDefinition(
                id: "base",
                mass: 1,
                centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
                inertia: KuyuInertiaTensor(ixx: 0.1, ixy: 0, ixz: 0, iyy: 0.1, iyz: 0, izz: 0.1)
            ),
            LinkDefinition(
                id: "tip",
                mass: 0.5,
                centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
                inertia: KuyuInertiaTensor(ixx: 0.05, ixy: 0, ixz: 0, iyy: 0.05, iyz: 0, izz: 0.05)
            )
        ],
        joints: [
            JointDefinition(
                id: "joint",
                kind: .revolute,
                parentLinkID: "base",
                childLinkID: "tip",
                origin: KuyuPose(),
                axis: KuyuVector3(x: 0, y: 0, z: 1),
                lowerLimit: -1,
                upperLimit: 1,
                effortLimit: 1,
                velocityLimit: 1
            )
        ],
        actuatorMounts: [
            ActuatorMount(
                actuatorID: "actuator",
                parentLinkID: "base",
                frameID: "actuator-output",
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 0, y: 0, z: 1)
            )
        ],
        actuatorAttachments: [
            ActuatorAttachment(
                actuatorID: "actuator",
                jointID: "joint",
                torqueLimit: 1,
                mountFrameID: "actuator-output"
            )
        ]
    )
}

private func sensorParityBody() -> KuyuBodyModel {
    KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "sensor-parity-body",
        name: "Sensor Parity Body",
        category: "manipulator",
        frames: [
            FrameDefinition(id: "actuator-output", parentID: "base", pose: KuyuPose()),
            FrameDefinition(id: "sensor-frame", parentID: "tip", pose: KuyuPose())
        ],
        links: [
            LinkDefinition(
                id: "base",
                mass: 1,
                centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
                inertia: KuyuInertiaTensor(ixx: 0.1, ixy: 0, ixz: 0, iyy: 0.1, iyz: 0, izz: 0.1)
            ),
            LinkDefinition(
                id: "tip",
                mass: 0.5,
                centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
                inertia: KuyuInertiaTensor(ixx: 0.05, ixy: 0, ixz: 0, iyy: 0.05, iyz: 0, izz: 0.05)
            )
        ],
        joints: [
            JointDefinition(
                id: "joint",
                kind: .revolute,
                parentLinkID: "base",
                childLinkID: "tip",
                origin: KuyuPose(),
                axis: KuyuVector3(x: 0, y: 0, z: 1),
                lowerLimit: -1,
                upperLimit: 1,
                effortLimit: 1,
                velocityLimit: 1
            )
        ],
        actuatorMounts: [
            ActuatorMount(
                actuatorID: "actuator",
                parentLinkID: "base",
                frameID: "actuator-output",
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 0, y: 0, z: 1)
            )
        ],
        actuatorAttachments: [
            ActuatorAttachment(
                actuatorID: "actuator",
                jointID: "joint",
                torqueLimit: 1,
                mountFrameID: "actuator-output"
            )
        ],
        sensorMounts: [
            SensorMount(sensorID: "joint-sensor", frameID: "sensor-frame")
        ]
    )
}

private func parityWorld() -> KuyuWorldModel {
    KuyuWorldModel(
        schemaVersion: "kuyu.world.v1",
        worldID: "parity-world",
        time: TimeModel(fixedStepSeconds: 0.01),
        integrator: IntegratorModel(kind: .semiImplicitEuler),
        solver: SolverModel(kind: .disabledContact, iterations: 0, tolerance: 0),
        gravity: .earthUniform,
        atmosphere: AtmosphereModel(kind: .none),
        wind: WindModel(kind: .none),
        contact: ContactModel(mode: .disabled),
        nap: NegligibilityApproximationPolicy(
            forceAbsoluteThreshold: 0,
            forceRelativeThreshold: 0,
            torqueAbsoluteThreshold: 0,
            torqueRelativeThreshold: 0
        ),
        randomness: RandomnessModel(seed: 0, deterministicReplay: true)
    )
}

private func parityEmbodiment() -> EmbodimentContract {
    EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "parity-contract",
        bodyID: "parity-body",
        signals: SignalCatalog(
            sensor: [],
            actuator: [
                SignalDefinition(
                    id: "actuator.joint",
                    index: 0,
                    name: "Joint actuator",
                    units: "rad",
                    range: ScalarRange(min: -1, max: 1)
                )
            ],
            drive: [
                SignalDefinition(id: "drive.joint", index: 0, name: "Joint drive", units: "rad")
            ],
            reflex: [
                SignalDefinition(id: "reflex.joint", index: 0, name: "Joint reflex", units: "rad")
            ]
        ),
        sensors: [],
        actuators: [
            ActuatorDefinition(
                id: "actuator",
                type: "servo",
                frameID: "actuator-output",
                channels: ["actuator.joint"],
                limits: ActuatorLimits(min: -1, max: 1, rateLimitPerSecond: 1),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.1, deadzone: 0, torqueLimit: 1)
            )
        ],
        control: ControlContract(driveChannels: ["drive.joint"], reflexChannels: ["reflex.joint"]),
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

private func sensorParityEmbodiment() -> EmbodimentContract {
    EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "sensor-parity-contract",
        bodyID: "sensor-parity-body",
        signals: SignalCatalog(
            sensor: [
                SignalDefinition(id: "sensor.angle", index: 0, name: "Joint angle", units: "rad")
            ],
            actuator: [
                SignalDefinition(
                    id: "actuator.joint",
                    index: 0,
                    name: "Joint actuator",
                    units: "rad",
                    range: ScalarRange(min: -1, max: 1)
                )
            ],
            drive: [
                SignalDefinition(id: "drive.joint", index: 0, name: "Joint drive", units: "rad")
            ],
            reflex: [
                SignalDefinition(id: "reflex.joint", index: 0, name: "Joint reflex", units: "rad")
            ]
        ),
        sensors: [
            SensorDefinition(
                id: "joint-sensor",
                type: "encoder",
                frameID: "sensor-frame",
                channels: ["sensor.angle"],
                rateHz: 100,
                latencySeconds: 0.01
            )
        ],
        actuators: [
            ActuatorDefinition(
                id: "actuator",
                type: "servo",
                frameID: "actuator-output",
                channels: ["actuator.joint"],
                limits: ActuatorLimits(min: -1, max: 1, rateLimitPerSecond: 1),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.1, deadzone: 0, torqueLimit: 1)
            )
        ],
        control: ControlContract(driveChannels: ["drive.joint"], reflexChannels: ["reflex.joint"]),
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

private func parityCompatibilityReport() -> CompatibilityReport {
    CompatibilityReport(
        schemaVersion: "kuyu.compatibility.v1",
        reportID: "parity-compat",
        sourceFormat: "native",
        targetContract: "kuyu.body.v1",
        mappings: [],
        readinessLevel: .dynamicSimulation
    )
}

private func sensorParityCompatibilityReport() -> CompatibilityReport {
    CompatibilityReport(
        schemaVersion: "kuyu.compatibility.v1",
        reportID: "sensor-parity-compat",
        sourceFormat: "native",
        targetContract: "kuyu.body.v1",
        mappings: [],
        readinessLevel: .dynamicSimulation
    )
}

private func parityHardwareReport(body: KuyuBodyModel, embodiment: EmbodimentContract) -> HardwareCalibrationReport {
    HardwareCalibrationReport(
        schemaVersion: "kuyu.hardware-calibration-report.v1",
        reportID: "measured",
        robotID: "fixture-robot",
        bodyID: body.bodyID,
        embodimentContractID: embodiment.contractID,
        readinessLevel: .hardwareParity,
        positionToleranceRadians: 0.05,
        source: HardwareCalibrationSource(measurementSystem: "fixture"),
        jointCalibrations: [
            JointHardwareCalibration(
                jointID: "joint",
                actuatorID: "actuator",
                commandDirection: 1,
                mechanicalReductionRatio: 1,
                identifiedDynamics: parityDynamics(meanError: 0.01, maxError: 0.02),
                samples: [
                    JointCalibrationSample(
                        commandedPositionRadians: 0,
                        measuredPositionRadians: 0,
                        commandPulse: 2047,
                        commandTimeSeconds: 0,
                        observedTimeSeconds: 0.1
                    ),
                    JointCalibrationSample(
                        commandedPositionRadians: 0.1,
                        measuredPositionRadians: 0.09,
                        commandPulse: 2000,
                        commandTimeSeconds: 1,
                        observedTimeSeconds: 1.1
                    ),
                    JointCalibrationSample(
                        commandedPositionRadians: -0.1,
                        measuredPositionRadians: -0.09,
                        commandPulse: 2100,
                        commandTimeSeconds: 2,
                        observedTimeSeconds: 2.1
                    ),
                ]
            )
        ]
    )
}

private func sensorParityHardwareReport(
    body: KuyuBodyModel,
    embodiment: EmbodimentContract,
    measuredLatencySeconds: Double,
    latencyToleranceSeconds: Double
) -> HardwareCalibrationReport {
    HardwareCalibrationReport(
        schemaVersion: "kuyu.hardware-calibration-report.v1",
        reportID: "sensor-measured",
        robotID: "fixture-robot",
        bodyID: body.bodyID,
        embodimentContractID: embodiment.contractID,
        readinessLevel: .hardwareParity,
        positionToleranceRadians: 0.05,
        source: HardwareCalibrationSource(measurementSystem: "fixture"),
        jointCalibrations: parityHardwareReport(body: body, embodiment: embodiment).jointCalibrations,
        sensorCalibrations: [
            SensorHardwareCalibration(
                sensorID: "joint-sensor",
                channelIDs: ["sensor.angle"],
                measuredLatencySeconds: measuredLatencySeconds,
                latencyToleranceSeconds: latencyToleranceSeconds,
                samples: [
                    SensorCalibrationSample(
                        channelID: "sensor.angle",
                        expectedValue: 0,
                        measuredValue: 0,
                        stimulusTimeSeconds: 0,
                        observedTimeSeconds: 0.01
                    ),
                    SensorCalibrationSample(
                        channelID: "sensor.angle",
                        expectedValue: 0.1,
                        measuredValue: 0.1,
                        stimulusTimeSeconds: 1,
                        observedTimeSeconds: 1.01
                    ),
                    SensorCalibrationSample(
                        channelID: "sensor.angle",
                        expectedValue: -0.1,
                        measuredValue: -0.1,
                        stimulusTimeSeconds: 2,
                        observedTimeSeconds: 2.01
                    ),
                ]
            )
        ]
    )
}

private func parityDynamics(meanError: Double, maxError: Double) -> IdentifiedJointDynamics {
    IdentifiedJointDynamics(
        latencySeconds: 0.02,
        timeConstantSeconds: 0.1,
        deadbandRadians: 0.001,
        backlashRadians: 0.002,
        viscousDamping: 0.01,
        coulombFriction: 0.01,
        meanAbsoluteErrorRadians: meanError,
        maxObservedErrorRadians: maxError
    )
}
