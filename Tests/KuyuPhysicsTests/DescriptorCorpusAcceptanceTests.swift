import EmbodimentContract
import Foundation
import KuyuCore
import KuyuPhysics
import Testing

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceRecordsHardwareParityGap() async throws {
    let fixture = roArmLikeDescriptorCorpusFixture()
    let hardwareReport = try roArmLikeHardwareReport(
        body: fixture.body,
        embodiment: fixture.embodiment,
        readinessLevel: .dynamicSimulation
    )
    let entry = DescriptorCorpusEntry(
        entryID: "roarm-m1-dynamic-gap",
        robotID: "roarm-m1-v0",
        label: "RoArm M1 descriptor corpus dynamic acceptance",
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        hardwareReport: hardwareReport,
        requiredReadiness: .dynamicSimulation,
        duration: 0.05,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(6),
        determinism: try DeterminismConfig(tier: .tier0)
    )

    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "real-robot-descriptor-corpus-smoke",
        entries: [entry]
    )
    let record = try #require(summary.records.first)

    #expect(summary.accepted)
    #expect(record.accepted)
    #expect(record.achievedReadiness == .dynamicSimulation)
    #expect(record.replay.passed)
    #expect(record.replay.sortedJSONByteStable)
    #expect(record.replay.stepCount == 10)
    #expect(summary.hardwareParityGaps.count == 1)
    #expect(summary.hardwareParityGaps.first?.readinessLevel == .hardwareParity)
    let evidence = try #require(record.hardwareEvidence)
    let expectedReportHash = try ConfigHash.hash(hardwareReport)
    #expect(evidence.reportID == hardwareReport.reportID)
    #expect(evidence.readinessLevel == .dynamicSimulation)
    #expect(evidence.reportHash == expectedReportHash)
    #expect(evidence.jointCalibrationCount == fixture.jointIDs.count)
    #expect(evidence.measuredJointSampleCount == fixture.jointIDs.count * 3)
    #expect(evidence.observedJointSampleCount == fixture.jointIDs.count * 3)
    #expect(evidence.sensorCalibrationCount == 0)
    #expect(evidence.sensorSampleCount == 0)
    #expect(evidence.observedSensorSampleCount == 0)
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceAcceptsHardwareParityReport() async throws {
    let fixture = roArmLikeDescriptorCorpusFixture()
    let hardwareReport = try roArmLikeHardwareReport(
        body: fixture.body,
        embodiment: fixture.embodiment,
        readinessLevel: .hardwareParity
    )
    let entry = DescriptorCorpusEntry(
        entryID: "roarm-m1-hardware-parity",
        robotID: "roarm-m1-v0",
        label: "RoArm M1 descriptor corpus hardware parity acceptance",
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        hardwareReport: hardwareReport,
        requiredReadiness: .hardwareParity,
        duration: 0.05,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(7),
        determinism: try DeterminismConfig(tier: .tier0)
    )

    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "real-robot-descriptor-corpus-hardware-parity",
        entries: [entry]
    )
    let record = try #require(summary.records.first)

    #expect(summary.accepted)
    #expect(record.achievedReadiness == .hardwareParity)
    #expect(record.hardwareParity == .accepted)
    #expect(summary.hardwareParityGaps.isEmpty)
    #expect(record.replay.stepCount == 10)
    let evidence = try #require(record.hardwareEvidence)
    let expectedReportHash = try ConfigHash.hash(hardwareReport)
    #expect(evidence.reportID == hardwareReport.reportID)
    #expect(evidence.robotID == record.robotID)
    #expect(evidence.bodyID == record.bodyID)
    #expect(evidence.embodimentContractID == record.embodimentContractID)
    #expect(evidence.readinessLevel == .hardwareParity)
    #expect(evidence.measurementSystem == "bench-fixture")
    #expect(evidence.deviceID == "roarm-m1-like-bench")
    #expect(evidence.reportHash == expectedReportHash)
    #expect(evidence.jointCalibrationCount == fixture.jointIDs.count)
    #expect(evidence.jointSampleCount == fixture.jointIDs.count * 3)
    #expect(evidence.measuredJointSampleCount == fixture.jointIDs.count * 3)
    #expect(evidence.observedJointSampleCount == fixture.jointIDs.count * 3)
    #expect(evidence.sensorCalibrationCount == 0)
    #expect(evidence.sensorSampleCount == 0)
    #expect(evidence.observedSensorSampleCount == 0)
    #expect(evidence.contactCalibrationCount == 0)
    #expect(evidence.contactSampleCount == 0)
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceArtifactStoreRejectsHardwareParityWithoutEvidence() async throws {
    let fixture = roArmLikeDescriptorCorpusFixture()
    let hardwareReport = try roArmLikeHardwareReport(
        body: fixture.body,
        embodiment: fixture.embodiment,
        readinessLevel: .hardwareParity
    )
    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "tampered-hardware-parity-descriptor-corpus",
        entries: [
            DescriptorCorpusEntry(
                entryID: "roarm-m1-hardware-parity",
                robotID: "roarm-m1-v0",
                label: "RoArm M1 descriptor corpus hardware parity acceptance",
                body: fixture.body,
                world: fixture.world,
                embodiment: fixture.embodiment,
                hardwareReport: hardwareReport,
                requiredReadiness: .hardwareParity,
                duration: 0.05,
                timeStep: try TimeStep(delta: fixture.timeStep),
                seed: ScenarioSeed(17),
                determinism: try DeterminismConfig(tier: .tier0)
            )
        ]
    )
    let record = try #require(summary.records.first)
    let tamperedRecord = DescriptorCorpusAcceptanceRecord(
        entryID: record.entryID,
        robotID: record.robotID,
        label: record.label,
        bodyID: record.bodyID,
        worldID: record.worldID,
        embodimentContractID: record.embodimentContractID,
        requiredReadiness: record.requiredReadiness,
        achievedReadiness: record.achievedReadiness,
        hardwareParity: record.hardwareParity,
        hardwareEvidence: nil,
        readinessGaps: record.readinessGaps,
        replay: record.replay
    )
    let tamperedSummary = DescriptorCorpusAcceptanceSummary(
        corpusID: summary.corpusID,
        generatedAt: summary.generatedAt,
        records: [tamperedRecord]
    )

    #expect(throws: DescriptorCorpusAcceptanceArtifactStore.StoreError.invalidSummary(
        "records.\(record.entryID).hardwareParity.evidence"
    )) {
        try DescriptorCorpusAcceptanceArtifactStore().validate(tamperedSummary)
    }
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceArtifactStoreRejectsHardwareParityWithoutObservedSampleEvidence() async throws {
    let fixture = roArmLikeDescriptorCorpusFixture()
    let hardwareReport = try roArmLikeHardwareReport(
        body: fixture.body,
        embodiment: fixture.embodiment,
        readinessLevel: .hardwareParity
    )
    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "tampered-hardware-parity-observed-samples",
        entries: [
            DescriptorCorpusEntry(
                entryID: "roarm-m1-hardware-parity",
                robotID: "roarm-m1-v0",
                label: "RoArm M1 descriptor corpus hardware parity acceptance",
                body: fixture.body,
                world: fixture.world,
                embodiment: fixture.embodiment,
                hardwareReport: hardwareReport,
                requiredReadiness: .hardwareParity,
                duration: 0.05,
                timeStep: try TimeStep(delta: fixture.timeStep),
                seed: ScenarioSeed(18),
                determinism: try DeterminismConfig(tier: .tier0)
            )
        ]
    )
    let record = try #require(summary.records.first)
    let evidence = try #require(record.hardwareEvidence)
    let tamperedEvidence = DescriptorCorpusHardwareEvidence(
        reportID: evidence.reportID,
        robotID: evidence.robotID,
        bodyID: evidence.bodyID,
        embodimentContractID: evidence.embodimentContractID,
        reportHash: evidence.reportHash,
        readinessLevel: evidence.readinessLevel,
        measurementSystem: evidence.measurementSystem,
        deviceID: evidence.deviceID,
        operatorID: evidence.operatorID,
        jointCalibrationCount: evidence.jointCalibrationCount,
        jointSampleCount: evidence.jointSampleCount,
        measuredJointSampleCount: evidence.measuredJointSampleCount,
        observedJointSampleCount: evidence.jointSampleCount - 1,
        contactCalibrationCount: evidence.contactCalibrationCount,
        contactSampleCount: evidence.contactSampleCount
    )
    let tamperedRecord = DescriptorCorpusAcceptanceRecord(
        entryID: record.entryID,
        robotID: record.robotID,
        label: record.label,
        bodyID: record.bodyID,
        worldID: record.worldID,
        embodimentContractID: record.embodimentContractID,
        requiredReadiness: record.requiredReadiness,
        achievedReadiness: record.achievedReadiness,
        hardwareParity: record.hardwareParity,
        hardwareEvidence: tamperedEvidence,
        readinessGaps: record.readinessGaps,
        replay: record.replay
    )
    let tamperedSummary = DescriptorCorpusAcceptanceSummary(
        corpusID: summary.corpusID,
        generatedAt: summary.generatedAt,
        records: [tamperedRecord]
    )

    #expect(throws: DescriptorCorpusAcceptanceArtifactStore.StoreError.invalidSummary(
        "records.\(record.entryID).hardwareParity.evidence.observedJointSampleCount"
    )) {
        try DescriptorCorpusAcceptanceArtifactStore().validate(tamperedSummary)
    }
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceArtifactStoreRejectsHardwareParityWithoutObservedSensorEvidence() async throws {
    let fixture = roArmLikeDescriptorCorpusFixture()
    let hardwareReport = try roArmLikeHardwareReport(
        body: fixture.body,
        embodiment: fixture.embodiment,
        readinessLevel: .hardwareParity
    )
    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "tampered-hardware-parity-observed-sensor-samples",
        entries: [
            DescriptorCorpusEntry(
                entryID: "roarm-m1-hardware-parity",
                robotID: "roarm-m1-v0",
                label: "RoArm M1 descriptor corpus hardware parity acceptance",
                body: fixture.body,
                world: fixture.world,
                embodiment: fixture.embodiment,
                hardwareReport: hardwareReport,
                requiredReadiness: .hardwareParity,
                duration: 0.05,
                timeStep: try TimeStep(delta: fixture.timeStep),
                seed: ScenarioSeed(19),
                determinism: try DeterminismConfig(tier: .tier0)
            )
        ]
    )
    let record = try #require(summary.records.first)
    let evidence = try #require(record.hardwareEvidence)
    let tamperedEvidence = DescriptorCorpusHardwareEvidence(
        reportID: evidence.reportID,
        robotID: evidence.robotID,
        bodyID: evidence.bodyID,
        embodimentContractID: evidence.embodimentContractID,
        reportHash: evidence.reportHash,
        readinessLevel: evidence.readinessLevel,
        measurementSystem: evidence.measurementSystem,
        deviceID: evidence.deviceID,
        operatorID: evidence.operatorID,
        jointCalibrationCount: evidence.jointCalibrationCount,
        jointSampleCount: evidence.jointSampleCount,
        measuredJointSampleCount: evidence.measuredJointSampleCount,
        observedJointSampleCount: evidence.observedJointSampleCount,
        sensorCalibrationCount: 1,
        sensorSampleCount: 3,
        observedSensorSampleCount: 2,
        contactCalibrationCount: evidence.contactCalibrationCount,
        contactSampleCount: evidence.contactSampleCount
    )
    let tamperedRecord = DescriptorCorpusAcceptanceRecord(
        entryID: record.entryID,
        robotID: record.robotID,
        label: record.label,
        bodyID: record.bodyID,
        worldID: record.worldID,
        embodimentContractID: record.embodimentContractID,
        requiredReadiness: record.requiredReadiness,
        achievedReadiness: record.achievedReadiness,
        hardwareParity: record.hardwareParity,
        hardwareEvidence: tamperedEvidence,
        readinessGaps: record.readinessGaps,
        replay: record.replay
    )
    let tamperedSummary = DescriptorCorpusAcceptanceSummary(
        corpusID: summary.corpusID,
        generatedAt: summary.generatedAt,
        records: [tamperedRecord]
    )

    #expect(throws: DescriptorCorpusAcceptanceArtifactStore.StoreError.invalidSummary(
        "records.\(record.entryID).hardwareParity.evidence.observedSensorSampleCount"
    )) {
        try DescriptorCorpusAcceptanceArtifactStore().validate(tamperedSummary)
    }
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceReplaysRigidActuatorDescriptors() async throws {
    let fixtures = [
        rigidAerialDescriptorCorpusFixture(robotID: "singleprop-v0", bodyID: "singleprop-body-v0", actuatorCount: 1),
        rigidAerialDescriptorCorpusFixture(robotID: "quadref-v0", bodyID: "quadref-body-v0", actuatorCount: 4)
    ]
    let entries = try fixtures.enumerated().map { index, fixture in
        DescriptorCorpusEntry(
            entryID: "rigid-actuator-\(fixture.robotID)",
            robotID: fixture.robotID,
            label: "Rigid actuator descriptor \(fixture.robotID)",
            body: fixture.body,
            world: fixture.world,
            embodiment: fixture.embodiment,
            requiredReadiness: .dynamicSimulation,
            duration: 0.04,
            timeStep: try TimeStep(delta: fixture.timeStep),
            seed: ScenarioSeed(UInt64(index + 10)),
            determinism: try DeterminismConfig(tier: .tier0)
        )
    }

    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "rigid-actuator-descriptor-corpus",
        entries: entries
    )

    #expect(summary.accepted)
    #expect(summary.records.map(\.robotID).sorted() == ["quadref-v0", "singleprop-v0"])
    #expect(summary.records.allSatisfy { $0.achievedReadiness == .dynamicSimulation })
    #expect(summary.records.allSatisfy { $0.hardwareParity == .notRequested })
    #expect(summary.records.allSatisfy { $0.hardwareEvidence == nil })
    #expect(summary.records.allSatisfy { $0.replay.passed })
    #expect(summary.records.allSatisfy { $0.replay.sortedJSONByteStable })
    #expect(summary.records.allSatisfy { $0.replay.stepCount == 4 })
    #expect(summary.hardwareParityGaps.isEmpty)
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptancePersistsContactMaterialTrainingVariant() async throws {
    let fixture = contactDescriptorCorpusFixture()
    let entry = DescriptorCorpusEntry(
        entryID: "contact-material-prismatic-foot-v0",
        robotID: "contact-material-foot-v0",
        label: "Contact material descriptor corpus variant",
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        requiredReadiness: .contactTraining,
        duration: 0.05,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(14),
        determinism: try DeterminismConfig(tier: .tier0)
    )

    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "contact-material-descriptor-corpus",
        entries: [entry],
        generatedAt: "2026-06-30T00:00:00Z"
    )
    let record = try #require(summary.records.first)
    let directory = temporaryDescriptorCorpusDirectory()
    defer {
        do {
            try FileManager.default.removeItem(at: directory)
        } catch {
            Issue.record("Temporary descriptor corpus directory cleanup failed: \(error)")
        }
    }

    let url = try DescriptorCorpusAcceptanceArtifactStore().write(summary, to: directory)
    let reloaded = try DescriptorCorpusAcceptanceArtifactStore().validatedSummary(at: url)

    #expect(summary.accepted)
    #expect(record.accepted)
    #expect(record.requiredReadiness == .contactTraining)
    #expect(record.achievedReadiness == .contactTraining)
    #expect(record.hardwareParity == .notRequested)
    #expect(record.hardwareEvidence == nil)
    #expect(record.readinessGaps.isEmpty)
    #expect(record.replay.passed)
    #expect(record.replay.sortedJSONByteStable)
    #expect(record.replay.stepCount == 10)
    let contact = try #require(record.replay.contact)
    #expect(contact.maxActiveContactCount > 0)
    #expect(contact.maxPenetration <= 1e-9)
    #expect(contact.maxSolverIterations > 0)
    #expect(reloaded == summary)
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceRejectsContactTrainingWithoutBodyMaterialCoefficients() async throws {
    let fixture = contactDescriptorCorpusFixture(
        bodyMaterial: BodyMaterial(id: "rubber", density: 1_000, dynamicFriction: 0.6, restitution: 0)
    )
    let entry = DescriptorCorpusEntry(
        entryID: "contact-material-missing-static-friction-v0",
        robotID: "contact-material-foot-v0",
        label: "Contact material descriptor with missing static friction",
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        requiredReadiness: .contactTraining,
        duration: 0.05,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(15),
        determinism: try DeterminismConfig(tier: .tier0)
    )

    do {
        _ = try await DescriptorCorpusAcceptanceService().accept(
            corpusID: "invalid-contact-material-descriptor-corpus",
            entries: [entry]
        )
        Issue.record("Expected contact training corpus acceptance to reject missing material coefficients.")
    } catch DescriptorCorpusAcceptanceError.readinessFailed(let entryID, let reason) {
        #expect(entryID == entry.entryID)
        #expect(reason.contains("contactTraining.body.materials.rubber.staticFriction"))
    }
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceArtifactStoreRejectsMissingContactReplayEvidence() async throws {
    let fixture = contactDescriptorCorpusFixture()
    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "tampered-contact-material-descriptor-corpus",
        entries: [
            DescriptorCorpusEntry(
                entryID: "contact-material-prismatic-foot-v0",
                robotID: "contact-material-foot-v0",
                label: "Contact material descriptor corpus variant",
                body: fixture.body,
                world: fixture.world,
                embodiment: fixture.embodiment,
                requiredReadiness: .contactTraining,
                duration: 0.05,
                timeStep: try TimeStep(delta: fixture.timeStep),
                seed: ScenarioSeed(16),
                determinism: try DeterminismConfig(tier: .tier0)
            )
        ]
    )
    let record = try #require(summary.records.first)
    let tamperedReplay = DescriptorCorpusReplayEvidence(
        passed: record.replay.passed,
        tier: record.replay.tier,
        stepCount: record.replay.stepCount,
        configHash: record.replay.configHash,
        sortedJSONByteStable: record.replay.sortedJSONByteStable,
        issues: record.replay.issues,
        residuals: record.replay.residuals,
        contact: nil
    )
    let tamperedRecord = DescriptorCorpusAcceptanceRecord(
        entryID: record.entryID,
        robotID: record.robotID,
        label: record.label,
        bodyID: record.bodyID,
        worldID: record.worldID,
        embodimentContractID: record.embodimentContractID,
        requiredReadiness: record.requiredReadiness,
        achievedReadiness: record.achievedReadiness,
        hardwareParity: record.hardwareParity,
        hardwareEvidence: record.hardwareEvidence,
        readinessGaps: record.readinessGaps,
        replay: tamperedReplay
    )
    let tamperedSummary = DescriptorCorpusAcceptanceSummary(
        corpusID: summary.corpusID,
        generatedAt: summary.generatedAt,
        records: [tamperedRecord]
    )

    #expect(throws: DescriptorCorpusAcceptanceArtifactStore.StoreError.invalidSummary(
        "records.\(record.entryID).replay.contact"
    )) {
        try DescriptorCorpusAcceptanceArtifactStore().validate(tamperedSummary)
    }
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceArtifactStoreRoundTripsValidatedSummary() async throws {
    let fixture = rigidAerialDescriptorCorpusFixture(
        robotID: "singleprop-v0",
        bodyID: "singleprop-body-v0",
        actuatorCount: 1
    )
    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "rigid-actuator-artifact-corpus",
        entries: [
            DescriptorCorpusEntry(
                entryID: "rigid-actuator-singleprop-v0",
                robotID: fixture.robotID,
                label: "Rigid actuator descriptor \(fixture.robotID)",
                body: fixture.body,
                world: fixture.world,
                embodiment: fixture.embodiment,
                requiredReadiness: .dynamicSimulation,
                duration: 0.04,
                timeStep: try TimeStep(delta: fixture.timeStep),
                seed: ScenarioSeed(11),
                determinism: try DeterminismConfig(tier: .tier0)
            )
        ],
        generatedAt: "2026-06-30T00:00:00Z"
    )
    let directory = temporaryDescriptorCorpusDirectory()
    defer {
        do {
            try FileManager.default.removeItem(at: directory)
        } catch {
            Issue.record("Temporary descriptor corpus directory cleanup failed: \(error)")
        }
    }

    let url = try DescriptorCorpusAcceptanceArtifactStore().write(summary, to: directory)
    let reloaded = try DescriptorCorpusAcceptanceArtifactStore().validatedSummary(at: url)
    let reloadedFromDirectory = try DescriptorCorpusAcceptanceArtifactStore().validatedSummary(in: directory)

    #expect(url.lastPathComponent == DescriptorCorpusAcceptanceArtifactStore.fileName)
    #expect(reloaded == summary)
    #expect(reloadedFromDirectory == summary)
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceArtifactStoreRejectsTamperedReplay() async throws {
    let fixture = rigidAerialDescriptorCorpusFixture(
        robotID: "singleprop-v0",
        bodyID: "singleprop-body-v0",
        actuatorCount: 1
    )
    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "tampered-rigid-actuator-artifact-corpus",
        entries: [
            DescriptorCorpusEntry(
                entryID: "rigid-actuator-singleprop-v0",
                robotID: fixture.robotID,
                label: "Rigid actuator descriptor \(fixture.robotID)",
                body: fixture.body,
                world: fixture.world,
                embodiment: fixture.embodiment,
                requiredReadiness: .dynamicSimulation,
                duration: 0.04,
                timeStep: try TimeStep(delta: fixture.timeStep),
                seed: ScenarioSeed(13),
                determinism: try DeterminismConfig(tier: .tier0)
            )
        ]
    )
    let record = try #require(summary.records.first)
    let tamperedReplay = DescriptorCorpusReplayEvidence(
        passed: false,
        tier: record.replay.tier,
        stepCount: record.replay.stepCount,
        configHash: record.replay.configHash,
        sortedJSONByteStable: record.replay.sortedJSONByteStable,
        issues: ["tampered"],
        residuals: record.replay.residuals
    )
    let tamperedRecord = DescriptorCorpusAcceptanceRecord(
        entryID: record.entryID,
        robotID: record.robotID,
        label: record.label,
        bodyID: record.bodyID,
        worldID: record.worldID,
        embodimentContractID: record.embodimentContractID,
        requiredReadiness: record.requiredReadiness,
        achievedReadiness: record.achievedReadiness,
        hardwareParity: record.hardwareParity,
        hardwareEvidence: record.hardwareEvidence,
        readinessGaps: record.readinessGaps,
        replay: tamperedReplay
    )
    let tamperedSummary = DescriptorCorpusAcceptanceSummary(
        corpusID: summary.corpusID,
        generatedAt: summary.generatedAt,
        records: [tamperedRecord]
    )

    #expect(throws: DescriptorCorpusAcceptanceArtifactStore.StoreError.invalidSummary(
        "records.\(record.entryID).accepted"
    )) {
        try DescriptorCorpusAcceptanceArtifactStore().validate(tamperedSummary)
    }
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceRejectsUnsupportedRigidActuatorChannelCount() async throws {
    let fixture = rigidAerialDescriptorCorpusFixture(
        robotID: "dualprop-v0",
        bodyID: "dualprop-body-v0",
        actuatorCount: 2
    )
    let entry = DescriptorCorpusEntry(
        entryID: "rigid-actuator-dualprop-v0",
        robotID: fixture.robotID,
        label: "Unsupported dual rigid actuator descriptor",
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        requiredReadiness: .dynamicSimulation,
        duration: 0.04,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(12),
        determinism: try DeterminismConfig(tier: .tier0)
    )

    do {
        _ = try await DescriptorCorpusAcceptanceService().accept(
            corpusID: "unsupported-rigid-actuator-descriptor-corpus",
            entries: [entry]
        )
        Issue.record("Expected unsupported rigid actuator channel count to fail readiness.")
    } catch DescriptorCorpusAcceptanceError.readinessFailed(let entryID, let reason) {
        #expect(entryID == entry.entryID)
        #expect(reason.contains("readiness.dynamic.rigid.channelCount"))
    }
}

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptanceRejectsDuplicateEntries() async throws {
    let fixture = roArmLikeDescriptorCorpusFixture()
    let entry = DescriptorCorpusEntry(
        entryID: "duplicate-entry",
        robotID: "roarm-m1-v0",
        label: "Duplicate descriptor entry",
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        requiredReadiness: .dynamicSimulation,
        duration: 0.05,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(8),
        determinism: try DeterminismConfig(tier: .tier0)
    )

    await #expect(throws: DescriptorCorpusAcceptanceError.duplicateEntryID("duplicate-entry")) {
        _ = try await DescriptorCorpusAcceptanceService().accept(
            corpusID: "duplicate-corpus",
            entries: [entry, entry]
        )
    }
}

@Test(.timeLimit(.minutes(1))) func loadedRobotDescriptorCorpusAcceptanceServiceWritesValidatedArtifact() async throws {
    let fixture = roArmLikeDescriptorCorpusFixture()
    let loadedRobot = loadedDescriptorCorpusRobot(fixture: fixture)
    let directory = temporaryDescriptorCorpusDirectory()
    defer {
        do {
            try FileManager.default.removeItem(at: directory)
        } catch {
            Issue.record("Temporary descriptor corpus directory cleanup failed: \(error)")
        }
    }

    let publication = try await LoadedRobotDescriptorCorpusAcceptanceService().write(
        LoadedRobotDescriptorCorpusAcceptanceRequest(
            corpusID: "loaded-robot-descriptor-corpus",
            loadedRobots: [loadedRobot],
            outputDirectory: directory,
            generatedAt: "2026-07-03T00:00:00Z",
            requiredReadiness: .dynamicSimulation,
            durationSteps: 4,
            seedBase: 31
        )
    )
    let reloaded = try DescriptorCorpusAcceptanceArtifactStore().validatedSummary(
        at: publication.artifactURL
    )
    let record = try #require(publication.summary.records.first)

    #expect(publication.artifactURL.lastPathComponent == DescriptorCorpusAcceptanceArtifactStore.fileName)
    #expect(publication.summary.accepted)
    #expect(record.entryID == "roarm-m1-like-v0-dynamicSimulation")
    #expect(record.robotID == loadedRobot.manifest.robotID)
    #expect(record.bodyID == fixture.body.bodyID)
    #expect(record.worldID == fixture.world.worldID)
    #expect(record.embodimentContractID == fixture.embodiment.contractID)
    #expect(record.requiredReadiness == .dynamicSimulation)
    #expect(record.achievedReadiness == .dynamicSimulation)
    #expect(record.hardwareParity == .notRequested)
    #expect(record.hardwareEvidence == nil)
    #expect(record.replay.passed)
    #expect(record.replay.stepCount == 4)
    #expect(reloaded == publication.summary)
}

@Test(.timeLimit(.minutes(1))) func loadedRobotDescriptorCorpusAcceptanceServiceRejectsEmptyRobots() async throws {
    await #expect(throws: LoadedRobotDescriptorCorpusAcceptanceError.emptyRobots) {
        _ = try await LoadedRobotDescriptorCorpusAcceptanceService().write(
            LoadedRobotDescriptorCorpusAcceptanceRequest(
                corpusID: "empty-loaded-robot-descriptor-corpus",
                loadedRobots: [],
                outputDirectory: temporaryDescriptorCorpusDirectory()
            )
        )
    }
}

@Test(.timeLimit(.minutes(1))) func loadedRobotDescriptorCorpusAcceptanceServiceRejectsSymlinkedOutputEscape() async throws {
    let root = temporaryDescriptorCorpusDirectory()
    let external = temporaryDescriptorCorpusDirectory()
    defer {
        for directory in [root, external] {
            do {
                try FileManager.default.removeItem(at: directory)
            } catch {
                Issue.record("Temporary descriptor corpus directory cleanup failed: \(error)")
            }
        }
    }
    try FileManager.default.createDirectory(at: root, withIntermediateDirectories: true)
    try FileManager.default.createDirectory(at: external, withIntermediateDirectories: true)
    let escapedOutput = root.appendingPathComponent("physics", isDirectory: true)
    try FileManager.default.createSymbolicLink(at: escapedOutput, withDestinationURL: external)
    let expectedEscapedPath = external.standardizedFileURL.resolvingSymlinksInPath().path

    await #expect(throws: LoadedRobotDescriptorCorpusAcceptanceError.outputEscapesArtifactRoot(
        expectedEscapedPath
    )) {
        _ = try await LoadedRobotDescriptorCorpusAcceptanceService().write(
            LoadedRobotDescriptorCorpusAcceptanceRequest(
                corpusID: "escaped-loaded-robot-descriptor-corpus",
                loadedRobots: [loadedDescriptorCorpusRobot(fixture: roArmLikeDescriptorCorpusFixture())],
                outputDirectory: escapedOutput,
                artifactRoot: root
            )
        )
    }
}

@Test func referenceQuadrotorEmbodimentResolverUsesDescriptorActuatorDynamics() throws {
    let fixture = rigidAerialDescriptorCorpusFixture(
        robotID: "quadref-v0",
        bodyID: "quadref-body-v0",
        actuatorCount: 4
    )
    let manifest = KuyuRobotManifest(
        schemaVersion: "kuyu.robot.v1",
        robotID: fixture.robotID,
        name: "QuadRef",
        category: "aerial",
        bodyModel: ModelReference(path: "body.json"),
        defaultWorldModel: ModelReference(path: "world.json"),
        embodimentContract: ModelReference(path: "embodiment.json")
    )
    let robot = LoadedKuyuRobot(
        manifest: manifest,
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        compatibilityReport: nil,
        baseURL: FileManager.default.temporaryDirectory
    )

    let resolution = try ReferenceQuadrotorEmbodimentResolver().resolution(for: robot)

    #expect(resolution.parameters.mass == 1)
    #expect(resolution.parameters.inertia == Axis3(x: 0.004, y: 0.004, z: 0.007))
    #expect(resolution.parameters.maxThrust == 12)
    #expect(resolution.parameters.motorTimeConstant == 0.04)
    #expect(abs(resolution.parameters.gravity - 9.80665) < 1e-12)
    #expect(abs(resolution.normalizedActuatorRateLimitPerSecond - (200.0 / 12.0)) < 1e-12)
}

private struct RoArmLikeDescriptorCorpusFixture {
    let body: KuyuBodyModel
    let world: KuyuWorldModel
    let embodiment: EmbodimentContract
    let jointIDs: [String]
    let actuatorIDs: [String]
    let timeStep: Double
}

private struct RigidAerialDescriptorCorpusFixture {
    let robotID: String
    let body: KuyuBodyModel
    let world: KuyuWorldModel
    let embodiment: EmbodimentContract
    let timeStep: Double
}

private struct ContactDescriptorCorpusFixture {
    let body: KuyuBodyModel
    let world: KuyuWorldModel
    let embodiment: EmbodimentContract
    let timeStep: Double
}

private func temporaryDescriptorCorpusDirectory() -> URL {
    FileManager.default.temporaryDirectory
        .appendingPathComponent("descriptor-corpus-\(UUID().uuidString)", isDirectory: true)
}

private func loadedDescriptorCorpusRobot(
    fixture: RoArmLikeDescriptorCorpusFixture,
    robotID: String = "roarm-m1-like-v0"
) -> LoadedKuyuRobot {
    let manifest = KuyuRobotManifest(
        schemaVersion: "kuyu.robot.v1",
        robotID: robotID,
        name: "RoArm M1 Like",
        category: "manipulator",
        bodyModel: ModelReference(path: "body.json"),
        defaultWorldModel: ModelReference(path: "world.json"),
        embodimentContract: ModelReference(path: "embodiment.json")
    )
    return LoadedKuyuRobot(
        manifest: manifest,
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        compatibilityReport: nil,
        baseURL: FileManager.default.temporaryDirectory
    )
}

private func contactDescriptorCorpusFixture(
    bodyMaterial: BodyMaterial = BodyMaterial(
        id: "rubber",
        density: 1_000,
        staticFriction: 0.8,
        dynamicFriction: 0.6,
        restitution: 0
    )
) -> ContactDescriptorCorpusFixture {
    let timeStep = 0.005
    let body = KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "contact-material-prismatic-foot-body-v0",
        name: "Contact Material Prismatic Foot",
        category: "contact-fixture",
        frames: [
            FrameDefinition(id: "vertical-output", parentID: "base", pose: KuyuPose())
        ],
        links: [
            LinkDefinition(
                id: "base",
                mass: 1,
                centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
                inertia: KuyuInertiaTensor(ixx: 0.01, ixy: 0, ixz: 0, iyy: 0.01, iyz: 0, izz: 0.01),
                collisions: [
                    GeometryInstance(
                        id: "base-clearance",
                        kind: .sphere,
                        pose: KuyuPose(xyz: KuyuVector3(x: 0, y: 0, z: 2)),
                        radius: 0.01
                    )
                ],
                materialID: "rubber"
            ),
            LinkDefinition(
                id: "foot",
                mass: 0.2,
                centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
                inertia: KuyuInertiaTensor(ixx: 0.01, ixy: 0, ixz: 0, iyy: 0.01, iyz: 0, izz: 0.01),
                collisions: [
                    GeometryInstance(id: "foot-contact", kind: .sphere, radius: 0.05)
                ],
                materialID: "rubber"
            )
        ],
        joints: [
            JointDefinition(
                id: "vertical_joint",
                kind: .prismatic,
                parentLinkID: "base",
                childLinkID: "foot",
                origin: KuyuPose(),
                axis: KuyuVector3(x: 0, y: 0, z: 1),
                lowerLimit: -0.2,
                upperLimit: 0.2,
                effortLimit: 1_000,
                velocityLimit: 1_000,
                homePosition: -0.10
            )
        ],
        materials: [bodyMaterial],
        actuatorMounts: [
            ActuatorMount(
                actuatorID: "vertical-actuator",
                parentLinkID: "base",
                frameID: "vertical-output",
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 0, y: 0, z: 1)
            )
        ],
        actuatorAttachments: [
            ActuatorAttachment(
                actuatorID: "vertical-actuator",
                jointID: "vertical_joint",
                torqueLimit: 1_000,
                mountFrameID: "vertical-output"
            )
        ]
    )
    let embodiment = EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "contact-material-prismatic-foot-contract-v0",
        bodyID: body.bodyID,
        signals: SignalCatalog(
            sensor: [],
            actuator: [
                SignalDefinition(id: "actuator.vertical", index: 0, name: "Vertical actuator", units: "m")
            ],
            drive: [
                SignalDefinition(
                    id: "drive.vertical",
                    index: 0,
                    name: "Vertical drive",
                    units: "m",
                    range: ScalarRange(min: -0.2, max: 0.2)
                )
            ],
            reflex: [
                SignalDefinition(id: "reflex.vertical", index: 0, name: "Vertical reflex", units: "m")
            ]
        ),
        sensors: [],
        actuators: [
            ActuatorDefinition(
                id: "vertical-actuator",
                type: "linear-servo",
                frameID: "vertical-output",
                channels: ["actuator.vertical"],
                limits: ActuatorLimits(min: -0.2, max: 0.2, rateLimitPerSecond: 1_000),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.001, deadzone: 0, torqueLimit: 1_000)
            )
        ],
        control: ControlContract(driveChannels: ["drive.vertical"], reflexChannels: ["reflex.vertical"]),
        motorNerve: MotorNerveContract(stages: [
            MotorNerveStageDefinition(
                id: "direct",
                type: .direct,
                inputs: ["drive.vertical"],
                outputs: ["actuator.vertical"]
            )
        ])
    )
    let world = KuyuWorldModel(
        schemaVersion: "kuyu.world.v1",
        worldID: "contact-material-prismatic-foot-world-v0",
        time: TimeModel(fixedStepSeconds: timeStep, substeps: 5),
        integrator: IntegratorModel(kind: .semiImplicitEuler),
        solver: SolverModel(kind: .deterministicConstraint, iterations: 20, tolerance: 1e-9),
        gravity: .earthUniform,
        atmosphere: AtmosphereModel(kind: .none),
        wind: WindModel(kind: .none),
        surfaces: [
            WorldSurface(
                id: "floor",
                frameID: "world",
                materialID: "floor",
                pose: KuyuPose(xyz: KuyuVector3(x: 0, y: 0, z: -0.025)),
                geometry: GeometryInstance(
                    id: "floor-box",
                    kind: .box,
                    size: KuyuVector3(x: 10, y: 10, z: 0.05)
                )
            )
        ],
        materials: [
            WorldMaterial(id: "floor", staticFriction: 0.7, dynamicFriction: 0.5, restitution: 0)
        ],
        contact: ContactModel(mode: .constraint),
        nap: NegligibilityApproximationPolicy(
            forceAbsoluteThreshold: 0,
            forceRelativeThreshold: 0,
            torqueAbsoluteThreshold: 0,
            torqueRelativeThreshold: 0
        ),
        randomness: RandomnessModel(seed: 0, deterministicReplay: true)
    )
    return ContactDescriptorCorpusFixture(
        body: body,
        world: world,
        embodiment: embodiment,
        timeStep: timeStep
    )
}

private func rigidAerialDescriptorCorpusFixture(
    robotID: String,
    bodyID: String,
    actuatorCount: Int
) -> RigidAerialDescriptorCorpusFixture {
    let actuatorIDs = (0..<actuatorCount).map { "motor_\($0 + 1)" }
    let actuatorSignalIDs = actuatorIDs
    let driveSignalIDs = (0..<actuatorCount).map { "drive_motor_\($0 + 1)" }
    let reflexSignalIDs = (0..<actuatorCount).map { "reflex_motor_\($0 + 1)" }
    let sensorSignalIDs = actuatorCount == 1
        ? ["imu_accel_z"]
        : ["imu_gyro_x", "imu_gyro_y", "imu_gyro_z", "imu_accel_x", "imu_accel_y", "imu_accel_z"]
    let timeStep = 0.01
    let body = KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: bodyID,
        name: "Rigid Aerial Descriptor \(robotID)",
        category: "aerial",
        frames: [FrameDefinition(id: "base_link")],
        links: [
            LinkDefinition(
                id: "base_link",
                mass: actuatorCount == 1 ? 0.25 : 1.0,
                centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
                inertia: KuyuInertiaTensor(
                    ixx: actuatorCount == 1 ? 0.001 : 0.004,
                    ixy: 0,
                    ixz: 0,
                    iyy: actuatorCount == 1 ? 0.001 : 0.004,
                    iyz: 0,
                    izz: actuatorCount == 1 ? 0.0015 : 0.007
                )
            )
        ],
        joints: [],
        sensorMounts: [SensorMount(sensorID: "imu", frameID: "base_link")]
    )
    let embodiment = EmbodimentContract(
        schemaVersion: "kuyu.embodiment.v1",
        contractID: "\(robotID)-rigid-embodiment-v0",
        bodyID: bodyID,
        signals: SignalCatalog(
            sensor: sensorSignalIDs.enumerated().map { index, id in
                SignalDefinition(id: id, index: index, name: id, units: index < 3 ? "rad/s" : "m/s^2")
            },
            actuator: actuatorSignalIDs.enumerated().map { index, id in
                SignalDefinition(id: id, index: index, name: id, units: "N", range: ScalarRange(min: 0, max: 12))
            },
            drive: driveSignalIDs.enumerated().map { index, id in
                SignalDefinition(id: id, index: index, name: id, units: "N", range: ScalarRange(min: 0, max: 12))
            },
            reflex: reflexSignalIDs.enumerated().map { index, id in
                SignalDefinition(id: id, index: index, name: id, units: "N", range: ScalarRange(min: -2, max: 2))
            }
        ),
        sensors: [
            SensorDefinition(
                id: "imu",
                type: actuatorCount == 1 ? "imu1" : "imu6",
                frameID: "base_link",
                channels: sensorSignalIDs,
                rateHz: 1_000,
                latencySeconds: 0.001
            )
        ],
        actuators: actuatorIDs.enumerated().map { index, id in
            ActuatorDefinition(
                id: id,
                type: "rotor",
                frameID: "base_link",
                channels: [actuatorSignalIDs[index]],
                limits: ActuatorLimits(min: 0, max: 12, rateLimitPerSecond: 200),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.04, deadzone: 0, torqueLimit: 0.8)
            )
        },
        control: ControlContract(driveChannels: driveSignalIDs, reflexChannels: reflexSignalIDs),
        motorNerve: MotorNerveContract(stages: [
            MotorNerveStageDefinition(
                id: "direct-thrust",
                type: .direct,
                inputs: driveSignalIDs,
                outputs: actuatorSignalIDs
            )
        ])
    )
    let world = KuyuWorldModel(
        schemaVersion: "kuyu.world.v1",
        worldID: "\(robotID)-rigid-world-v0",
        time: TimeModel(fixedStepSeconds: timeStep, substeps: 1),
        integrator: IntegratorModel(kind: .semiImplicitEuler),
        solver: SolverModel(kind: .disabledContact, iterations: 0, tolerance: 0),
        gravity: .earthUniform,
        atmosphere: AtmosphereModel(kind: .standard, airDensity: 1.225, temperatureKelvin: 288.15),
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
    return RigidAerialDescriptorCorpusFixture(
        robotID: robotID,
        body: body,
        world: world,
        embodiment: embodiment,
        timeStep: timeStep
    )
}

private func roArmLikeDescriptorCorpusFixture() -> RoArmLikeDescriptorCorpusFixture {
    let jointCount = RoArmM1ServoCommandEncoder.jointCount
    let jointIDs = (0..<jointCount).map { "joint_\($0 + 1)" }
    let actuatorIDs = (0..<jointCount).map { "servo_\($0 + 1)" }
    let driveSignalIDs = (0..<jointCount).map { "drive.roarm.joint_\($0 + 1)" }
    let actuatorSignalIDs = (0..<jointCount).map { "actuator.roarm.joint_\($0 + 1)" }
    let reflexSignalIDs = (0..<jointCount).map { "reflex.roarm.joint_\($0 + 1)" }
    let linkIDs = (0...jointCount).map { "link_\($0)" }
    let frameIDs = (0..<jointCount).map { "servo_frame_\($0 + 1)" }
    let limits = RoArmM1ServoCommandEncoder.safeCommissioningJointLimits

    let links = linkIDs.enumerated().map { index, id in
        LinkDefinition(
            id: id,
            mass: 0.10 + Double(index) * 0.015,
            centerOfMass: KuyuVector3(x: 0.02, y: 0, z: 0),
            inertia: KuyuInertiaTensor(ixx: 0.004, ixy: 0, ixz: 0, iyy: 0.004, iyz: 0, izz: 0.004)
        )
    }
    let joints = (0..<jointCount).map { index in
        JointDefinition(
            id: jointIDs[index],
            kind: .revolute,
            parentLinkID: linkIDs[index],
            childLinkID: linkIDs[index + 1],
            origin: KuyuPose(xyz: KuyuVector3(x: 0.045, y: 0, z: 0.015)),
            axis: index == 0 ? KuyuVector3(x: 0, y: 0, z: 1) : KuyuVector3(x: 0, y: 1, z: 0),
            lowerLimit: limits[index].lowerBound,
            upperLimit: limits[index].upperBound,
            effortLimit: 5.0,
            velocityLimit: 5.0,
            homePosition: 0,
            damping: 0.02
        )
    }
    let frames = (0..<jointCount).map { index in
        FrameDefinition(id: frameIDs[index], parentID: linkIDs[index], pose: KuyuPose())
    }
    let mounts = (0..<jointCount).map { index in
        ActuatorMount(
            actuatorID: actuatorIDs[index],
            parentLinkID: linkIDs[index],
            frameID: frameIDs[index],
            pose: KuyuPose(),
            outputAxis: KuyuVector3(x: 0, y: 0, z: 1)
        )
    }
    let attachments = (0..<jointCount).map { index in
        ActuatorAttachment(
            actuatorID: actuatorIDs[index],
            jointID: jointIDs[index],
            torqueLimit: 5.0,
            mountFrameID: frameIDs[index],
            reflectedInertia: 0.0002
        )
    }
    let body = KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "roarm-m1-like-body-v0",
        name: "RoArm M1 Like Descriptor",
        category: "manipulator",
        provenance: [
            ModelProvenance(
                source: "roarm-m1-descriptor-corpus-smoke",
                format: "kuyu.synthetic-real-shape",
                notes: "Real-profile shape used to exercise descriptor corpus acceptance."
            )
        ],
        frames: frames,
        links: links,
        joints: Array(joints.reversed()),
        actuatorMounts: mounts,
        actuatorAttachments: attachments
    )
    let embodiment = EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "roarm-m1-like-embodiment-v0",
        bodyID: body.bodyID,
        signals: SignalCatalog(
            sensor: [],
            actuator: actuatorSignalIDs.enumerated().map { index, id in
                SignalDefinition(id: id, index: index, name: "RoArm actuator \(index + 1)", units: "rad")
            },
            drive: driveSignalIDs.enumerated().map { index, id in
                SignalDefinition(
                    id: id,
                    index: index,
                    name: "RoArm drive \(index + 1)",
                    units: "rad",
                    range: ScalarRange(min: limits[index].lowerBound, max: limits[index].upperBound)
                )
            },
            reflex: reflexSignalIDs.enumerated().map { index, id in
                SignalDefinition(id: id, index: index, name: "RoArm reflex \(index + 1)", units: "rad")
            }
        ),
        sensors: [],
        actuators: actuatorIDs.enumerated().map { index, id in
            ActuatorDefinition(
                id: id,
                type: "serial-servo",
                frameID: frameIDs[index],
                channels: [actuatorSignalIDs[index]],
                limits: ActuatorLimits(
                    min: limits[index].lowerBound,
                    max: limits[index].upperBound,
                    rateLimitPerSecond: 5.0
                ),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.002, deadzone: 0, torqueLimit: 5.0)
            )
        },
        control: ControlContract(driveChannels: driveSignalIDs, reflexChannels: reflexSignalIDs),
        motorNerve: MotorNerveContract(stages: [
            MotorNerveStageDefinition(
                id: "direct",
                type: .direct,
                inputs: driveSignalIDs,
                outputs: actuatorSignalIDs
            )
        ])
    )
    let timeStep = 0.005
    let world = KuyuWorldModel(
        schemaVersion: "kuyu.world.v1",
        worldID: "roarm-m1-like-bench-world-v0",
        time: TimeModel(fixedStepSeconds: timeStep, substeps: 5),
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

    return RoArmLikeDescriptorCorpusFixture(
        body: body,
        world: world,
        embodiment: embodiment,
        jointIDs: jointIDs,
        actuatorIDs: actuatorIDs,
        timeStep: timeStep
    )
}

private func roArmLikeHardwareReport(
    body: KuyuBodyModel,
    embodiment: EmbodimentContract,
    readinessLevel: ReadinessLevel
) throws -> HardwareCalibrationReport {
    HardwareCalibrationReport(
        schemaVersion: "kuyu.hardware-calibration.v1",
        reportID: "roarm-m1-like-hardware-report-\(readinessLevel.rawValue)",
        robotID: "roarm-m1-v0",
        bodyID: body.bodyID,
        embodimentContractID: embodiment.contractID,
        readinessLevel: readinessLevel,
        positionToleranceRadians: 0.02,
        minimumSamplesPerJoint: 3,
        source: HardwareCalibrationSource(
            deviceID: "roarm-m1-like-bench",
            measurementSystem: "bench-fixture"
        ),
        jointCalibrations: try activeCorpusJoints(body: body).map { joint in
            let attachment = try #require(body.actuatorAttachments.first { $0.jointID == joint.id })
            return JointHardwareCalibration(
                jointID: joint.id,
                actuatorID: attachment.actuatorID,
                commandDirection: attachment.commandDirection,
                mechanicalReductionRatio: attachment.mechanicalReductionRatio,
                identifiedDynamics: IdentifiedJointDynamics(
                    latencySeconds: 0.004,
                    timeConstantSeconds: 0.002,
                    deadbandRadians: 0.001,
                    backlashRadians: 0.001,
                    viscousDamping: 0.02,
                    coulombFriction: 0.01,
                    meanAbsoluteErrorRadians: 0.005,
                    maxObservedErrorRadians: 0.010
                ),
                samples: [
                    JointCalibrationSample(
                        commandedPositionRadians: -0.10,
                        measuredPositionRadians: -0.096,
                        commandTimeSeconds: 0.0,
                        observedTimeSeconds: 0.02
                    ),
                    JointCalibrationSample(
                        commandedPositionRadians: 0.0,
                        measuredPositionRadians: 0.002,
                        commandTimeSeconds: 0.1,
                        observedTimeSeconds: 0.12
                    ),
                    JointCalibrationSample(
                        commandedPositionRadians: 0.10,
                        measuredPositionRadians: 0.094,
                        commandTimeSeconds: 0.2,
                        observedTimeSeconds: 0.22
                    )
                ]
            )
        }
    )
}

private func activeCorpusJoints(body: KuyuBodyModel) -> [JointDefinition] {
    body.joints.filter { joint in
        joint.mimic == nil && (joint.kind == .revolute || joint.kind == .continuous || joint.kind == .prismatic)
    }
}
