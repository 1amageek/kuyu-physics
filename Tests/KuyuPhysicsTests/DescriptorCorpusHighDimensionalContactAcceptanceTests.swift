import EmbodimentContract
import Foundation
import KuyuCore
import KuyuPhysics
import Testing

@Test(.timeLimit(.minutes(1))) func descriptorCorpusAcceptancePersistsHighDimensionalContactMaterialVariant() async throws {
    let fixture = highDimensionalContactDescriptorCorpusFixture(jointCount: 24)
    let entry = DescriptorCorpusEntry(
        entryID: "high-dimensional-contact-material-chain-v0",
        robotID: "high-dimensional-contact-chain-v0",
        label: "High-dimensional contact material descriptor corpus variant",
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        requiredReadiness: .contactTraining,
        duration: 0.05,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(24),
        determinism: try DeterminismConfig(tier: .tier0)
    )

    let summary = try await DescriptorCorpusAcceptanceService().accept(
        corpusID: "high-dimensional-contact-material-descriptor-corpus",
        entries: [entry],
        generatedAt: "2026-06-30T00:00:00Z"
    )
    let record = try #require(summary.records.first)
    let contact = try #require(record.replay.contact)
    let directory = highDimensionalContactDescriptorCorpusDirectory()
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
    #expect(record.replay.passed)
    #expect(record.replay.sortedJSONByteStable)
    #expect(record.replay.stepCount == 10)
    #expect(contact.maxActiveContactCount >= Double(fixture.jointIDs.count))
    #expect(contact.maxPenetration <= 1e-9)
    #expect(contact.maxSolverIterations > 0)
    #expect(reloaded == summary)
}

private struct HighDimensionalContactDescriptorCorpusFixture {
    let body: KuyuBodyModel
    let world: KuyuWorldModel
    let embodiment: EmbodimentContract
    let timeStep: Double
    let jointIDs: [String]
}

private func highDimensionalContactDescriptorCorpusDirectory() -> URL {
    FileManager.default.temporaryDirectory
        .appendingPathComponent("high-dimensional-contact-descriptor-corpus-\(UUID().uuidString)", isDirectory: true)
}

private func highDimensionalContactDescriptorCorpusFixture(
    jointCount: Int
) -> HighDimensionalContactDescriptorCorpusFixture {
    let jointIDs = (0..<jointCount).map { "contact_joint_\($0)" }
    let actuatorIDs = (0..<jointCount).map { "contact_actuator_\($0)" }
    let driveSignalIDs = (0..<jointCount).map { "drive.contact_joint_\($0)" }
    let actuatorSignalIDs = (0..<jointCount).map { "actuator.contact_joint_\($0)" }
    let reflexSignalIDs = (0..<jointCount).map { "reflex.contact_joint_\($0)" }
    let linkIDs = (0...jointCount).map { "contact_link_\($0)" }
    let frameIDs = (0..<jointCount).map { "contact_actuator_frame_\($0)" }
    let jointRange = -0.02...0.02
    let timeStep = 0.005
    let body = KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "high-dimensional-contact-material-body-\(jointCount)",
        name: "High Dimensional Contact Material Chain",
        category: "contact-manipulator",
        frames: (0..<jointCount).map { index in
            FrameDefinition(id: frameIDs[index], parentID: linkIDs[index], pose: KuyuPose())
        },
        links: linkIDs.enumerated().map { index, id in
            LinkDefinition(
                id: id,
                mass: 0.2,
                centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
                inertia: KuyuInertiaTensor(ixx: 0.01, ixy: 0, ixz: 0, iyy: 0.01, iyz: 0, izz: 0.01),
                collisions: [
                    GeometryInstance(
                        id: "\(id)-contact",
                        kind: .sphere,
                        pose: KuyuPose(xyz: KuyuVector3(x: 0, y: 0, z: index == 0 ? 2 : 0)),
                        radius: 0.01
                    )
                ],
                materialID: "rubber"
            )
        },
        joints: Array((0..<jointCount).map { index in
            JointDefinition(
                id: jointIDs[index],
                kind: .prismatic,
                parentLinkID: linkIDs[index],
                childLinkID: linkIDs[index + 1],
                origin: KuyuPose(),
                axis: KuyuVector3(x: 0, y: 0, z: 1),
                lowerLimit: jointRange.lowerBound,
                upperLimit: jointRange.upperBound,
                effortLimit: 1_000,
                velocityLimit: 1_000,
                homePosition: 0,
                damping: 0.001
            )
        }.reversed()),
        materials: [
            BodyMaterial(id: "rubber", density: 1_000, staticFriction: 0.8, dynamicFriction: 0.6, restitution: 0)
        ],
        actuatorMounts: (0..<jointCount).map { index in
            ActuatorMount(
                actuatorID: actuatorIDs[index],
                parentLinkID: linkIDs[index],
                frameID: frameIDs[index],
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 0, y: 0, z: 1)
            )
        },
        actuatorAttachments: (0..<jointCount).map { index in
            ActuatorAttachment(
                actuatorID: actuatorIDs[index],
                jointID: jointIDs[index],
                torqueLimit: 1_000,
                mountFrameID: frameIDs[index]
            )
        }
    )
    let embodiment = EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "high-dimensional-contact-material-contract-\(jointCount)",
        bodyID: body.bodyID,
        signals: SignalCatalog(
            sensor: [],
            actuator: actuatorSignalIDs.enumerated().map { index, id in
                SignalDefinition(id: id, index: index, name: "Contact actuator \(index)", units: "m")
            },
            drive: driveSignalIDs.enumerated().map { index, id in
                SignalDefinition(
                    id: id,
                    index: index,
                    name: "Contact drive \(index)",
                    units: "m",
                    range: ScalarRange(min: jointRange.lowerBound, max: jointRange.upperBound)
                )
            },
            reflex: reflexSignalIDs.enumerated().map { index, id in
                SignalDefinition(id: id, index: index, name: "Contact reflex \(index)", units: "m")
            }
        ),
        sensors: [],
        actuators: actuatorIDs.enumerated().map { index, id in
            ActuatorDefinition(
                id: id,
                type: "linear-servo",
                frameID: frameIDs[index],
                channels: [actuatorSignalIDs[index]],
                limits: ActuatorLimits(
                    min: jointRange.lowerBound,
                    max: jointRange.upperBound,
                    rateLimitPerSecond: 1_000
                ),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.001, deadzone: 0, torqueLimit: 1_000)
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
    let world = KuyuWorldModel(
        schemaVersion: "kuyu.world.v1",
        worldID: "high-dimensional-contact-material-world",
        time: TimeModel(fixedStepSeconds: timeStep, substeps: 5),
        integrator: IntegratorModel(kind: .semiImplicitEuler),
        solver: SolverModel(kind: .deterministicConstraint, iterations: 30, tolerance: 1e-9),
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
    return HighDimensionalContactDescriptorCorpusFixture(
        body: body,
        world: world,
        embodiment: embodiment,
        timeStep: timeStep,
        jointIDs: jointIDs
    )
}
