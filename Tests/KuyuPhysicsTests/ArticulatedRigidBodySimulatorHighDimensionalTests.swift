import Foundation
import KuyuCore
import KuyuPhysics
import Testing

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorPreservesTwelveAxisSignalContracts() async throws {
    let fixture = highDimensionalFixture(jointCount: 12)
    let request = ArticulatedRigidBodySimulationRequest(
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 1.0,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(12)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: HighDimensionalDriveProvider(
            expectedJointIDs: fixture.jointIDs,
            expectedDriveSignalIDs: fixture.driveSignalIDs,
            expectedActuatorSignalIDs: fixture.actuatorSignalIDs,
            expectedRanges: fixture.jointRanges,
            expectedInitialPositions: fixture.homePositions,
            activations: fixture.targets
        )
    )

    #expect(log.events.count == 200)
    for step in log.events {
        #expect(step.sensorSamples.count == fixture.jointIDs.count)
        #expect(step.driveIntents.count == fixture.jointIDs.count)
        #expect(step.actuatorValues.count == fixture.jointIDs.count)
        #expect(step.actuatorTelemetry.channels.count == fixture.jointIDs.count)
        #expect(step.motorNerveTrace?.uOut.count == fixture.jointIDs.count)

        for index in fixture.jointIDs.indices {
            let jointID = fixture.jointIDs[index]
            let actuatorSignalID = fixture.actuatorSignalIDs[index]
            let position = try #require(step.plantState.scalars[jointID])
            let target = try #require(step.plantState.scalars["target_\(jointID)"])
            let actuatorPosition = try #require(step.plantState.scalars[actuatorSignalID])
            let velocity = try #require(step.plantState.scalars["velocity_\(jointID)"])
            let torque = try #require(step.plantState.scalars["torque_\(jointID)"])

            #expect(position.isFinite)
            #expect(target == fixture.targets[index])
            #expect(actuatorPosition == position)
            #expect(velocity.isFinite)
            #expect(torque.isFinite)
            #expect(fixture.jointRanges[index].contains(position))
        }
    }
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorTwelveAxisReplayIsByteStable() async throws {
    let fixture = highDimensionalFixture(jointCount: 12)
    let request = ArticulatedRigidBodySimulationRequest(
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 1.0,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(12)
    )
    let provider = HighDimensionalDriveProvider(
        expectedJointIDs: fixture.jointIDs,
        expectedDriveSignalIDs: fixture.driveSignalIDs,
        expectedActuatorSignalIDs: fixture.actuatorSignalIDs,
        expectedRanges: fixture.jointRanges,
        expectedInitialPositions: fixture.homePositions,
        activations: fixture.targets
    )

    let first = try await ArticulatedRigidBodySimulator().run(request: request, driveProvider: provider)
    let second = try await ArticulatedRigidBodySimulator().run(request: request, driveProvider: provider)
    let encoder = JSONEncoder()
    encoder.outputFormatting = [.sortedKeys]

    #expect(first == second)
    #expect(try encoder.encode(first) == encoder.encode(second))
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorTwelveAxisPerformanceBudget() async throws {
    let fixture = highDimensionalFixture(jointCount: 12)
    let request = ArticulatedRigidBodySimulationRequest(
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 5.0,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(12)
    )

    let startedAt = Date()
    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: HighDimensionalFixedDriveProvider(activations: fixture.targets)
    )
    let elapsedSeconds = Date().timeIntervalSince(startedAt)
    let axisStepsPerSecond = Double(log.events.count * fixture.jointIDs.count) / elapsedSeconds

    #expect(log.events.count == 1_000)
    #expect(axisStepsPerSecond >= 10_000)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorPreservesFortyEightAxisMixedJointContracts() async throws {
    let fixture = highDimensionalFixture(
        jointCount: 48,
        mixedJointKinds: true,
        nonZeroHomePositions: true
    )
    let request = ArticulatedRigidBodySimulationRequest(
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.25,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(48)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: HighDimensionalDriveProvider(
            expectedJointIDs: fixture.jointIDs,
            expectedDriveSignalIDs: fixture.driveSignalIDs,
            expectedActuatorSignalIDs: fixture.actuatorSignalIDs,
            expectedRanges: fixture.jointRanges,
            expectedInitialPositions: fixture.homePositions,
            activations: fixture.targets
        )
    )
    let finalStep = try #require(log.events.last)

    #expect(log.events.count == 50)
    #expect(finalStep.plantState.bodies.count == fixture.jointIDs.count)
    #expect(finalStep.sensorSamples.count == fixture.jointIDs.count)
    #expect(finalStep.driveIntents.count == fixture.jointIDs.count)
    #expect(finalStep.actuatorValues.count == fixture.jointIDs.count)
    #expect(finalStep.actuatorTelemetry.channels.count == fixture.jointIDs.count)
    #expect(finalStep.motorNerveTrace?.uOut.count == fixture.jointIDs.count)

    for index in fixture.jointIDs.indices {
        let jointID = fixture.jointIDs[index]
        let actuatorSignalID = fixture.actuatorSignalIDs[index]
        let position = try #require(finalStep.plantState.scalars[jointID])
        let target = try #require(finalStep.plantState.scalars["target_\(jointID)"])
        let actuatorPosition = try #require(finalStep.plantState.scalars[actuatorSignalID])
        let velocity = try #require(finalStep.plantState.scalars["velocity_\(jointID)"])
        let torque = try #require(finalStep.plantState.scalars["torque_\(jointID)"])

        #expect(position.isFinite)
        #expect(target == fixture.targets[index])
        #expect(actuatorPosition == position)
        #expect(velocity.isFinite)
        #expect(torque.isFinite)
        #expect(fixture.jointRanges[index].contains(position))
    }
}

@Test func readinessRejectsHighDimensionalPlantWithMissingActuatorDynamics() throws {
    let fixture = highDimensionalFixture(
        jointCount: 48,
        mixedJointKinds: true,
        nonZeroHomePositions: true
    )
    let invalidEmbodiment = highDimensionalEmbodimentDroppingDynamics(
        fixture.embodiment,
        actuatorID: "actuator_31"
    )

    #expect(throws: KuyuModelValidationError.empty("readiness.dynamic.actuators.actuator_31.dynamics")) {
        _ = try ReadinessGate().validate(
            body: fixture.body,
            world: fixture.world,
            embodiment: invalidEmbodiment,
            report: nil,
            requiredLevel: .dynamicSimulation
        )
    }
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorFortyEightAxisMixedReplayIsByteStable() async throws {
    let fixture = highDimensionalFixture(
        jointCount: 48,
        mixedJointKinds: true,
        nonZeroHomePositions: true
    )
    let request = ArticulatedRigidBodySimulationRequest(
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.25,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(48)
    )
    let provider = HighDimensionalFixedDriveProvider(activations: fixture.targets)
    let encoder = JSONEncoder()
    encoder.outputFormatting = [.sortedKeys]

    let first = try await ArticulatedRigidBodySimulator().run(request: request, driveProvider: provider)
    let second = try await ArticulatedRigidBodySimulator().run(request: request, driveProvider: provider)

    #expect(first == second)
    #expect(try encoder.encode(first) == encoder.encode(second))
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorFortyEightAxisMixedPerformanceBudget() async throws {
    let fixture = highDimensionalFixture(
        jointCount: 48,
        mixedJointKinds: true,
        nonZeroHomePositions: true
    )
    let request = ArticulatedRigidBodySimulationRequest(
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 1.0,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(48)
    )

    let startedAt = Date()
    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: HighDimensionalFixedDriveProvider(activations: fixture.targets)
    )
    let elapsedSeconds = Date().timeIntervalSince(startedAt)
    let axisStepsPerSecond = Double(log.events.count * fixture.jointIDs.count) / elapsedSeconds

    #expect(log.events.count == 200)
    #expect(axisStepsPerSecond >= 20_000)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorFortyEightAxisContactConstraintPerformanceBudget() async throws {
    let fixture = highDimensionalContactFixture(jointCount: 48)
    let request = ArticulatedRigidBodySimulationRequest(
        body: fixture.body,
        world: fixture.world,
        embodiment: fixture.embodiment,
        determinism: try DeterminismConfig(tier: .tier0),
        readinessLevel: .contactTraining,
        duration: 0.25,
        timeStep: try TimeStep(delta: fixture.timeStep),
        seed: ScenarioSeed(4_848)
    )

    let startedAt = Date()
    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: HighDimensionalFixedDriveProvider(activations: fixture.targets)
    )
    let elapsedSeconds = Date().timeIntervalSince(startedAt)
    let axisStepsPerSecond = Double(log.events.count * fixture.jointIDs.count) / elapsedSeconds
    let finalStep = try #require(log.events.last)
    let finalPenetration = try #require(finalStep.plantState.scalars["contact.penetration.max"])
    let maxActiveContacts = log.events.compactMap { $0.plantState.scalars["contact.active.count"] }.max() ?? 0

    #expect(log.events.count == 50)
    #expect(maxActiveContacts >= 48)
    #expect(finalPenetration <= 1e-9)
    #expect(axisStepsPerSecond >= 5_000)
}

private struct HighDimensionalFixture {
    let body: KuyuBodyModel
    let world: KuyuWorldModel
    let embodiment: EmbodimentContract
    let timeStep: Double
    let jointIDs: [String]
    let driveSignalIDs: [String]
    let actuatorSignalIDs: [String]
    let jointRanges: [ClosedRange<Double>]
    let homePositions: [Double]
    let targets: [Double]
}

private func highDimensionalFixture(
    jointCount: Int,
    mixedJointKinds: Bool = false,
    nonZeroHomePositions: Bool = false
) -> HighDimensionalFixture {
    let jointIDs = (0..<jointCount).map { "joint_\($0)" }
    let actuatorIDs = (0..<jointCount).map { "actuator_\($0)" }
    let driveSignalIDs = (0..<jointCount).map { "drive.joint_\($0)" }
    let actuatorSignalIDs = (0..<jointCount).map { "actuator.joint_\($0)" }
    let reflexSignalIDs = (0..<jointCount).map { "reflex.joint_\($0)" }
    let linkIDs = (0...jointCount).map { "link_\($0)" }
    let frameIDs = (0..<jointCount).map { "actuator_frame_\($0)" }
    let jointRange = -0.5...0.5
    let jointRanges = Array(repeating: jointRange, count: jointCount)
    let homePositions = (0..<jointCount).map { index in
        nonZeroHomePositions ? -0.12 + (Double(index % 7) * 0.04) : 0.0
    }
    let targets = (0..<jointCount).map { index in
        -0.24 + (Double(index % 13) * 0.04)
    }

    let links = linkIDs.enumerated().map { index, id in
        LinkDefinition(
            id: id,
            mass: 0.2 + Double(index) * 0.01,
            centerOfMass: KuyuVector3(x: 0.02, y: 0, z: 0),
            inertia: KuyuInertiaTensor(ixx: 0.01, ixy: 0, ixz: 0, iyy: 0.01, iyz: 0, izz: 0.01)
        )
    }
    let joints = (0..<jointCount).map { index in
        let isPrismatic = mixedJointKinds && index % 3 == 1
        let axis: KuyuVector3
        if isPrismatic {
            axis = KuyuVector3(x: 1, y: 0, z: 0)
        } else if mixedJointKinds && index % 4 == 0 {
            axis = KuyuVector3(x: 0, y: 1, z: 0)
        } else {
            axis = KuyuVector3(x: 0, y: 0, z: 1)
        }
        return JointDefinition(
            id: jointIDs[index],
            kind: isPrismatic ? .prismatic : .revolute,
            parentLinkID: linkIDs[index],
            childLinkID: linkIDs[index + 1],
            origin: KuyuPose(xyz: KuyuVector3(x: 0.05, y: 0, z: 0)),
            axis: axis,
            lowerLimit: jointRange.lowerBound,
            upperLimit: jointRange.upperBound,
            effortLimit: 1_000,
            velocityLimit: 1_000,
            homePosition: homePositions[index],
            damping: 0.001
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
            torqueLimit: 1_000,
            mountFrameID: frameIDs[index]
        )
    }
    let body = KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "high-dimensional-articulated-body-\(jointCount)",
        name: "High Dimensional Articulated Body",
        category: "manipulator",
        frames: frames,
        links: links,
        joints: Array(joints.reversed()),
        actuatorMounts: mounts,
        actuatorAttachments: attachments
    )
    let embodiment = EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "high-dimensional-articulated-contract-\(jointCount)",
        bodyID: body.bodyID,
        signals: SignalCatalog(
            sensor: [],
            actuator: actuatorSignalIDs.enumerated().map { index, id in
                SignalDefinition(id: id, index: index, name: "Actuator \(index)", units: "rad")
            },
            drive: driveSignalIDs.enumerated().map { index, id in
                SignalDefinition(
                    id: id,
                    index: index,
                    name: "Drive \(index)",
                    units: "rad",
                    range: ScalarRange(min: jointRange.lowerBound, max: jointRange.upperBound)
                )
            },
            reflex: reflexSignalIDs.enumerated().map { index, id in
                SignalDefinition(id: id, index: index, name: "Reflex \(index)", units: "rad")
            }
        ),
        sensors: [],
        actuators: actuatorIDs.enumerated().map { index, id in
            ActuatorDefinition(
                id: id,
                type: "servo",
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
    let timeStep = 0.005
    let world = KuyuWorldModel(
        schemaVersion: "kuyu.world.v1",
        worldID: "high-dimensional-articulated-world",
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

    return HighDimensionalFixture(
        body: body,
        world: world,
        embodiment: embodiment,
        timeStep: timeStep,
        jointIDs: jointIDs,
        driveSignalIDs: driveSignalIDs,
        actuatorSignalIDs: actuatorSignalIDs,
        jointRanges: jointRanges,
        homePositions: homePositions,
        targets: targets
    )
}

private func highDimensionalEmbodimentDroppingDynamics(
    _ embodiment: EmbodimentContract,
    actuatorID: String
) -> EmbodimentContract {
    let actuators = embodiment.actuators.map { actuator in
        guard actuator.id == actuatorID else { return actuator }
        return ActuatorDefinition(
            id: actuator.id,
            type: actuator.type,
            frameID: actuator.frameID,
            channels: actuator.channels,
            limits: actuator.limits,
            dynamics: nil,
            swapProfile: actuator.swapProfile
        )
    }
    return EmbodimentContract(
        schemaVersion: embodiment.schemaVersion,
        contractID: embodiment.contractID,
        bodyID: embodiment.bodyID,
        signals: embodiment.signals,
        sensors: embodiment.sensors,
        actuators: actuators,
        control: embodiment.control,
        observation: embodiment.observation,
        motorNerve: embodiment.motorNerve
    )
}

private func highDimensionalContactFixture(jointCount: Int) -> HighDimensionalFixture {
    let jointIDs = (0..<jointCount).map { "contact_joint_\($0)" }
    let actuatorIDs = (0..<jointCount).map { "contact_actuator_\($0)" }
    let driveSignalIDs = (0..<jointCount).map { "drive.contact_joint_\($0)" }
    let actuatorSignalIDs = (0..<jointCount).map { "actuator.contact_joint_\($0)" }
    let reflexSignalIDs = (0..<jointCount).map { "reflex.contact_joint_\($0)" }
    let linkIDs = (0...jointCount).map { "contact_link_\($0)" }
    let frameIDs = (0..<jointCount).map { "contact_actuator_frame_\($0)" }
    let jointRange = -0.5...0.5
    let jointRanges = Array(repeating: jointRange, count: jointCount)
    let homePositions = Array(repeating: 0.0, count: jointCount)
    let targets = Array(repeating: 0.0, count: jointCount)

    let links = linkIDs.enumerated().map { index, id in
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
    }
    let joints = (0..<jointCount).map { index in
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
            homePosition: homePositions[index],
            damping: 0.001
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
            torqueLimit: 1_000,
            mountFrameID: frameIDs[index]
        )
    }
    let body = KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "high-dimensional-contact-body-\(jointCount)",
        name: "High Dimensional Contact Body",
        category: "contact-manipulator",
        frames: frames,
        links: links,
        joints: Array(joints.reversed()),
        materials: [
            BodyMaterial(id: "rubber", density: 1_000, staticFriction: 0.8, dynamicFriction: 0.6, restitution: 0)
        ],
        actuatorMounts: mounts,
        actuatorAttachments: attachments
    )
    let embodiment = EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "high-dimensional-contact-contract-\(jointCount)",
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
    let timeStep = 0.005
    let world = KuyuWorldModel(
        schemaVersion: "kuyu.world.v1",
        worldID: "high-dimensional-contact-world",
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

    return HighDimensionalFixture(
        body: body,
        world: world,
        embodiment: embodiment,
        timeStep: timeStep,
        jointIDs: jointIDs,
        driveSignalIDs: driveSignalIDs,
        actuatorSignalIDs: actuatorSignalIDs,
        jointRanges: jointRanges,
        homePositions: homePositions,
        targets: targets
    )
}

private struct HighDimensionalDriveProvider: ArticulatedRigidBodyDriveProvider {
    let providerID = "high-dimensional-drive-provider"
    let expectedJointIDs: [String]
    let expectedDriveSignalIDs: [String]
    let expectedActuatorSignalIDs: [String]
    let expectedRanges: [ClosedRange<Double>]
    let expectedInitialPositions: [Double]
    let activations: [Double]
    private var hasCheckedInitialContext = false

    mutating func reset(context: ArticulatedRigidBodyDriveProviderResetContext) throws {
        guard context.jointIDs == expectedJointIDs else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("reset.jointIDs")
        }
        guard context.driveSignalIDs == expectedDriveSignalIDs else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("reset.driveSignalIDs")
        }
        guard context.jointRanges == expectedRanges else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("reset.jointRanges")
        }
    }

    mutating func driveIntents(context: ArticulatedRigidBodyDriveContext) throws -> [DriveIntent] {
        guard context.jointIDs == expectedJointIDs else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("step.jointIDs")
        }
        guard context.driveSignalIDs == expectedDriveSignalIDs else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("step.driveSignalIDs")
        }
        guard context.actuatorSignalIDs == expectedActuatorSignalIDs else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("step.actuatorSignalIDs")
        }
        guard context.jointRanges == expectedRanges else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("step.jointRanges")
        }
        guard context.positions.count == expectedJointIDs.count,
              context.velocities.count == expectedJointIDs.count,
              context.targets.count == expectedJointIDs.count,
              context.torques.count == expectedJointIDs.count else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("step.state.count")
        }
        guard context.positions.allSatisfy(\.isFinite),
              context.velocities.allSatisfy(\.isFinite),
              context.targets.allSatisfy(\.isFinite),
              context.torques.allSatisfy(\.isFinite) else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("step.state.finite")
        }
        if !hasCheckedInitialContext {
            guard context.positions == expectedInitialPositions,
                  context.targets == expectedInitialPositions else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("step.initialState")
            }
            hasCheckedInitialContext = true
        }
        return try activations.enumerated().map { index, value in
            try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
        }
    }
}

private struct HighDimensionalFixedDriveProvider: ArticulatedRigidBodyDriveProvider {
    let providerID = "high-dimensional-fixed-drive-provider"
    let activations: [Double]

    mutating func driveIntents(context: ArticulatedRigidBodyDriveContext) throws -> [DriveIntent] {
        guard context.jointIDs.count == activations.count else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("fixed-drive-count")
        }
        return try activations.enumerated().map { index, value in
            try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
        }
    }
}
