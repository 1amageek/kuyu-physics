import Foundation
import KuyuCore
import KuyuPhysics
import simd
import Testing

@Test(.timeLimit(.minutes(1))) func articulatedSnapshotComposesTransformsIndependentOfJointOrder() async throws {
    let shoulderPosition = 0.2
    let slidePosition = 0.4
    let fixedRoll = 0.3
    let body = articulatedSnapshotBody(
        shoulderPosition: shoulderPosition,
        slidePosition: slidePosition,
        fixedRoll: fixedRoll
    )
    let request = ArticulatedRigidBodySimulationRequest(
        body: body,
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    let log = try await ArticulatedRigidBodySimulator().run(request: request)
    let step = try #require(log.events.last)
    let snapshots = Dictionary(uniqueKeysWithValues: step.plantState.bodies.map { ($0.id, $0) })
    let arm = try #require(snapshots["arm"])
    let slider = try #require(snapshots["slider"])
    let tip = try #require(snapshots["tip"])
    let shoulder = try #require(step.plantState.scalars["shoulder_joint"])
    let slide = try #require(step.plantState.scalars["slide_joint"])

    let shoulderOrientation = simd_quatd(angle: shoulder, axis: SIMD3<Double>(0, 0, 1))
    let expectedSliderPosition = shoulderOrientation.act(SIMD3<Double>(1 + slide, 0, 0))
    let expectedTipPosition = expectedSliderPosition + shoulderOrientation.act(SIMD3<Double>(0, 1, 0))
    let expectedTipOrientation = (
        shoulderOrientation * simd_quatd(angle: fixedRoll, axis: SIMD3<Double>(1, 0, 0))
    ).normalizedQuat

    assertAxisApproximatelyEqual(arm.position, Axis3(x: 0, y: 0, z: 0), tolerance: 1e-12)
    assertQuaternionApproximatelyEqual(arm.orientation, QuaternionSnapshot(orientation: shoulderOrientation), tolerance: 1e-12)
    assertAxisApproximatelyEqual(slider.position, axis3(expectedSliderPosition), tolerance: 1e-12)
    assertAxisApproximatelyEqual(tip.position, axis3(expectedTipPosition), tolerance: 1e-12)
    assertQuaternionApproximatelyEqual(tip.orientation, QuaternionSnapshot(orientation: expectedTipOrientation), tolerance: 1e-12)
}

@Test(.timeLimit(.minutes(1))) func articulatedSnapshotPreservesBranchedUnorderedJointGraph() async throws {
    let leftAngle = 0.5
    let rightSlide = 0.25
    let fixedPitch = -0.2
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedBranchedSnapshotBody(
            leftAngle: leftAngle,
            rightSlide: rightSlide,
            fixedPitch: fixedPitch
        ),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedBranchedSnapshotEmbodiment(
            leftAngle: leftAngle,
            rightSlide: rightSlide
        ),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [rightSlide, leftAngle])
    )
    let step = try #require(log.events.last)
    let snapshots = Dictionary(uniqueKeysWithValues: step.plantState.bodies.map { ($0.id, $0) })
    let left = try #require(snapshots["left"])
    let leftTip = try #require(snapshots["left_tip"])
    let right = try #require(snapshots["right"])

    let leftOrientation = simd_quatd(angle: leftAngle, axis: SIMD3<Double>(0, 0, 1))
    let expectedLeftPosition = SIMD3<Double>(1, 0, 0)
    let expectedLeftTipPosition = expectedLeftPosition + leftOrientation.act(SIMD3<Double>(0, 1, 0))
    let expectedLeftTipOrientation = (
        leftOrientation * simd_quatd(angle: fixedPitch, axis: SIMD3<Double>(0, 1, 0))
    ).normalizedQuat
    let expectedRightPosition = SIMD3<Double>(0, -1 + rightSlide, 0)

    #expect(Set(step.plantState.bodies.map(\.id)) == Set(["right", "left", "left_tip"]))
    assertAxisApproximatelyEqual(left.position, axis3(expectedLeftPosition), tolerance: 1e-12)
    assertQuaternionApproximatelyEqual(left.orientation, QuaternionSnapshot(orientation: leftOrientation), tolerance: 1e-12)
    assertAxisApproximatelyEqual(leftTip.position, axis3(expectedLeftTipPosition), tolerance: 1e-12)
    assertQuaternionApproximatelyEqual(
        leftTip.orientation,
        QuaternionSnapshot(orientation: expectedLeftTipOrientation),
        tolerance: 1e-12
    )
    assertAxisApproximatelyEqual(right.position, axis3(expectedRightPosition), tolerance: 1e-12)
    assertQuaternionApproximatelyEqual(
        right.orientation,
        QuaternionSnapshot(w: 1, x: 0, y: 0, z: 0),
        tolerance: 1e-12
    )
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorAcceptsExternalDriveProvider() async throws {
    let body = articulatedSnapshotBody(
        shoulderPosition: 0.2,
        slidePosition: 0.4,
        fixedRoll: 0.0
    )
    let request = ArticulatedRigidBodySimulationRequest(
        body: body,
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.02,
        timeStep: try TimeStep(delta: 0.01)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [0.4, 0.2])
    )
    let first = try #require(log.events.first)

    expectHexConfigHash(log.configHash)
    #expect(first.driveIntents.map(\.activation) == [0.4, 0.2])
    #expect(first.driveIntents.map { Int($0.index.rawValue) } == [0, 1])
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorOrdersProviderContextByActuatorSignalIndex() async throws {
    let body = articulatedSnapshotBody(
        shoulderPosition: 0.2,
        slidePosition: 0.4,
        fixedRoll: 0.0,
        activeJointOrder: ["shoulder_joint", "slide_joint"]
    )
    let request = ArticulatedRigidBodySimulationRequest(
        body: body,
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: ExpectedOrderDriveProvider(
            expectedJointIDs: ["slide_joint", "shoulder_joint"],
            activations: [0.4, 0.2]
        )
    )
    let step = try #require(log.events.last)

    #expect(step.plantState.scalars["slide_joint"] == 0.4)
    #expect(step.plantState.scalars["shoulder_joint"] == 0.2)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorInitializesProviderContextInsideJointRanges() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: ExpectedInitialStateDriveProvider(
            expectedPositions: [0.4, 0.2],
            activations: [0.4, 0.2]
        )
    )
    let step = try #require(log.events.last)

    #expect(step.plantState.scalars["slide_joint"] == 0.4)
    #expect(step.plantState.scalars["shoulder_joint"] == 0.2)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorAppliesActuatorAttachmentMappingEndToEnd() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedTransmissionMappedBody(),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedTransmissionMappedEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: ExpectedApproximateInitialStateDriveProvider(
            expectedPositions: [0.15],
            activations: [-0.10],
            tolerance: 1e-12
        )
    )
    let step = try #require(log.events.last)
    let jointPosition = try #require(step.plantState.scalars["mapped_joint"])
    let actuatorPosition = try #require(step.plantState.scalars["actuator.mapped"])
    let jointTarget = try #require(step.plantState.scalars["target_mapped_joint"])
    let actuatorTarget = try #require(step.plantState.scalars["target_actuator.mapped"])
    let actuatorTelemetry = try #require(step.actuatorTelemetry.channels.first)

    expectApproximatelyEqual(jointPosition, 0.15, tolerance: 1e-12)
    expectApproximatelyEqual(actuatorPosition, -0.10, tolerance: 1e-12)
    expectApproximatelyEqual(jointTarget, 0.15, tolerance: 1e-12)
    expectApproximatelyEqual(actuatorTarget, -0.10, tolerance: 1e-12)
    expectApproximatelyEqual(actuatorTelemetry.value, -0.10, tolerance: 1e-12)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorMapsActuatorTorqueUsingMechanicalReduction() async throws {
    let body = articulatedTransmissionMappedBody(
        transmissionRatio: 3,
        mechanicalReductionRatio: 4,
        efficiency: 0.5,
        jointLowerLimit: -1,
        jointUpperLimit: 1,
        jointHomePosition: 0.05
    )
    let request = ArticulatedRigidBodySimulationRequest(
        body: body,
        world: articulatedSnapshotWorld(),
        embodiment: articulatedTransmissionMappedEmbodiment(
            actuatorLimits: ActuatorLimits(min: -0.6, max: 0.1, rateLimitPerSecond: 1_000)
        ),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [-0.6])
    )
    let step = try #require(log.events.last)
    let jointTorque = try #require(step.plantState.scalars["torque_mapped_joint"])
    let actuatorTorque = try #require(step.plantState.scalars["torque_actuator.mapped"])

    #expect(abs(jointTorque) > 1e-9)
    expectApproximatelyEqual(actuatorTorque, -jointTorque / 2, tolerance: 1e-9)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorHonorsZeroJointEffortLimit() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedTransmissionMappedBody(
            jointLowerLimit: -1,
            jointUpperLimit: 1,
            jointHomePosition: 0,
            jointEffortLimit: 0
        ),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedTransmissionMappedEmbodiment(
            actuatorLimits: ActuatorLimits(min: -0.6, max: 0.2, rateLimitPerSecond: 1_000)
        ),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [-0.6])
    )
    let step = try #require(log.events.last)
    let jointPosition = try #require(step.plantState.scalars["mapped_joint"])
    let jointTorque = try #require(step.plantState.scalars["torque_mapped_joint"])

    expectApproximatelyEqual(jointPosition, 0, tolerance: 1e-12)
    expectApproximatelyEqual(jointTorque, 0, tolerance: 1e-12)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorUsesMassForPrismaticEffectiveInertia() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedTransmissionMappedBody(
            transmissionRatio: 1,
            jointLowerLimit: -1,
            jointUpperLimit: 2,
            jointHomePosition: 0,
            jointEffortLimit: 1,
            commandDirection: 1,
            actuatorZeroOffset: 0,
            jointZeroOffset: 0,
            mappedLinkMass: 0.2
        ),
        world: articulatedSnapshotWorld(substeps: 1),
        embodiment: articulatedTransmissionMappedEmbodiment(
            actuatorLimits: ActuatorLimits(min: 0, max: 1, rateLimitPerSecond: 1_000),
            timeConstant: 0.01,
            torqueLimit: 1
        ),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [1])
    )
    let step = try #require(log.events.last)
    let jointPosition = try #require(step.plantState.scalars["mapped_joint"])
    let jointVelocity = try #require(step.plantState.scalars["velocity_mapped_joint"])
    let jointTorque = try #require(step.plantState.scalars["torque_mapped_joint"])

    expectApproximatelyEqual(jointTorque, 1, tolerance: 1e-12)
    expectApproximatelyEqual(jointVelocity, 0.05, tolerance: 1e-12)
    expectApproximatelyEqual(jointPosition, 0.0005, tolerance: 1e-12)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorRejectsWorldTimeStepMismatch() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.02,
        timeStep: try TimeStep(delta: 0.02)
    )

    await #expect(throws: ArticulatedRigidBodySimulator.SimulationError.timeStepMismatch(
        request: 0.02,
        world: 0.01
    )) {
        _ = try await ArticulatedRigidBodySimulator().run(request: request)
    }
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorRejectsNonIntegralDurationStepCount() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.015,
        timeStep: try TimeStep(delta: 0.01)
    )

    await #expect(throws: ArticulatedRigidBodySimulator.SimulationError.durationStepMismatch(
        duration: 0.015,
        timeStep: 0.01
    )) {
        _ = try await ArticulatedRigidBodySimulator().run(request: request)
    }
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorRejectsNumericallyUnstableSubstep() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0),
        world: articulatedSnapshotWorld(substeps: 1),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    await #expect(throws: ArticulatedRigidBodySimulator.SimulationError.unstableTimeStep(
        substep: 0.01,
        timeConstant: 0.001,
        actuator: "slide-actuator"
    )) {
        _ = try await ArticulatedRigidBodySimulator().run(request: request)
    }
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorRejectsSubstepAboveDeclaredActuatorTimeConstant() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(slideTimeConstant: 0.0005),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    await #expect(throws: ArticulatedRigidBodySimulator.SimulationError.unstableTimeStep(
        substep: 0.001,
        timeConstant: 0.0005,
        actuator: "slide-actuator"
    )) {
        _ = try await ArticulatedRigidBodySimulator().run(request: request)
    }
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorRejectsUnsupportedIntegrator() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0),
        world: articulatedSnapshotWorld(integrator: .rk4),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    await #expect(throws: ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("integrator.rk4")) {
        _ = try await ArticulatedRigidBodySimulator().run(request: request)
    }
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorReplayIsByteStable() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.10,
        timeStep: try TimeStep(delta: 0.01),
        seed: ScenarioSeed(42)
    )

    let first = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [0.4, 0.2])
    )
    let second = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [0.4, 0.2])
    )
    let encoder = JSONEncoder()
    encoder.outputFormatting = [.sortedKeys]

    #expect(first == second)
    #expect(try encoder.encode(first) == encoder.encode(second))
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorConfigHashBindsDescriptorAndProvider() async throws {
    let baseRequest = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01),
        seed: ScenarioSeed(42)
    )
    let changedBodyRequest = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(
            shoulderPosition: 0.2,
            slidePosition: 0.4,
            fixedRoll: 0.0,
            armMass: 0.6
        ),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01),
        seed: ScenarioSeed(42)
    )
    let provider = FixedArticulatedDriveProvider(activations: [0.4, 0.2])

    let first = try await ArticulatedRigidBodySimulator().run(request: baseRequest, driveProvider: provider)
    let second = try await ArticulatedRigidBodySimulator().run(request: baseRequest, driveProvider: provider)
    let changed = try await ArticulatedRigidBodySimulator().run(request: changedBodyRequest, driveProvider: provider)
    let defaultProviderLog = try await ArticulatedRigidBodySimulator().run(request: baseRequest)

    expectHexConfigHash(first.configHash)
    #expect(first.configHash == second.configHash)
    #expect(first.configHash != changed.configHash)
    #expect(first.configHash != defaultProviderLog.configHash)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorPerformanceBudgetForTwoThousandSteps() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0),
        world: articulatedSnapshotWorld(),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 20.0,
        timeStep: try TimeStep(delta: 0.01)
    )

    let strictTarget = strictPerformanceBudgetTarget(2_000)
    let measurement = try await bestSimulationThroughput(
        target: strictTarget ?? 0,
        unitCount: { $0.events.count }
    ) {
        try await ArticulatedRigidBodySimulator().run(
            request: request,
            driveProvider: FixedArticulatedDriveProvider(activations: [0.4, 0.2])
        )
    }

    #expect(measurement.log.events.count == 2_000)
    if let strictTarget {
        #expect(measurement.unitsPerSecond >= strictTarget)
    } else {
        #expect(measurement.unitsPerSecond > 0)
    }
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorSolvesDescriptorDynamicsContactConstraintEndToEnd() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedContactBody(),
        world: articulatedContactWorld(),
        embodiment: articulatedContactEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        readinessLevel: .contactTraining,
        duration: 0.10,
        timeStep: try TimeStep(delta: 0.005),
        seed: ScenarioSeed(7)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [-0.10])
    )
    let finalStep = try #require(log.events.last)
    let verticalPosition = try #require(finalStep.plantState.scalars["vertical_joint"])
    let penetration = try #require(finalStep.plantState.scalars["contact.penetration.max"])
    let activeContacts = try #require(finalStep.plantState.scalars["contact.active.count"])
    let solverIterations = try #require(finalStep.plantState.scalars["contact.solver.iterations"])
    let snapshots = Dictionary(uniqueKeysWithValues: finalStep.plantState.bodies.map { ($0.id, $0) })
    let foot = try #require(snapshots["foot"])

    #expect(log.events.count == 20)
    #expect(verticalPosition >= 0.05 - 1e-9)
    #expect(foot.position.z - 0.05 >= -1e-9)
    #expect(activeContacts >= 1)
    #expect(penetration <= 1e-9)
    #expect(solverIterations >= 1)
}

@Test(.timeLimit(.minutes(1))) func articulatedContactProjectionWeightsCorrectionByEffectiveInertia() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedTwoAxisContactBody(),
        world: articulatedContactWorld(),
        embodiment: articulatedTwoAxisContactEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        readinessLevel: .contactTraining,
        duration: 0.005,
        timeStep: try TimeStep(delta: 0.005),
        seed: ScenarioSeed(17)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [-0.05, -0.05])
    )
    let finalStep = try #require(log.events.last)
    let heavyPosition = try #require(finalStep.plantState.scalars["heavy_joint"])
    let lightPosition = try #require(finalStep.plantState.scalars["light_joint"])
    let penetration = try #require(finalStep.plantState.scalars["contact.penetration.max"])
    let heavyCorrection = heavyPosition + 0.05
    let lightCorrection = lightPosition + 0.05

    #expect(penetration <= 1e-9)
    #expect(heavyCorrection >= 0)
    #expect(heavyCorrection < 0.001)
    #expect(lightCorrection > 0.14)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorAppliesDescriptorPenaltyContactForceEndToEnd() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedContactBody(),
        world: articulatedContactWorld(
            solver: SolverModel(kind: .deterministicConstraint, iterations: 20, tolerance: 1e-9),
            contact: ContactModel(mode: .penalty, stiffness: 500, damping: 20)
        ),
        embodiment: articulatedContactEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        readinessLevel: .contactTraining,
        duration: 0.02,
        timeStep: try TimeStep(delta: 0.005),
        seed: ScenarioSeed(8)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [-0.10])
    )
    let firstStep = try #require(log.events.first)
    let finalStep = try #require(log.events.last)
    let firstForce = try #require(firstStep.plantState.scalars["contact.normalForce.max"])
    let firstImpulse = try #require(firstStep.plantState.scalars["contact.normalImpulse.max"])
    let finalPosition = try #require(finalStep.plantState.scalars["vertical_joint"])

    #expect(firstForce > 0)
    #expect(firstImpulse > 0)
    #expect(finalPosition > -0.10)
    #expect(finalPosition.isFinite)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorComposesWorldSurfaceAndGeometryPoseForContactPlane() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedContactBody(),
        world: articulatedContactWorld(
            surfacePose: KuyuPose(xyz: KuyuVector3(x: 0, y: 0, z: -0.05)),
            surfaceGeometryPose: KuyuPose(xyz: KuyuVector3(x: 0, y: 0, z: 0.025))
        ),
        embodiment: articulatedContactEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        readinessLevel: .contactTraining,
        duration: 0.05,
        timeStep: try TimeStep(delta: 0.005),
        seed: ScenarioSeed(9)
    )

    let log = try await ArticulatedRigidBodySimulator().run(
        request: request,
        driveProvider: FixedArticulatedDriveProvider(activations: [-0.10])
    )
    let finalStep = try #require(log.events.last)
    let verticalPosition = try #require(finalStep.plantState.scalars["vertical_joint"])
    let penetration = try #require(finalStep.plantState.scalars["contact.penetration.max"])

    expectApproximatelyEqual(verticalPosition, 0.05, tolerance: 1e-9)
    #expect(penetration <= 1e-9)
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorContactConstraintReplayIsByteStableForOneHundredRuns() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedContactBody(),
        world: articulatedContactWorld(),
        embodiment: articulatedContactEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        readinessLevel: .contactTraining,
        duration: 0.05,
        timeStep: try TimeStep(delta: 0.005),
        seed: ScenarioSeed(100)
    )
    let encoder = JSONEncoder()
    encoder.outputFormatting = [.sortedKeys]
    var referenceLog: SimulationLog?
    var referenceData: Data?

    for _ in 0..<100 {
        let log = try await ArticulatedRigidBodySimulator().run(
            request: request,
            driveProvider: FixedArticulatedDriveProvider(activations: [-0.10])
        )
        let data = try encoder.encode(log)
        if let referenceLog, let referenceData {
            #expect(log == referenceLog)
            #expect(data == referenceData)
        } else {
            referenceLog = log
            referenceData = data
        }
    }
}

@Test(.timeLimit(.minutes(1))) func articulatedSimulatorRejectsUnsupportedContactWorld() async throws {
    let request = ArticulatedRigidBodySimulationRequest(
        body: articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0),
        world: articulatedSnapshotWorld(
            solver: SolverModel(kind: .deterministicConstraint, iterations: 20, tolerance: 1e-8),
            contact: ContactModel(mode: .penalty, stiffness: 1_000, damping: 10)
        ),
        embodiment: articulatedSnapshotEmbodiment(),
        determinism: try DeterminismConfig(tier: .tier0),
        duration: 0.01,
        timeStep: try TimeStep(delta: 0.01)
    )

    await #expect(throws: ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld(
        "contact.surfaces"
    )) {
        _ = try await ArticulatedRigidBodySimulator().run(request: request)
    }
}

@Test func readinessRejectsActuatorLimitsOutsideJointEnvelope() throws {
    let body = articulatedSnapshotBody(shoulderPosition: 0.2, slidePosition: 0.4, fixedRoll: 0.0)
    let embodiment = articulatedSnapshotEmbodiment(
        slideActuatorLimits: ActuatorLimits(min: -1.0, max: 1.0, rateLimitPerSecond: 1_000)
    )

    #expect(throws: KuyuModelValidationError.invalidRange("readiness.dynamic.actuators.slide-actuator.limits")) {
        _ = try ReadinessGate().validate(
            body: body,
            world: articulatedSnapshotWorld(),
            embodiment: embodiment,
            report: nil,
            requiredLevel: .dynamicSimulation
        )
    }
}

@Test func readinessRejectsMappedActuatorLimitsOutsideJointEnvelope() throws {
    let embodiment = articulatedTransmissionMappedEmbodiment(
        actuatorLimits: ActuatorLimits(min: -0.20, max: -0.10, rateLimitPerSecond: 1_000)
    )

    #expect(throws: KuyuModelValidationError.invalidRange("readiness.dynamic.actuators.mapped-actuator.limits")) {
        _ = try ReadinessGate().validate(
            body: articulatedTransmissionMappedBody(),
            world: articulatedSnapshotWorld(),
            embodiment: embodiment,
            report: nil,
            requiredLevel: .dynamicSimulation
        )
    }
}

private func articulatedSnapshotBody(
    shoulderPosition: Double,
    slidePosition: Double,
    fixedRoll: Double,
    activeJointOrder: [String] = ["slide_joint", "shoulder_joint"],
    armMass: Double = 0.4
) -> KuyuBodyModel {
    let tipFixedJoint = JointDefinition(
        id: "tip_fixed",
        kind: .fixed,
        parentLinkID: "slider",
        childLinkID: "tip",
        origin: KuyuPose(
            xyz: KuyuVector3(x: 0, y: 1, z: 0),
            rpy: KuyuVector3(x: fixedRoll, y: 0, z: 0)
        ),
        axis: KuyuVector3(x: 0, y: 0, z: 0)
    )
    let slideJoint = JointDefinition(
        id: "slide_joint",
        kind: .prismatic,
        parentLinkID: "arm",
        childLinkID: "slider",
        origin: KuyuPose(xyz: KuyuVector3(x: 1, y: 0, z: 0)),
        axis: KuyuVector3(x: 1, y: 0, z: 0),
        lowerLimit: slidePosition,
        upperLimit: slidePosition,
        effortLimit: 1_000,
        velocityLimit: 1_000
    )
    let shoulderJoint = JointDefinition(
        id: "shoulder_joint",
        kind: .revolute,
        parentLinkID: "base",
        childLinkID: "arm",
        origin: KuyuPose(),
        axis: KuyuVector3(x: 0, y: 0, z: 1),
        lowerLimit: shoulderPosition,
        upperLimit: shoulderPosition,
        effortLimit: 1_000,
        velocityLimit: 1_000
    )
    let activeJointsByID = Dictionary(uniqueKeysWithValues: [
        (slideJoint.id, slideJoint),
        (shoulderJoint.id, shoulderJoint),
    ])
    let activeJoints = activeJointOrder.compactMap { activeJointsByID[$0] }

    return KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "articulated-snapshot-body",
        name: "Articulated Snapshot Body",
        category: "manipulator",
        frames: [
            FrameDefinition(id: "shoulder-output", parentID: "base", pose: KuyuPose()),
            FrameDefinition(id: "slide-output", parentID: "arm", pose: KuyuPose())
        ],
        links: [
            articulatedLink(id: "base", mass: 1),
            articulatedLink(id: "arm", mass: armMass),
            articulatedLink(id: "slider", mass: 0.2),
            articulatedLink(id: "tip", mass: 0.1)
        ],
        joints: [tipFixedJoint] + activeJoints,
        actuatorMounts: [
            ActuatorMount(
                actuatorID: "slide-actuator",
                parentLinkID: "arm",
                frameID: "slide-output",
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 1, y: 0, z: 0)
            ),
            ActuatorMount(
                actuatorID: "shoulder-actuator",
                parentLinkID: "base",
                frameID: "shoulder-output",
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 0, y: 0, z: 1)
            )
        ],
        actuatorAttachments: [
            ActuatorAttachment(
                actuatorID: "slide-actuator",
                jointID: "slide_joint",
                torqueLimit: 1_000,
                mountFrameID: "slide-output"
            ),
            ActuatorAttachment(
                actuatorID: "shoulder-actuator",
                jointID: "shoulder_joint",
                torqueLimit: 1_000,
                mountFrameID: "shoulder-output"
            )
        ]
    )
}

private func articulatedBranchedSnapshotBody(
    leftAngle: Double,
    rightSlide: Double,
    fixedPitch: Double
) -> KuyuBodyModel {
    KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "articulated-branched-snapshot-body",
        name: "Articulated Branched Snapshot Body",
        category: "manipulator",
        frames: [
            FrameDefinition(id: "left-output", parentID: "base", pose: KuyuPose()),
            FrameDefinition(id: "right-output", parentID: "base", pose: KuyuPose())
        ],
        links: [
            articulatedLink(id: "base", mass: 1),
            articulatedLink(id: "left", mass: 0.4),
            articulatedLink(id: "left_tip", mass: 0.1),
            articulatedLink(id: "right", mass: 0.2)
        ],
        joints: [
            JointDefinition(
                id: "left_tip_fixed",
                kind: .fixed,
                parentLinkID: "left",
                childLinkID: "left_tip",
                origin: KuyuPose(
                    xyz: KuyuVector3(x: 0, y: 1, z: 0),
                    rpy: KuyuVector3(x: 0, y: fixedPitch, z: 0)
                ),
                axis: KuyuVector3(x: 0, y: 0, z: 0)
            ),
            JointDefinition(
                id: "right_slide",
                kind: .prismatic,
                parentLinkID: "base",
                childLinkID: "right",
                origin: KuyuPose(xyz: KuyuVector3(x: 0, y: -1, z: 0)),
                axis: KuyuVector3(x: 0, y: 1, z: 0),
                lowerLimit: rightSlide,
                upperLimit: rightSlide,
                effortLimit: 1_000,
                velocityLimit: 1_000,
                homePosition: rightSlide
            ),
            JointDefinition(
                id: "left_yaw",
                kind: .revolute,
                parentLinkID: "base",
                childLinkID: "left",
                origin: KuyuPose(xyz: KuyuVector3(x: 1, y: 0, z: 0)),
                axis: KuyuVector3(x: 0, y: 0, z: 1),
                lowerLimit: leftAngle,
                upperLimit: leftAngle,
                effortLimit: 1_000,
                velocityLimit: 1_000,
                homePosition: leftAngle
            )
        ],
        actuatorMounts: [
            ActuatorMount(
                actuatorID: "right-actuator",
                parentLinkID: "base",
                frameID: "right-output",
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 0, y: 1, z: 0)
            ),
            ActuatorMount(
                actuatorID: "left-actuator",
                parentLinkID: "base",
                frameID: "left-output",
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 0, y: 0, z: 1)
            )
        ],
        actuatorAttachments: [
            ActuatorAttachment(
                actuatorID: "right-actuator",
                jointID: "right_slide",
                torqueLimit: 1_000,
                mountFrameID: "right-output"
            ),
            ActuatorAttachment(
                actuatorID: "left-actuator",
                jointID: "left_yaw",
                torqueLimit: 1_000,
                mountFrameID: "left-output"
            )
        ]
    )
}

private func articulatedLink(id: String, mass: Double) -> LinkDefinition {
    LinkDefinition(
        id: id,
        mass: mass,
        centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
        inertia: KuyuInertiaTensor(ixx: 0.01, ixy: 0, ixz: 0, iyy: 0.01, iyz: 0, izz: 0.01)
    )
}

private func articulatedTransmissionMappedBody(
    transmissionRatio: Double = 2,
    mechanicalReductionRatio: Double = 1,
    efficiency: Double? = nil,
    jointLowerLimit: Double = 0.15,
    jointUpperLimit: Double = 0.15,
    jointHomePosition: Double = 0.15,
    jointEffortLimit: Double = 1_000,
    commandDirection: Double = -1,
    actuatorZeroOffset: Double = 0.10,
    jointZeroOffset: Double = 0.05,
    mappedLinkMass: Double = 0.2
) -> KuyuBodyModel {
    KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "articulated-transmission-mapped-body",
        name: "Articulated Transmission Mapped Body",
        category: "manipulator",
        frames: [
            FrameDefinition(id: "mapped-output", parentID: "base", pose: KuyuPose())
        ],
        links: [
            articulatedLink(id: "base", mass: 1),
            articulatedLink(id: "mapped-link", mass: mappedLinkMass)
        ],
        joints: [
            JointDefinition(
                id: "mapped_joint",
                kind: .prismatic,
                parentLinkID: "base",
                childLinkID: "mapped-link",
                origin: KuyuPose(),
                axis: KuyuVector3(x: 0, y: 0, z: 1),
                lowerLimit: jointLowerLimit,
                upperLimit: jointUpperLimit,
                effortLimit: jointEffortLimit,
                velocityLimit: 1_000,
                homePosition: jointHomePosition
            )
        ],
        actuatorMounts: [
            ActuatorMount(
                actuatorID: "mapped-actuator",
                parentLinkID: "base",
                frameID: "mapped-output",
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 0, y: 0, z: 1)
            )
        ],
        actuatorAttachments: [
            ActuatorAttachment(
                actuatorID: "mapped-actuator",
                jointID: "mapped_joint",
                transmissionRatio: transmissionRatio,
                torqueLimit: 1_000,
                mountFrameID: "mapped-output",
                mechanicalReductionRatio: mechanicalReductionRatio,
                commandDirection: commandDirection,
                actuatorZeroOffset: actuatorZeroOffset,
                jointZeroOffset: jointZeroOffset,
                efficiency: efficiency
            )
        ]
    )
}

private func articulatedSnapshotWorld(
    integrator: IntegratorKind = .semiImplicitEuler,
    solver: SolverModel = SolverModel(kind: .disabledContact, iterations: 0, tolerance: 0),
    contact: ContactModel = ContactModel(mode: .disabled),
    substeps: Int = 10
) -> KuyuWorldModel {
    KuyuWorldModel(
        schemaVersion: "kuyu.world.v1",
        worldID: "articulated-snapshot-world",
        time: TimeModel(fixedStepSeconds: 0.01, substeps: substeps),
        integrator: IntegratorModel(kind: integrator),
        solver: solver,
        gravity: .earthUniform,
        atmosphere: AtmosphereModel(kind: .none),
        wind: WindModel(kind: .none),
        contact: contact,
        nap: NegligibilityApproximationPolicy(
            forceAbsoluteThreshold: 0,
            forceRelativeThreshold: 0,
            torqueAbsoluteThreshold: 0,
            torqueRelativeThreshold: 0
        ),
        randomness: RandomnessModel(seed: 0, deterministicReplay: true)
    )
}

private func articulatedContactBody() -> KuyuBodyModel {
    KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "articulated-contact-body",
        name: "Articulated Contact Body",
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
        materials: [
            BodyMaterial(id: "rubber", density: 1_000, staticFriction: 0.8, dynamicFriction: 0.6, restitution: 0)
        ],
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
}

private func articulatedTwoAxisContactBody() -> KuyuBodyModel {
    KuyuBodyModel(
        schemaVersion: "kuyu.body.v1",
        bodyID: "articulated-two-axis-contact-body",
        name: "Articulated Two Axis Contact Body",
        category: "contact-fixture",
        frames: [
            FrameDefinition(id: "heavy-output", parentID: "base", pose: KuyuPose()),
            FrameDefinition(id: "light-output", parentID: "mid", pose: KuyuPose())
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
                id: "mid",
                mass: 0.2,
                centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
                inertia: KuyuInertiaTensor(ixx: 0.01, ixy: 0, ixz: 0, iyy: 0.01, iyz: 0, izz: 0.01),
                collisions: [
                    GeometryInstance(
                        id: "mid-clearance",
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
                id: "heavy_joint",
                kind: .prismatic,
                parentLinkID: "base",
                childLinkID: "mid",
                origin: KuyuPose(),
                axis: KuyuVector3(x: 0, y: 0, z: 1),
                lowerLimit: -0.2,
                upperLimit: 0.2,
                effortLimit: 0,
                velocityLimit: 1_000,
                homePosition: -0.05
            ),
            JointDefinition(
                id: "light_joint",
                kind: .prismatic,
                parentLinkID: "mid",
                childLinkID: "foot",
                origin: KuyuPose(),
                axis: KuyuVector3(x: 0, y: 0, z: 1),
                lowerLimit: -0.2,
                upperLimit: 0.2,
                effortLimit: 0,
                velocityLimit: 1_000,
                homePosition: -0.05
            )
        ],
        materials: [
            BodyMaterial(id: "rubber", density: 1_000, staticFriction: 0.8, dynamicFriction: 0.6, restitution: 0)
        ],
        actuatorMounts: [
            ActuatorMount(
                actuatorID: "heavy-actuator",
                parentLinkID: "base",
                frameID: "heavy-output",
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 0, y: 0, z: 1)
            ),
            ActuatorMount(
                actuatorID: "light-actuator",
                parentLinkID: "mid",
                frameID: "light-output",
                pose: KuyuPose(),
                outputAxis: KuyuVector3(x: 0, y: 0, z: 1)
            )
        ],
        actuatorAttachments: [
            ActuatorAttachment(
                actuatorID: "heavy-actuator",
                jointID: "heavy_joint",
                torqueLimit: 1_000,
                mountFrameID: "heavy-output",
                reflectedInertia: 100
            ),
            ActuatorAttachment(
                actuatorID: "light-actuator",
                jointID: "light_joint",
                torqueLimit: 1_000,
                mountFrameID: "light-output"
            )
        ]
    )
}

private func articulatedContactWorld(
    solver: SolverModel = SolverModel(kind: .deterministicConstraint, iterations: 20, tolerance: 1e-9),
    contact: ContactModel = ContactModel(mode: .constraint),
    surfacePose: KuyuPose = KuyuPose(xyz: KuyuVector3(x: 0, y: 0, z: -0.025)),
    surfaceGeometryPose: KuyuPose = KuyuPose()
) -> KuyuWorldModel {
    KuyuWorldModel(
        schemaVersion: "kuyu.world.v1",
        worldID: "articulated-contact-world",
        time: TimeModel(fixedStepSeconds: 0.005, substeps: 5),
        integrator: IntegratorModel(kind: .semiImplicitEuler),
        solver: solver,
        gravity: .earthUniform,
        atmosphere: AtmosphereModel(kind: .none),
        wind: WindModel(kind: .none),
        surfaces: [
            WorldSurface(
                id: "floor",
                frameID: "world",
                materialID: "floor",
                pose: surfacePose,
                geometry: GeometryInstance(
                    id: "floor-box",
                    kind: .box,
                    pose: surfaceGeometryPose,
                    size: KuyuVector3(x: 10, y: 10, z: 0.05)
                )
            )
        ],
        materials: [
            WorldMaterial(id: "floor", staticFriction: 0.7, dynamicFriction: 0.5, restitution: 0)
        ],
        contact: contact,
        nap: NegligibilityApproximationPolicy(
            forceAbsoluteThreshold: 0,
            forceRelativeThreshold: 0,
            torqueAbsoluteThreshold: 0,
            torqueRelativeThreshold: 0
        ),
        randomness: RandomnessModel(seed: 0, deterministicReplay: true)
    )
}

private func articulatedTransmissionMappedEmbodiment(
    actuatorLimits: ActuatorLimits = ActuatorLimits(min: -0.10, max: -0.10, rateLimitPerSecond: 1_000),
    timeConstant: Double = 0.001,
    torqueLimit: Double = 1_000
) -> EmbodimentContract {
    EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "articulated-transmission-mapped-contract",
        bodyID: "articulated-transmission-mapped-body",
        signals: SignalCatalog(
            sensor: [],
            actuator: [
                SignalDefinition(id: "actuator.mapped", index: 0, name: "Mapped actuator", units: "encoder")
            ],
            drive: [
                SignalDefinition(
                    id: "drive.mapped",
                    index: 0,
                    name: "Mapped drive",
                    units: "encoder",
                    range: ScalarRange(min: actuatorLimits.min, max: actuatorLimits.max)
                )
            ],
            reflex: [
                SignalDefinition(id: "reflex.mapped", index: 0, name: "Mapped reflex", units: "encoder")
            ]
        ),
        sensors: [],
        actuators: [
            ActuatorDefinition(
                id: "mapped-actuator",
                type: "linear-servo",
                frameID: "mapped-output",
                channels: ["actuator.mapped"],
                limits: actuatorLimits,
                dynamics: ActuatorDynamics(timeConstantSeconds: timeConstant, deadzone: 0, torqueLimit: torqueLimit)
            )
        ],
        control: ControlContract(driveChannels: ["drive.mapped"], reflexChannels: ["reflex.mapped"]),
        motorNerve: MotorNerveContract(stages: [
            MotorNerveStageDefinition(
                id: "direct",
                type: .direct,
                inputs: ["drive.mapped"],
                outputs: ["actuator.mapped"]
            )
        ])
    )
}

private func articulatedContactEmbodiment() -> EmbodimentContract {
    EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "articulated-contact-contract",
        bodyID: "articulated-contact-body",
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
}

private func articulatedTwoAxisContactEmbodiment() -> EmbodimentContract {
    EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "articulated-two-axis-contact-contract",
        bodyID: "articulated-two-axis-contact-body",
        signals: SignalCatalog(
            sensor: [],
            actuator: [
                SignalDefinition(id: "actuator.heavy", index: 0, name: "Heavy actuator", units: "m"),
                SignalDefinition(id: "actuator.light", index: 1, name: "Light actuator", units: "m")
            ],
            drive: [
                SignalDefinition(
                    id: "drive.heavy",
                    index: 0,
                    name: "Heavy drive",
                    units: "m",
                    range: ScalarRange(min: -0.2, max: 0.2)
                ),
                SignalDefinition(
                    id: "drive.light",
                    index: 1,
                    name: "Light drive",
                    units: "m",
                    range: ScalarRange(min: -0.2, max: 0.2)
                )
            ],
            reflex: [
                SignalDefinition(id: "reflex.heavy", index: 0, name: "Heavy reflex", units: "m"),
                SignalDefinition(id: "reflex.light", index: 1, name: "Light reflex", units: "m")
            ]
        ),
        sensors: [],
        actuators: [
            ActuatorDefinition(
                id: "heavy-actuator",
                type: "linear-servo",
                frameID: "heavy-output",
                channels: ["actuator.heavy"],
                limits: ActuatorLimits(min: -0.2, max: 0.2, rateLimitPerSecond: 1_000),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.001, deadzone: 0)
            ),
            ActuatorDefinition(
                id: "light-actuator",
                type: "linear-servo",
                frameID: "light-output",
                channels: ["actuator.light"],
                limits: ActuatorLimits(min: -0.2, max: 0.2, rateLimitPerSecond: 1_000),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.001, deadzone: 0)
            )
        ],
        control: ControlContract(driveChannels: ["drive.heavy", "drive.light"], reflexChannels: ["reflex.heavy", "reflex.light"]),
        motorNerve: MotorNerveContract(stages: [
            MotorNerveStageDefinition(
                id: "direct",
                type: .direct,
                inputs: ["drive.heavy", "drive.light"],
                outputs: ["actuator.heavy", "actuator.light"]
            )
        ])
    )
}

private func articulatedBranchedSnapshotEmbodiment(
    leftAngle: Double,
    rightSlide: Double
) -> EmbodimentContract {
    EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "articulated-branched-snapshot-contract",
        bodyID: "articulated-branched-snapshot-body",
        signals: SignalCatalog(
            sensor: [],
            actuator: [
                SignalDefinition(id: "actuator.right", index: 0, name: "Right actuator", units: "m"),
                SignalDefinition(id: "actuator.left", index: 1, name: "Left actuator", units: "rad")
            ],
            drive: [
                SignalDefinition(
                    id: "drive.right",
                    index: 0,
                    name: "Right drive",
                    units: "m",
                    range: ScalarRange(min: rightSlide, max: rightSlide)
                ),
                SignalDefinition(
                    id: "drive.left",
                    index: 1,
                    name: "Left drive",
                    units: "rad",
                    range: ScalarRange(min: leftAngle, max: leftAngle)
                )
            ],
            reflex: [
                SignalDefinition(id: "reflex.right", index: 0, name: "Right reflex", units: "m"),
                SignalDefinition(id: "reflex.left", index: 1, name: "Left reflex", units: "rad")
            ]
        ),
        sensors: [],
        actuators: [
            ActuatorDefinition(
                id: "right-actuator",
                type: "linear-servo",
                frameID: "right-output",
                channels: ["actuator.right"],
                limits: ActuatorLimits(min: rightSlide, max: rightSlide, rateLimitPerSecond: 1_000),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.001, deadzone: 0, torqueLimit: 1_000)
            ),
            ActuatorDefinition(
                id: "left-actuator",
                type: "servo",
                frameID: "left-output",
                channels: ["actuator.left"],
                limits: ActuatorLimits(min: leftAngle, max: leftAngle, rateLimitPerSecond: 1_000),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.001, deadzone: 0, torqueLimit: 1_000)
            )
        ],
        control: ControlContract(
            driveChannels: ["drive.right", "drive.left"],
            reflexChannels: ["reflex.right", "reflex.left"]
        ),
        motorNerve: MotorNerveContract(stages: [
            MotorNerveStageDefinition(
                id: "direct",
                type: .direct,
                inputs: ["drive.right", "drive.left"],
                outputs: ["actuator.right", "actuator.left"]
            )
        ])
    )
}

private func articulatedSnapshotEmbodiment(
    slideActuatorLimits: ActuatorLimits = ActuatorLimits(min: 0.4, max: 0.4, rateLimitPerSecond: 1_000),
    shoulderActuatorLimits: ActuatorLimits = ActuatorLimits(min: 0.2, max: 0.2, rateLimitPerSecond: 1_000),
    slideTimeConstant: Double = 0.001,
    shoulderTimeConstant: Double = 0.001
) -> EmbodimentContract {
    EmbodimentContract(
        schemaVersion: "embodiment.contract.v1",
        contractID: "articulated-snapshot-contract",
        bodyID: "articulated-snapshot-body",
        signals: SignalCatalog(
            sensor: [],
            actuator: [
                SignalDefinition(id: "actuator.slide", index: 0, name: "Slide actuator", units: "m"),
                SignalDefinition(id: "actuator.shoulder", index: 1, name: "Shoulder actuator", units: "rad")
            ],
            drive: [
                SignalDefinition(
                    id: "drive.slide",
                    index: 0,
                    name: "Slide drive",
                    units: "m",
                    range: ScalarRange(min: 0.4, max: 0.4)
                ),
                SignalDefinition(
                    id: "drive.shoulder",
                    index: 1,
                    name: "Shoulder drive",
                    units: "rad",
                    range: ScalarRange(min: 0.2, max: 0.2)
                )
            ],
            reflex: [
                SignalDefinition(id: "reflex.slide", index: 0, name: "Slide reflex", units: "m"),
                SignalDefinition(id: "reflex.shoulder", index: 1, name: "Shoulder reflex", units: "rad")
            ]
        ),
        sensors: [],
        actuators: [
            ActuatorDefinition(
                id: "slide-actuator",
                type: "linear-servo",
                frameID: "slide-output",
                channels: ["actuator.slide"],
                limits: slideActuatorLimits,
                dynamics: ActuatorDynamics(timeConstantSeconds: slideTimeConstant, deadzone: 0, torqueLimit: 1_000)
            ),
            ActuatorDefinition(
                id: "shoulder-actuator",
                type: "servo",
                frameID: "shoulder-output",
                channels: ["actuator.shoulder"],
                limits: shoulderActuatorLimits,
                dynamics: ActuatorDynamics(timeConstantSeconds: shoulderTimeConstant, deadzone: 0, torqueLimit: 1_000)
            )
        ],
        control: ControlContract(
            driveChannels: ["drive.slide", "drive.shoulder"],
            reflexChannels: ["reflex.slide", "reflex.shoulder"]
        ),
        motorNerve: MotorNerveContract(stages: [
            MotorNerveStageDefinition(
                id: "direct",
                type: .direct,
                inputs: ["drive.slide", "drive.shoulder"],
                outputs: ["actuator.slide", "actuator.shoulder"]
            )
        ])
    )
}

private func axis3(_ vector: SIMD3<Double>) -> Axis3 {
    Axis3(x: vector.x, y: vector.y, z: vector.z)
}

private func assertAxisApproximatelyEqual(_ lhs: Axis3, _ rhs: Axis3, tolerance: Double) {
    #expect(abs(lhs.x - rhs.x) < tolerance)
    #expect(abs(lhs.y - rhs.y) < tolerance)
    #expect(abs(lhs.z - rhs.z) < tolerance)
}

private func expectApproximatelyEqual(_ lhs: Double, _ rhs: Double, tolerance: Double) {
    #expect(abs(lhs - rhs) <= tolerance)
}

private func expectHexConfigHash(_ hash: String) {
    let hexCharacters = Set("0123456789abcdef")
    #expect(hash.count == 16)
    #expect(hash.allSatisfy { hexCharacters.contains($0) })
}

private func assertQuaternionApproximatelyEqual(
    _ lhs: QuaternionSnapshot,
    _ rhs: QuaternionSnapshot,
    tolerance: Double
) {
    #expect(abs(abs(lhs.dot(rhs)) - 1.0) < tolerance)
}

private struct FixedArticulatedDriveProvider: ArticulatedRigidBodyDriveProvider {
    let providerID = "fixed-articulated-drive-provider"
    let activations: [Double]

    mutating func driveIntents(context: ArticulatedRigidBodyDriveContext) throws -> [DriveIntent] {
        guard context.jointIDs.count == activations.count else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput(
                "test-drive-count expected=\(context.jointIDs.count) actual=\(activations.count)"
            )
        }
        return try activations.enumerated().map { index, value in
            try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
        }
    }
}

private struct ExpectedOrderDriveProvider: ArticulatedRigidBodyDriveProvider {
    let providerID = "expected-order-drive-provider"
    let expectedJointIDs: [String]
    let activations: [Double]

    mutating func reset(context: ArticulatedRigidBodyDriveProviderResetContext) throws {
        guard context.jointIDs == expectedJointIDs else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput(
                "reset-joint-order expected=\(expectedJointIDs) actual=\(context.jointIDs)"
            )
        }
    }

    mutating func driveIntents(context: ArticulatedRigidBodyDriveContext) throws -> [DriveIntent] {
        guard context.jointIDs == expectedJointIDs else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput(
                "step-joint-order expected=\(expectedJointIDs) actual=\(context.jointIDs)"
            )
        }
        return try activations.enumerated().map { index, value in
            try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
        }
    }
}

private struct ExpectedInitialStateDriveProvider: ArticulatedRigidBodyDriveProvider {
    let providerID = "expected-initial-state-drive-provider"
    let expectedPositions: [Double]
    let activations: [Double]

    mutating func driveIntents(context: ArticulatedRigidBodyDriveContext) throws -> [DriveIntent] {
        guard context.positions == expectedPositions else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput(
                "initial-positions expected=\(expectedPositions) actual=\(context.positions)"
            )
        }
        guard context.targets == expectedPositions else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput(
                "initial-targets expected=\(expectedPositions) actual=\(context.targets)"
            )
        }
        return try activations.enumerated().map { index, value in
            try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
        }
    }
}

private struct ExpectedApproximateInitialStateDriveProvider: ArticulatedRigidBodyDriveProvider {
    let providerID = "expected-approximate-initial-state-drive-provider"
    let expectedPositions: [Double]
    let activations: [Double]
    let tolerance: Double

    mutating func driveIntents(context: ArticulatedRigidBodyDriveContext) throws -> [DriveIntent] {
        guard context.positions.count == expectedPositions.count,
              context.targets.count == expectedPositions.count else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput("initial-state-count")
        }
        for index in expectedPositions.indices {
            guard abs(context.positions[index] - expectedPositions[index]) <= tolerance else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput(
                    "initial-positions expected=\(expectedPositions) actual=\(context.positions)"
                )
            }
            guard abs(context.targets[index] - expectedPositions[index]) <= tolerance else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidDriveProviderOutput(
                    "initial-targets expected=\(expectedPositions) actual=\(context.targets)"
                )
            }
        }
        return try activations.enumerated().map { index, value in
            try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
        }
    }
}
