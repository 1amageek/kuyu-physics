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

    #expect(log.configHash.contains("fixed-articulated-drive-provider"))
    #expect(first.driveIntents.map(\.activation) == [0.4, 0.2])
    #expect(first.driveIntents.map { Int($0.index.rawValue) } == [0, 1])
}

private func articulatedSnapshotBody(
    shoulderPosition: Double,
    slidePosition: Double,
    fixedRoll: Double
) -> KuyuBodyModel {
    KuyuBodyModel(
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
            articulatedLink(id: "arm", mass: 0.4),
            articulatedLink(id: "slider", mass: 0.2),
            articulatedLink(id: "tip", mass: 0.1)
        ],
        joints: [
            JointDefinition(
                id: "tip_fixed",
                kind: .fixed,
                parentLinkID: "slider",
                childLinkID: "tip",
                origin: KuyuPose(
                    xyz: KuyuVector3(x: 0, y: 1, z: 0),
                    rpy: KuyuVector3(x: fixedRoll, y: 0, z: 0)
                ),
                axis: KuyuVector3(x: 0, y: 0, z: 0)
            ),
            JointDefinition(
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
            ),
            JointDefinition(
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
        ],
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

private func articulatedLink(id: String, mass: Double) -> LinkDefinition {
    LinkDefinition(
        id: id,
        mass: mass,
        centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
        inertia: KuyuInertiaTensor(ixx: 0.01, ixy: 0, ixz: 0, iyy: 0.01, iyz: 0, izz: 0.01)
    )
}

private func articulatedSnapshotWorld() -> KuyuWorldModel {
    KuyuWorldModel(
        schemaVersion: "kuyu.world.v1",
        worldID: "articulated-snapshot-world",
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

private func articulatedSnapshotEmbodiment() -> EmbodimentContract {
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
                limits: ActuatorLimits(min: 0.4, max: 0.4, rateLimitPerSecond: 1_000),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.001, deadzone: 0, torqueLimit: 1_000)
            ),
            ActuatorDefinition(
                id: "shoulder-actuator",
                type: "servo",
                frameID: "shoulder-output",
                channels: ["actuator.shoulder"],
                limits: ActuatorLimits(min: 0.2, max: 0.2, rateLimitPerSecond: 1_000),
                dynamics: ActuatorDynamics(timeConstantSeconds: 0.001, deadzone: 0, torqueLimit: 1_000)
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
