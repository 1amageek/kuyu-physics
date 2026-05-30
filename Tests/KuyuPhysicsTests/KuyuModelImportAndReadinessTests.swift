import Foundation
import Testing
import KuyuPhysics

@Test func urdfBodyImporterRejectsMissingInertialInsteadOfSupplementing() throws {
    let url = try writeTemporaryFile(
        name: "missing-inertial.urdf",
        contents: """
        <robot name="missing">
          <link name="base"/>
        </robot>
        """
    )

    #expect(throws: URDFBodyImporter.ImportError.missingInertial("base")) {
        _ = try URDFBodyImporter().importBody(url: url, bodyID: "missing-body", category: "test")
    }
}

@Test func urdfBodyImporterMapsExpandedUrdfToBodyModelAndCompatibilityReport() throws {
    let url = try writeTemporaryFile(
        name: "mini.urdf",
        contents: """
        <robot name="mini">
          <link name="base">
            <inertial>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <mass value="1.0"/>
              <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry><box size="0.1 0.1 0.1"/></geometry>
            </visual>
          </link>
        </robot>
        """
    )

    let result = try URDFBodyImporter().importBody(url: url, bodyID: "mini-body", category: "fixture")

    #expect(result.body.bodyID == "mini-body")
    #expect(result.body.links.count == 1)
    #expect(result.body.links[0].mass == 1.0)
    #expect(result.report.mappings.contains { $0.status == .exact })
}

@Test func urdfBodyImporterRejectsMovableJointWithoutAxis() throws {
    let url = try writeTemporaryFile(
        name: "missing-axis.urdf",
        contents: """
        <robot name="missing-axis">
          <link name="base">
            <inertial>
              <mass value="1.0"/>
              <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
          </link>
          <link name="tip">
            <inertial>
              <mass value="1.0"/>
              <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
          </link>
          <joint name="joint_1" type="revolute">
            <parent link="base"/>
            <child link="tip"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>
        """
    )

    #expect(throws: URDFBodyImporter.ImportError.parseFailed("joint.axis.joint_1")) {
        _ = try URDFBodyImporter().importBody(url: url, bodyID: "missing-axis-body", category: "fixture")
    }
}

@Test func sdfWorldImporterMapsPhysicsAndContactDeclarations() throws {
    let url = try writeTemporaryFile(
        name: "mini.sdf",
        contents: """
        <sdf version="1.9">
          <world name="mini">
            <gravity>0 0 -9.8</gravity>
            <physics>
              <max_step_size>0.01</max_step_size>
            </physics>
            <surface>
              <friction>
                <ode>
                  <static_friction>0.7</static_friction>
                  <dynamic_friction>0.6</dynamic_friction>
                </ode>
              </friction>
              <contact>
                <stiffness>1000</stiffness>
                <damping>10</damping>
              </contact>
            </surface>
          </world>
        </sdf>
        """
    )

    let result = try SDFWorldImporter().importWorld(url: url, worldID: "mini-world")

    #expect(result.world.worldID == "mini-world")
    #expect(result.world.time.fixedStepSeconds == 0.01)
    #expect(result.world.contact.mode == .penalty)
    #expect(result.report.readinessLevel == .contactTraining)
}

@Test func sdfWorldImporterRejectsMissingRequiredPhysicsValues() throws {
    let url = try writeTemporaryFile(
        name: "missing-physics.sdf",
        contents: """
        <sdf version="1.9">
          <world name="missing">
            <physics/>
          </world>
        </sdf>
        """
    )

    #expect(throws: SDFWorldImporter.ImportError.parseFailed("physics.max_step_size")) {
        _ = try SDFWorldImporter().importWorld(url: url, worldID: "missing-world")
    }
}

@Test func kuyuModelLoaderRejectsMismatchedModelReferenceHash() throws {
    let directory = FileManager.default.temporaryDirectory.appendingPathComponent(
        "kuyu-loader-hash-\(UUID().uuidString)",
        isDirectory: true
    )
    try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
    let bodyURL = directory.appendingPathComponent("body.kuyubody.json", isDirectory: false)
    let worldURL = directory.appendingPathComponent("world.kuyuworld.json", isDirectory: false)
    let embodimentURL = directory.appendingPathComponent("embodiment.embodiment.json", isDirectory: false)
    let manifestURL = directory.appendingPathComponent("robot.kuyurobot.json", isDirectory: false)

    let encoder = JSONEncoder()
    try encoder.encode(minimalBody()).write(to: bodyURL, options: [.atomic])
    try encoder.encode(minimalWorld()).write(to: worldURL, options: [.atomic])
    try encoder.encode(minimalEmbodiment()).write(to: embodimentURL, options: [.atomic])
    let manifest = KuyuRobotManifest(
        schemaVersion: "1.0",
        robotID: "hash-fixture",
        name: "Hash Fixture",
        category: "fixture",
        bodyModel: ModelReference(path: "body.kuyubody.json", sha256: "00"),
        defaultWorldModel: ModelReference(path: "world.kuyuworld.json"),
        embodimentContract: ModelReference(path: "embodiment.embodiment.json")
    )
    try encoder.encode(manifest).write(to: manifestURL, options: [.atomic])

    #expect(throws: KuyuModelLoader.LoaderError.hashMismatch(bodyURL.path)) {
        _ = try KuyuModelLoader().loadRobot(path: manifestURL.path)
    }
}

private func writeTemporaryFile(name: String, contents: String) throws -> URL {
    let directory = FileManager.default.temporaryDirectory.appendingPathComponent(
        "kuyu-model-import-\(UUID().uuidString)",
        isDirectory: true
    )
    try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
    let url = directory.appendingPathComponent(name, isDirectory: false)
    try contents.data(using: .utf8)?.write(to: url, options: [.atomic])
    return url
}

private func minimalBody() -> KuyuBodyModel {
    KuyuBodyModel(
        schemaVersion: "1.0",
        bodyID: "hash-body",
        name: "Hash Body",
        category: "fixture",
        links: [
            LinkDefinition(
                id: "base",
                mass: 1,
                centerOfMass: KuyuVector3(x: 0, y: 0, z: 0),
                inertia: KuyuInertiaTensor(ixx: 0.1, ixy: 0, ixz: 0, iyy: 0.1, iyz: 0, izz: 0.1)
            )
        ],
        joints: []
    )
}

private func minimalWorld() -> KuyuWorldModel {
    KuyuWorldModel(
        schemaVersion: "1.0",
        worldID: "hash-world",
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

private func minimalEmbodiment() -> EmbodimentContract {
    EmbodimentContract(
        schemaVersion: "1.0",
        contractID: "hash-embodiment",
        bodyID: "hash-body",
        signals: SignalCatalog(
            sensor: [
                SignalDefinition(id: "sensor.angle", index: 0, name: "Angle", units: "rad")
            ],
            actuator: [
                SignalDefinition(id: "actuator.joint", index: 0, name: "Joint actuator", units: "rad")
            ],
            drive: [
                SignalDefinition(id: "drive.joint", index: 0, name: "Joint drive", units: "normalized")
            ],
            reflex: [
                SignalDefinition(id: "reflex.joint", index: 0, name: "Joint reflex", units: "normalized")
            ]
        ),
        sensors: [
            SensorDefinition(
                id: "joint-sensor",
                type: "joint-state",
                channels: ["sensor.angle"],
                rateHz: 100,
                latencySeconds: 0
            )
        ],
        actuators: [
            ActuatorDefinition(
                id: "joint-actuator",
                type: "servo",
                channels: ["actuator.joint"],
                limits: ActuatorLimits(min: -1, max: 1, rateLimitPerSecond: 1)
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
