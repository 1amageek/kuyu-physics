import Foundation
import KuyuPhysics
import Testing

@Test(.timeLimit(.minutes(1))) func urdfKinematicParserReadsVisualJointGraph() throws {
    let url = FileManager.default.temporaryDirectory
        .appendingPathComponent("kuyu-urdf-\(UUID().uuidString).urdf")
    let xml = """
    <?xml version="1.0"?>
    <robot name="test_arm">
      <link name="base_link">
        <visual>
          <origin xyz="0 0 0.1" rpy="0 0 0"/>
          <geometry>
            <cylinder radius="0.1" length="0.2"/>
          </geometry>
        </visual>
      </link>
      <link name="link_1">
        <visual>
          <origin xyz="0.5 0 0" rpy="0 0 0"/>
          <geometry>
            <box size="1.0 0.1 0.1"/>
          </geometry>
        </visual>
      </link>
      <joint name="joint_1" type="revolute">
        <parent link="base_link"/>
        <child link="link_1"/>
        <origin xyz="0 0 0.2" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
      </joint>
    </robot>
    """
    let data = try #require(xml.data(using: .utf8))
    try data.write(to: url)
    defer {
        do {
            try FileManager.default.removeItem(at: url)
        } catch {
            Issue.record("Temporary URDF cleanup failed: \(error)")
        }
    }

    let model = try URDFKinematicParser().parse(url: url)

    #expect(model.rootLinkNames == ["base_link"])
    #expect(model.links.count == 2)
    #expect(model.links[0].visuals.count == 1)
    #expect(model.joints.count == 1)
    #expect(model.joints[0].name == "joint_1")
    #expect(model.joints[0].parent == "base_link")
    #expect(model.joints[0].child == "link_1")
}
