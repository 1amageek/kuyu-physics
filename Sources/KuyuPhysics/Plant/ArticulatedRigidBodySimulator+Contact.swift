import KuyuCore

extension ArticulatedRigidBodySimulator {
    func makeContactSolver(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        bindings: [ArticulatedJointBinding],
        dynamics: [ArticulatedJointDynamics],
        topology: ArticulatedSnapshotTopology
    ) throws -> ArticulatedContactSolver? {
        guard world.contact.mode != .disabled else { return nil }
        return try ArticulatedContactSolver(
            body: body,
            world: world,
            bindings: bindings,
            dynamics: dynamics,
            topology: topology
        )
    }
}
