import KuyuCore

struct ArticulatedContactSolver: Sendable {
    let body: KuyuBodyModel
    let world: KuyuWorldModel
    let bindings: [ArticulatedJointBinding]
    let kinematics: ArticulatedKinematics
    let bindingIndexByJointID: [String: Int]
    let inverseInertiaByBindingIndex: [Double]
    let ancestorJointIDsByLinkID: [String: [String]]
    let orderedLinkIDs: [String]
    let linkByID: [String: LinkDefinition]
    let bodyMaterialsByID: [String: BodyMaterial]
    let surfaces: [ContactSurface]

    init(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        bindings: [ArticulatedJointBinding],
        dynamics: [ArticulatedJointDynamics],
        topology: ArticulatedSnapshotTopology
    ) throws {
        guard world.contact.mode != .disabled else {
            throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.disabled")
        }
        guard world.solver.kind == .deterministicConstraint else {
            throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld(
                "contact.\(world.contact.mode.rawValue).solver.\(world.solver.kind.rawValue)"
            )
        }
        guard dynamics.count == bindings.count else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.dynamics-count")
        }
        for index in bindings.indices where bindings[index].joint.id != dynamics[index].joint.id {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.dynamics-order")
        }
        self.body = body
        self.world = world
        self.bindings = bindings
        self.kinematics = ArticulatedKinematics(topology: topology)
        self.bindingIndexByJointID = Dictionary(uniqueKeysWithValues: bindings.enumerated().map { ($0.element.joint.id, $0.offset) })
        self.inverseInertiaByBindingIndex = dynamics.map { 1.0 / max($0.effectiveInertia, 1e-12) }
        self.ancestorJointIDsByLinkID = Self.ancestorJointIDsByLinkID(topology: topology)
        self.orderedLinkIDs = topology.rootLinks.map(\.id) + topology.orderedJoints.map(\.childLinkID)
        self.linkByID = Dictionary(uniqueKeysWithValues: body.links.map { ($0.id, $0) })
        self.bodyMaterialsByID = Dictionary(uniqueKeysWithValues: body.materials.map { ($0.id, $0) })
        self.surfaces = try Self.contactSurfaces(from: world)
        try validateCollisions(body: body)
    }
}
