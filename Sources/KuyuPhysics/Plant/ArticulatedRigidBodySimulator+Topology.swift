import KuyuCore

extension ArticulatedRigidBodySimulator {
    func snapshotTopology(body: KuyuBodyModel) throws -> ArticulatedSnapshotTopology {
        let childLinkIDs = Set(body.joints.map(\.childLinkID))
        let rootLinks = body.links.filter { !childLinkIDs.contains($0.id) }
        guard !rootLinks.isEmpty else {
            throw SimulationError.invalidBody("root-link")
        }
        var jointsByParentLinkID: [String: [JointDefinition]] = [:]
        for joint in body.joints {
            jointsByParentLinkID[joint.parentLinkID, default: []].append(joint)
        }
        var orderedJoints: [JointDefinition] = []
        orderedJoints.reserveCapacity(body.joints.count)
        var visitedJointIDs: Set<String> = []
        var linkStack = rootLinks.map(\.id)

        while !linkStack.isEmpty {
            let parentLinkID = linkStack.removeFirst()
            for joint in jointsByParentLinkID[parentLinkID] ?? [] {
                guard visitedJointIDs.insert(joint.id).inserted else {
                    throw SimulationError.invalidBody("joint-topology.duplicate.\(joint.id)")
                }
                orderedJoints.append(joint)
                linkStack.append(joint.childLinkID)
            }
        }
        guard orderedJoints.count == body.joints.count else {
            let resolvedIDs = Set(orderedJoints.map(\.id))
            let unresolvedIDs = body.joints
                .filter { !resolvedIDs.contains($0.id) }
                .map(\.id)
                .joined(separator: ",")
            throw SimulationError.invalidBody("joint-topology.\(unresolvedIDs)")
        }
        return ArticulatedSnapshotTopology(rootLinks: rootLinks, orderedJoints: orderedJoints)
    }

    func linkSnapshots(
        topology: ArticulatedSnapshotTopology,
        scalars: [String: Double]
    ) throws -> [RigidBodySnapshot] {
        try ArticulatedKinematics(topology: topology).snapshots(scalars: scalars)
    }
}
