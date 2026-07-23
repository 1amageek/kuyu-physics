import simd

extension ArticulatedContactSolver {
    func jacobianEntries(
        for contact: ArticulatedContact,
        evaluation: ArticulatedKinematicEvaluation
    ) -> [ContactJacobianEntry] {
        let ancestorJointIDs = ancestorJointIDsByLinkID[contact.linkID] ?? []
        var entries: [ContactJacobianEntry] = []
        entries.reserveCapacity(ancestorJointIDs.count)
        for jointID in ancestorJointIDs {
            guard let index = bindingIndexByJointID[jointID],
                  let jointState = evaluation.jointsByID[jointID] else {
                continue
            }
            let linear: SIMD3<Double>
            switch jointState.joint.kind {
            case .fixed:
                continue
            case .revolute, .continuous:
                linear = simd_cross(jointState.axisWorld, contact.point - jointState.originPosition)
            case .prismatic:
                linear = jointState.axisWorld
            }
            entries.append(ContactJacobianEntry(index: index, linear: linear))
        }
        return entries
    }

    func inverseInertiaWeightedNormalDenominator(
        entries: [ContactJacobianEntry],
        normal: SIMD3<Double>
    ) -> Double {
        entries.reduce(0.0) { partial, entry in
            let normalJacobian = simd_dot(entry.linear, normal)
            return partial + normalJacobian * normalJacobian * inverseInertiaByBindingIndex[entry.index]
        }
    }

    static func ancestorJointIDsByLinkID(topology: ArticulatedSnapshotTopology) -> [String: [String]] {
        var result = Dictionary(uniqueKeysWithValues: topology.rootLinks.map { ($0.id, [String]()) })
        for joint in topology.orderedJoints {
            let parentAncestors = result[joint.parentLinkID] ?? []
            result[joint.childLinkID] = parentAncestors + [joint.id]
        }
        return result
    }
}
