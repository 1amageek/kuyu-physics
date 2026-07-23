import simd

extension ArticulatedContactSolver {
    func allContacts(state: ArticulatedState) throws -> ContactEvaluationBatch {
        let scalars = try scalarState(state)
        let evaluation = try kinematics.evaluate(scalars: scalars)
        var contacts: [ArticulatedContact] = []
        for linkID in orderedLinkIDs {
            guard let link = linkByID[linkID],
                  let linkState = evaluation.linksByID[linkID] else {
                continue
            }
            let material = try material(for: link)
            for collision in link.collisions.sorted(by: { $0.id < $1.id }) {
                let witness = try witness(for: collision, linkState: linkState)
                for surface in surfaces {
                    guard surface.contains(witness.point) else { continue }
                    let penetration = surface.planeZ - witness.point.z
                    contacts.append(ArticulatedContact(
                        linkID: linkID,
                        point: witness.point,
                        pointVelocity: witness.velocity,
                        normal: surface.normal,
                        penetration: penetration,
                        normalVelocity: simd_dot(witness.velocity, surface.normal),
                        material: material.combined(with: surface.material),
                        linkMass: max(link.mass, 1e-9)
                    ))
                }
            }
        }
        contacts.sort {
            if $0.linkID != $1.linkID { return $0.linkID < $1.linkID }
            if $0.point.x != $1.point.x { return $0.point.x < $1.point.x }
            if $0.point.y != $1.point.y { return $0.point.y < $1.point.y }
            return $0.point.z < $1.point.z
        }
        return ContactEvaluationBatch(contacts: contacts, evaluation: evaluation)
    }

    func scalarState(_ state: ArticulatedState) throws -> [String: Double] {
        var scalars: [String: Double] = [:]
        var positionsByJointID: [String: Double] = [:]
        var velocitiesByJointID: [String: Double] = [:]
        for index in state.position.indices {
            let jointID = bindings[index].joint.id
            scalars[jointID] = state.position[index]
            scalars["velocity_\(jointID)"] = state.velocity[index]
            positionsByJointID[jointID] = state.position[index]
            velocitiesByJointID[jointID] = state.velocity[index]
        }

        var unresolvedMimics = body.joints.filter { $0.mimic != nil }
        while !unresolvedMimics.isEmpty {
            var remaining: [JointDefinition] = []
            var resolvedCount = 0
            for joint in unresolvedMimics {
                guard let mimic = joint.mimic,
                      let masterPosition = positionsByJointID[mimic.jointID] else {
                    remaining.append(joint)
                    continue
                }
                let masterVelocity = velocitiesByJointID[mimic.jointID] ?? 0
                let position = masterPosition * mimic.multiplier + mimic.offset
                let velocity = masterVelocity * mimic.multiplier
                positionsByJointID[joint.id] = position
                velocitiesByJointID[joint.id] = velocity
                scalars[joint.id] = position
                scalars["velocity_\(joint.id)"] = velocity
                resolvedCount += 1
            }
            if resolvedCount == 0 {
                let ids = remaining.map(\.id).joined(separator: ",")
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("mimic.\(ids)")
            }
            unresolvedMimics = remaining
        }
        return scalars
    }
}
