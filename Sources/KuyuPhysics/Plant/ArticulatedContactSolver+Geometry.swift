import Foundation
import simd

extension ArticulatedContactSolver {
    func witness(
        for geometry: GeometryInstance,
        linkState: ArticulatedLinkKinematicState
    ) throws -> CollisionWitness {
        let poseOrientation = orientation(fromRPY: geometry.pose.rpy)
        let worldOrientation = (linkState.orientation * poseOrientation).normalizedQuat
        let center = linkState.position + linkState.orientation.act(simdVector(geometry.pose.xyz))
        let point: SIMD3<Double>
        switch geometry.kind {
        case .sphere:
            guard let radius = geometry.radius else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.geometry.\(geometry.id).radius")
            }
            let scale = geometry.scale.map { max($0.x, max($0.y, $0.z)) } ?? 1
            point = center - SIMD3<Double>(0, 0, radius * scale)
        case .box:
            guard let size = geometry.size else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.geometry.\(geometry.id).size")
            }
            let scale = geometry.scale ?? KuyuVector3(x: 1, y: 1, z: 1)
            let half = SIMD3<Double>(size.x * scale.x, size.y * scale.y, size.z * scale.z) * 0.5
            point = boxVertices(halfExtents: half, orientation: worldOrientation, center: center).min { lhs, rhs in
                lhs.z < rhs.z
            } ?? center
        case .cylinder:
            guard let radius = geometry.radius, let length = geometry.length else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.geometry.\(geometry.id).cylinder")
            }
            let scale = geometry.scale ?? KuyuVector3(x: 1, y: 1, z: 1)
            let scaledRadius = radius * max(scale.x, scale.y)
            let halfLength = length * scale.z * 0.5
            point = cylinderWitnessPoint(
                radius: scaledRadius,
                halfLength: halfLength,
                orientation: worldOrientation,
                center: center
            )
        case .mesh:
            throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.geometry.mesh.\(geometry.id)")
        }
        let velocity = linkState.velocity + simd_cross(linkState.angularVelocity, point - linkState.position)
        return CollisionWitness(point: point, velocity: velocity)
    }

    func boxVertices(
        halfExtents: SIMD3<Double>,
        orientation: simd_quatd,
        center: SIMD3<Double>
    ) -> [SIMD3<Double>] {
        var vertices: [SIMD3<Double>] = []
        vertices.reserveCapacity(8)
        for x in [-halfExtents.x, halfExtents.x] {
            for y in [-halfExtents.y, halfExtents.y] {
                for z in [-halfExtents.z, halfExtents.z] {
                    vertices.append(center + orientation.act(SIMD3<Double>(x, y, z)))
                }
            }
        }
        return vertices
    }

    func cylinderWitnessPoint(
        radius: Double,
        halfLength: Double,
        orientation: simd_quatd,
        center: SIMD3<Double>
    ) -> SIMD3<Double> {
        var candidates: [SIMD3<Double>] = []
        candidates.reserveCapacity(16)
        for z in [-halfLength, halfLength] {
            for index in 0..<8 {
                let angle = Double(index) * .pi * 0.25
                let local = SIMD3<Double>(cos(angle) * radius, sin(angle) * radius, z)
                candidates.append(center + orientation.act(local))
            }
        }
        return candidates.min { lhs, rhs in lhs.z < rhs.z } ?? center
    }

    func orientation(fromRPY rpy: KuyuVector3) -> simd_quatd {
        let roll = simd_quatd(angle: rpy.x, axis: SIMD3<Double>(1, 0, 0))
        let pitch = simd_quatd(angle: rpy.y, axis: SIMD3<Double>(0, 1, 0))
        let yaw = simd_quatd(angle: rpy.z, axis: SIMD3<Double>(0, 0, 1))
        return (yaw * pitch * roll).normalizedQuat
    }

    func simdVector(_ vector: KuyuVector3) -> SIMD3<Double> {
        SIMD3<Double>(vector.x, vector.y, vector.z)
    }
}
