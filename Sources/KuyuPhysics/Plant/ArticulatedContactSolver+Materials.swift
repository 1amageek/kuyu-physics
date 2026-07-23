import simd

extension ArticulatedContactSolver {
    func material(for link: LinkDefinition) throws -> ContactMaterial {
        guard let materialID = link.materialID else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.material.\(link.id)")
        }
        guard let material = bodyMaterialsByID[materialID] else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.material.\(materialID)")
        }
        guard let staticFriction = material.staticFriction else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody(
                "contact.material.\(materialID).staticFriction"
            )
        }
        guard let dynamicFriction = material.dynamicFriction else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody(
                "contact.material.\(materialID).dynamicFriction"
            )
        }
        guard let restitution = material.restitution else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody(
                "contact.material.\(materialID).restitution"
            )
        }
        return ContactMaterial(
            staticFriction: staticFriction,
            dynamicFriction: dynamicFriction,
            restitution: restitution
        )
    }

    static func contactSurfaces(from world: KuyuWorldModel) throws -> [ContactSurface] {
        guard !world.surfaces.isEmpty else {
            throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.surfaces")
        }
        let materialsByID = Dictionary(uniqueKeysWithValues: world.materials.map { ($0.id, $0) })
        return try world.surfaces.sorted(by: { $0.id < $1.id }).map { surface in
            guard surface.frameID == "world" else {
                throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.surface.frame.\(surface.id)")
            }
            guard surface.geometry.kind == .box, let size = surface.geometry.size else {
                throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.surface.geometry.\(surface.id)")
            }
            guard abs(surface.pose.rpy.x) <= 1e-12,
                  abs(surface.pose.rpy.y) <= 1e-12,
                  abs(surface.pose.rpy.z) <= 1e-12 else {
                throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.surface.rotation.\(surface.id)")
            }
            guard abs(surface.geometry.pose.rpy.x) <= 1e-12,
                  abs(surface.geometry.pose.rpy.y) <= 1e-12,
                  abs(surface.geometry.pose.rpy.z) <= 1e-12 else {
                throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld(
                    "contact.surface.geometry.rotation.\(surface.id)"
                )
            }
            guard let material = materialsByID[surface.materialID] else {
                throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.surface.material.\(surface.materialID)")
            }
            let scale = surface.geometry.scale ?? KuyuVector3(x: 1, y: 1, z: 1)
            let scaledSize = SIMD3<Double>(
                size.x * scale.x,
                size.y * scale.y,
                size.z * scale.z
            )
            let center = SIMD3<Double>(
                surface.pose.xyz.x + surface.geometry.pose.xyz.x,
                surface.pose.xyz.y + surface.geometry.pose.xyz.y,
                surface.pose.xyz.z + surface.geometry.pose.xyz.z
            )
            return ContactSurface(
                id: surface.id,
                center: center,
                halfExtents: scaledSize * 0.5,
                planeZ: center.z + scaledSize.z * 0.5,
                normal: SIMD3<Double>(0, 0, 1),
                material: ContactMaterial(
                    staticFriction: material.staticFriction,
                    dynamicFriction: material.dynamicFriction,
                    restitution: material.restitution
                )
            )
        }
    }

    func validateCollisions(body: KuyuBodyModel) throws {
        let collidableLinks = body.links.filter { !$0.collisions.isEmpty }
        guard !collidableLinks.isEmpty else {
            throw ArticulatedRigidBodySimulator.SimulationError.invalidBody("contact.collisions")
        }
        for link in collidableLinks {
            _ = try material(for: link)
            for geometry in link.collisions where geometry.kind == .mesh {
                throw ArticulatedRigidBodySimulator.SimulationError.unsupportedWorld("contact.geometry.mesh.\(geometry.id)")
            }
        }
    }
}
