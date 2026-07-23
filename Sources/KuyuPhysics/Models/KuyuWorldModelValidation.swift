import Foundation

public extension KuyuWorldModel {
    func validate() throws {
        if schemaVersion.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("world.schemaVersion")
        }
        if worldID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("world.worldID")
        }
        try validatePositive(time.fixedStepSeconds, "world.time.fixedStepSeconds")
        if time.substeps <= 0 {
            throw KuyuModelValidationError.invalidRange("world.time.substeps")
        }
        if solver.iterations < 0 {
            throw KuyuModelValidationError.invalidRange("world.solver.iterations")
        }
        try validateNonNegative(solver.tolerance, "world.solver.tolerance")
        try validateFinite(gravity.acceleration.x, "world.gravity.acceleration.x")
        try validateFinite(gravity.acceleration.y, "world.gravity.acceleration.y")
        try validateFinite(gravity.acceleration.z, "world.gravity.acceleration.z")
        try validateAtmosphere(atmosphere)
        try validateFinite(wind.velocityWorld.x, "world.wind.velocityWorld.x")
        try validateFinite(wind.velocityWorld.y, "world.wind.velocityWorld.y")
        try validateFinite(wind.velocityWorld.z, "world.wind.velocityWorld.z")
        try validateWorldMaterials(materials)
        try validateWorldSurfaces(surfaces, materials: materials)
        try validateContact(contact, solver: solver)
        try validateNonNegative(nap.forceAbsoluteThreshold, "world.nap.forceAbsoluteThreshold")
        try validateNonNegative(nap.forceRelativeThreshold, "world.nap.forceRelativeThreshold")
        try validateNonNegative(nap.torqueAbsoluteThreshold, "world.nap.torqueAbsoluteThreshold")
        try validateNonNegative(nap.torqueRelativeThreshold, "world.nap.torqueRelativeThreshold")
    }
}

private func validateAtmosphere(_ atmosphere: AtmosphereModel) throws {
    try validateOptionalPositive(atmosphere.airDensity, "world.atmosphere.airDensity")
    try validateOptionalPositive(atmosphere.temperatureKelvin, "world.atmosphere.temperatureKelvin")
}

private func validateWorldMaterials(_ materials: [WorldMaterial]) throws {
    var materialIDs: Set<String> = []
    for material in materials {
        if material.id.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("world.materials.id")
        }
        if !materialIDs.insert(material.id).inserted {
            throw KuyuModelValidationError.duplicate("world.materials.\(material.id)")
        }
        try validateNonNegative(material.staticFriction, "world.materials.\(material.id).staticFriction")
        try validateNonNegative(material.dynamicFriction, "world.materials.\(material.id).dynamicFriction")
        if material.dynamicFriction > material.staticFriction {
            throw KuyuModelValidationError.invalidRange("world.materials.\(material.id).dynamicFriction")
        }
        try validateUnitInterval(material.restitution, "world.materials.\(material.id).restitution")
    }
}

private func validateWorldSurfaces(
    _ surfaces: [WorldSurface],
    materials: [WorldMaterial]
) throws {
    let materialIDs = Set(materials.map(\.id))
    var surfaceIDs: Set<String> = []
    for surface in surfaces {
        if surface.id.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("world.surfaces.id")
        }
        if !surfaceIDs.insert(surface.id).inserted {
            throw KuyuModelValidationError.duplicate("world.surfaces.\(surface.id)")
        }
        if surface.frameID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("world.surfaces.\(surface.id).frameID")
        }
        if !materialIDs.contains(surface.materialID) {
            throw KuyuModelValidationError.unknownReference("world.surfaces.\(surface.id).materialID")
        }
        try validatePose(surface.pose, field: "world.surfaces.\(surface.id).pose")
        try validateGeometry(surface.geometry, field: "world.surfaces.\(surface.id).geometry")
    }
}

private func validateContact(_ contact: ContactModel, solver: SolverModel) throws {
    switch contact.mode {
    case .disabled:
        if solver.kind != .disabledContact {
            throw KuyuModelValidationError.invalidRange("world.solver.kind")
        }
        try validateOptionalPositive(contact.stiffness, "world.contact.stiffness")
        try validateOptionalNonNegative(contact.damping, "world.contact.damping")
    case .penalty:
        guard solver.kind == .deterministicConstraint else {
            throw KuyuModelValidationError.invalidRange("world.solver.kind")
        }
        guard let stiffness = contact.stiffness else {
            throw KuyuModelValidationError.empty("world.contact.stiffness")
        }
        guard let damping = contact.damping else {
            throw KuyuModelValidationError.empty("world.contact.damping")
        }
        try validatePositive(stiffness, "world.contact.stiffness")
        try validateNonNegative(damping, "world.contact.damping")
    case .constraint:
        guard solver.kind == .deterministicConstraint else {
            throw KuyuModelValidationError.invalidRange("world.solver.kind")
        }
        try validateOptionalPositive(contact.stiffness, "world.contact.stiffness")
        try validateOptionalNonNegative(contact.damping, "world.contact.damping")
    }
}
