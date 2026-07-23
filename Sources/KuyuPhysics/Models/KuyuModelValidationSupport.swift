import Foundation

func validateGeometry(_ geometry: GeometryInstance, field: String) throws {
    if geometry.id.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
        throw KuyuModelValidationError.empty("\(field).id")
    }
    try validatePose(geometry.pose, field: "\(field).pose")
    if let scale = geometry.scale {
        try validatePositive(scale.x, "\(field).scale.x")
        try validatePositive(scale.y, "\(field).scale.y")
        try validatePositive(scale.z, "\(field).scale.z")
    }
    switch geometry.kind {
    case .box:
        guard let size = geometry.size else {
            throw KuyuModelValidationError.empty("\(field).size")
        }
        try validatePositive(size.x, "\(field).size.x")
        try validatePositive(size.y, "\(field).size.y")
        try validatePositive(size.z, "\(field).size.z")
    case .cylinder:
        guard let radius = geometry.radius else {
            throw KuyuModelValidationError.empty("\(field).radius")
        }
        guard let length = geometry.length else {
            throw KuyuModelValidationError.empty("\(field).length")
        }
        try validatePositive(radius, "\(field).radius")
        try validatePositive(length, "\(field).length")
    case .sphere:
        guard let radius = geometry.radius else {
            throw KuyuModelValidationError.empty("\(field).radius")
        }
        try validatePositive(radius, "\(field).radius")
    case .mesh:
        if geometry.meshPath?.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty != false {
            throw KuyuModelValidationError.empty("\(field).meshPath")
        }
    }
}

func validateJointTopology(joints: [JointDefinition]) throws {
    let parentByChild = Dictionary(uniqueKeysWithValues: joints.map { ($0.childLinkID, $0.parentLinkID) })
    for joint in joints {
        var visited: Set<String> = []
        var current = joint.childLinkID
        while let parent = parentByChild[current] {
            if !visited.insert(current).inserted {
                throw KuyuModelValidationError.invalidRange("body.joints.topology.cycle.\(joint.id)")
            }
            current = parent
        }
    }
}

func validateCompliance(_ compliance: ComplianceModel, field: String) throws {
    try validatePositive(compliance.stiffness, "\(field).stiffness")
    try validateNonNegative(compliance.damping, "\(field).damping")
}

func validatePose(_ pose: KuyuPose, field: String) throws {
    try validateFinite(pose.xyz.x, "\(field).xyz.x")
    try validateFinite(pose.xyz.y, "\(field).xyz.y")
    try validateFinite(pose.xyz.z, "\(field).xyz.z")
    try validateFinite(pose.rpy.x, "\(field).rpy.x")
    try validateFinite(pose.rpy.y, "\(field).rpy.y")
    try validateFinite(pose.rpy.z, "\(field).rpy.z")
}

func vectorMagnitude(_ vector: KuyuVector3) -> Double {
    sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
}

func validateInertia(_ inertia: KuyuInertiaTensor, field: String) throws {
    try validatePositive(inertia.ixx, "\(field).ixx")
    try validatePositive(inertia.iyy, "\(field).iyy")
    try validatePositive(inertia.izz, "\(field).izz")
    try validateFinite(inertia.ixy, "\(field).ixy")
    try validateFinite(inertia.ixz, "\(field).ixz")
    try validateFinite(inertia.iyz, "\(field).iyz")
}

func validatePositive(_ value: Double, _ field: String) throws {
    try validateFinite(value, field)
    if value <= 0 {
        throw KuyuModelValidationError.nonPositive(field)
    }
}

func validateOptionalPositive(_ value: Double?, _ field: String) throws {
    guard let value else { return }
    try validatePositive(value, field)
}

func validateOptionalFinite(_ value: Double?, _ field: String) throws {
    guard let value else { return }
    try validateFinite(value, field)
}

func validateNonNegative(_ value: Double, _ field: String) throws {
    try validateFinite(value, field)
    if value < 0 {
        throw KuyuModelValidationError.invalidRange(field)
    }
}

func validateOptionalNonNegative(_ value: Double?, _ field: String) throws {
    guard let value else { return }
    try validateNonNegative(value, field)
}

func validateUnitInterval(_ value: Double, _ field: String) throws {
    try validateFinite(value, field)
    if value < 0 || value > 1 {
        throw KuyuModelValidationError.invalidRange(field)
    }
}

func validateOptionalUnitInterval(_ value: Double?, _ field: String) throws {
    guard let value else { return }
    try validateUnitInterval(value, field)
}

func validateFinite(_ value: Double, _ field: String) throws {
    if !value.isFinite {
        throw KuyuModelValidationError.nonFinite(field)
    }
}
