import Foundation

public extension KuyuRobotManifest {
    func validate() throws {
        if schemaVersion.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("manifest.schemaVersion")
        }
        if robotID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("manifest.robotID")
        }
        if bodyModel.path.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("manifest.bodyModel.path")
        }
        if embodimentContract.path.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            throw KuyuModelValidationError.empty("manifest.embodimentContract.path")
        }
    }
}
