import Foundation

public struct KuyuRobotManifest: Sendable, Codable, Equatable {
    public let schemaVersion: String
    public let robotID: String
    public let name: String
    public let category: String
    public let manufacturer: String?
    public let tags: [String]
    public let bodyModel: ModelReference
    public let defaultWorldModel: ModelReference?
    public let embodimentContract: ModelReference
    public let compatibilityReport: ModelReference?
    public let renderAssets: [RenderAssetReference]

    public init(
        schemaVersion: String,
        robotID: String,
        name: String,
        category: String,
        manufacturer: String? = nil,
        tags: [String] = [],
        bodyModel: ModelReference,
        defaultWorldModel: ModelReference? = nil,
        embodimentContract: ModelReference,
        compatibilityReport: ModelReference? = nil,
        renderAssets: [RenderAssetReference] = []
    ) {
        self.schemaVersion = schemaVersion
        self.robotID = robotID
        self.name = name
        self.category = category
        self.manufacturer = manufacturer
        self.tags = tags
        self.bodyModel = bodyModel
        self.defaultWorldModel = defaultWorldModel
        self.embodimentContract = embodimentContract
        self.compatibilityReport = compatibilityReport
        self.renderAssets = renderAssets
    }
}

public struct ModelReference: Sendable, Codable, Equatable {
    public let path: String
    public let sha256: String?

    public init(path: String, sha256: String? = nil) {
        self.path = path
        self.sha256 = sha256
    }
}

public struct RenderAssetReference: Sendable, Codable, Equatable {
    public let id: String
    public let name: String
    public let format: RenderMeshFormat
    public let path: String
    public let frameID: String?
    public let scale: KuyuVector3?

    public init(
        id: String,
        name: String,
        format: RenderMeshFormat,
        path: String,
        frameID: String? = nil,
        scale: KuyuVector3? = nil
    ) {
        self.id = id
        self.name = name
        self.format = format
        self.path = path
        self.frameID = frameID
        self.scale = scale
    }
}
