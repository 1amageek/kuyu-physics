import EmbodimentContract
import CryptoKit
import Foundation
import KuyuCore

public struct KuyuModelLoader: Sendable {
    public enum LoaderError: Error, Equatable {
        case fileNotFound(String)
        case readFailed(String)
        case decodeFailed(String)
        case hashMismatch(String)
        case invalid(String)
        case missingWorldModel
    }

    private let decoder: JSONDecoder

    public init(decoder: JSONDecoder = JSONDecoder()) {
        self.decoder = decoder
    }

    public func loadRobot(path: String, worldPath overrideWorldPath: String? = nil) throws -> LoadedKuyuRobot {
        let manifestURL = try existingURL(path)
        let manifest: KuyuRobotManifest = try decode(manifestURL)
        do {
            try manifest.validate()
        } catch {
            throw LoaderError.invalid(String(describing: error))
        }

        let baseURL = manifestURL.deletingLastPathComponent()
        let bodyURL = resolve(manifest.bodyModel.path, baseURL: baseURL)
        let contractURL = resolve(manifest.embodimentContract.path, baseURL: baseURL)
        let worldURL: URL
        let worldSHA256: String?
        if let overrideWorldPath, !overrideWorldPath.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            worldURL = resolve(overrideWorldPath, baseURL: baseURL)
            worldSHA256 = nil
        } else if let defaultWorldModel = manifest.defaultWorldModel {
            worldURL = resolve(defaultWorldModel.path, baseURL: baseURL)
            worldSHA256 = defaultWorldModel.sha256
        } else {
            throw LoaderError.missingWorldModel
        }

        let body: KuyuBodyModel = try decode(bodyURL, expectedSHA256: manifest.bodyModel.sha256)
        let world: KuyuWorldModel = try decode(worldURL, expectedSHA256: worldSHA256)
        let embodiment: EmbodimentContract = try decode(
            contractURL,
            expectedSHA256: manifest.embodimentContract.sha256
        )
        let report = try manifest.compatibilityReport.map { reference in
            let url = resolve(reference.path, baseURL: baseURL)
            let report: CompatibilityReport = try decode(url, expectedSHA256: reference.sha256)
            return report
        }

        do {
            try body.validate()
            try world.validate()
            try embodiment.validate()
        } catch {
            throw LoaderError.invalid(String(describing: error))
        }

        return LoadedKuyuRobot(
            manifest: manifest,
            body: body,
            world: world,
            embodiment: embodiment,
            compatibilityReport: report,
            baseURL: baseURL
        )
    }

    public func loadBody(path: String) throws -> KuyuBodyModel {
        let url = try existingURL(path)
        let body: KuyuBodyModel = try decode(url)
        try body.validate()
        return body
    }

    public func loadWorld(path: String) throws -> KuyuWorldModel {
        let url = try existingURL(path)
        let world: KuyuWorldModel = try decode(url)
        try world.validate()
        return world
    }

    public func loadEmbodiment(path: String) throws -> EmbodimentContract {
        let url = try existingURL(path)
        let contract: EmbodimentContract = try decode(url)
        try contract.validate()
        return contract
    }

    public func resolveRenderAsset(_ asset: RenderAssetReference, baseURL: URL) -> URL {
        resolve(asset.path, baseURL: baseURL)
    }

    public func primaryRenderAsset(robot loaded: LoadedKuyuRobot) -> RenderAssetReference? {
        loaded.manifest.renderAssets.first
    }

    public func loadPlantInertialProperties(robot loaded: LoadedKuyuRobot) throws -> PlantInertialProperties {
        let dynamicLinks = loaded.body.links.filter { $0.mass > 0 }
        let mass = dynamicLinks.reduce(0.0) { $0 + $1.mass }
        guard mass > 0 else {
            throw LoaderError.invalid("body.links.mass")
        }
        let inertia = dynamicLinks.reduce(KuyuVector3(x: 0, y: 0, z: 0)) { partial, link in
            KuyuVector3(
                x: partial.x + link.inertia.ixx,
                y: partial.y + link.inertia.iyy,
                z: partial.z + link.inertia.izz
            )
        }
        return PlantInertialProperties(
            mass: mass,
            inertia: Axis3(x: inertia.x, y: inertia.y, z: inertia.z)
        )
    }

    private func existingURL(_ path: String) throws -> URL {
        let url = URL(fileURLWithPath: path)
        guard FileManager.default.fileExists(atPath: url.path) else {
            throw LoaderError.fileNotFound(path)
        }
        return url
    }

    private func decode<T: Decodable>(_ url: URL, expectedSHA256: String? = nil) throws -> T {
        let data: Data
        do {
            data = try Data(contentsOf: url)
        } catch {
            throw LoaderError.readFailed("\(url.path): \(error)")
        }

        if let expectedSHA256 {
            let actualSHA256 = sha256Hex(for: data)
            if actualSHA256.lowercased() != expectedSHA256.lowercased() {
                throw LoaderError.hashMismatch(url.path)
            }
        }

        do {
            return try decoder.decode(T.self, from: data)
        } catch {
            throw LoaderError.decodeFailed("\(url.path): \(error)")
        }
    }

    private func sha256Hex(for data: Data) -> String {
        SHA256.hash(data: data).map { String(format: "%02x", $0) }.joined()
    }

    private func resolve(_ path: String, baseURL: URL) -> URL {
        if path.hasPrefix("/") {
            return URL(fileURLWithPath: path)
        }
        return baseURL.appendingPathComponent(path)
    }
}

public struct LoadedKuyuRobot: Sendable, Equatable {
    public let manifest: KuyuRobotManifest
    public let body: KuyuBodyModel
    public let world: KuyuWorldModel
    public let embodiment: EmbodimentContract
    public let compatibilityReport: CompatibilityReport?
    public let baseURL: URL

    public init(
        manifest: KuyuRobotManifest,
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        embodiment: EmbodimentContract,
        compatibilityReport: CompatibilityReport?,
        baseURL: URL
    ) {
        self.manifest = manifest
        self.body = body
        self.world = world
        self.embodiment = embodiment
        self.compatibilityReport = compatibilityReport
        self.baseURL = baseURL
    }
}
