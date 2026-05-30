import CryptoKit
import Foundation

public struct RobotManifestIdentity: Sendable, Equatable {
    public let path: String
    public let robotID: String
    public let sha256: String

    public init(path: String, robotID: String, sha256: String) {
        self.path = path
        self.robotID = robotID
        self.sha256 = sha256
    }
}

public enum RobotManifestIdentityResolver {
    public static func resolve(path: String) throws -> RobotManifestIdentity? {
        let trimmed = path.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return nil }

        let url = URL(fileURLWithPath: trimmed)
        guard FileManager.default.fileExists(atPath: url.path) else {
            throw KuyuModelLoader.LoaderError.fileNotFound(trimmed)
        }

        let data: Data
        do {
            data = try Data(contentsOf: url)
        } catch {
            throw KuyuModelLoader.LoaderError.readFailed("\(url.path): \(error)")
        }

        let manifest: KuyuRobotManifest
        do {
            manifest = try JSONDecoder().decode(KuyuRobotManifest.self, from: data)
            try manifest.validate()
        } catch {
            throw KuyuModelLoader.LoaderError.decodeFailed("\(url.path): \(error)")
        }

        return RobotManifestIdentity(
            path: url.path,
            robotID: manifest.robotID,
            sha256: sha256Hex(for: data)
        )
    }

    private static func sha256Hex(for data: Data) -> String {
        SHA256.hash(data: data).map { String(format: "%02x", $0) }.joined()
    }
}
