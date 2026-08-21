import CryptoKit
import Foundation

public extension CanonicalProgramDigest {
    enum ComputationError: Error, Equatable {
        case encoding(String)
    }

    static func canonicalSHA256<Value: Encodable>(of value: Value) throws -> Self {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.sortedKeys, .withoutEscapingSlashes]
        let data: Data
        do {
            data = try encoder.encode(value)
        } catch {
            throw ComputationError.encoding(String(describing: error))
        }
        let rawValue = SHA256.hash(data: data)
            .map { String(format: "%02x", $0) }
            .joined()
        return try CanonicalProgramDigest(rawValue)
    }
}
