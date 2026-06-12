import Foundation
import KuyuCore

public struct ArticulatedSineDriveProvider: ArticulatedRigidBodyDriveProvider, Sendable, Equatable {
    public let providerID: String
    public let frequencyHz: Double
    public let amplitudeScale: Double
    public let phaseStrideRadians: Double

    public init(
        providerID: String = "articulated-sine-drive-provider-v1",
        frequencyHz: Double = 0.20,
        amplitudeScale: Double = 0.70,
        phaseStrideRadians: Double = 0.70
    ) {
        self.providerID = providerID
        self.frequencyHz = frequencyHz
        self.amplitudeScale = amplitudeScale
        self.phaseStrideRadians = phaseStrideRadians
    }

    public mutating func driveIntents(context: ArticulatedRigidBodyDriveContext) throws -> [DriveIntent] {
        try context.jointRanges.enumerated().map { index, range in
            let center = (range.lowerBound + range.upperBound) * 0.5
            let amplitude = (range.upperBound - range.lowerBound) * 0.5 * amplitudeScale
            let phase = Double(index) * phaseStrideRadians
            let value = center + amplitude * sin((2.0 * Double.pi * frequencyHz * context.time.time) + phase)
            return try DriveIntent(index: DriveIndex(UInt32(index)), activation: value)
        }
    }
}
