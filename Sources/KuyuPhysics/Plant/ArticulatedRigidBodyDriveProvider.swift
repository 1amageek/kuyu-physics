import KuyuCore

public struct ArticulatedRigidBodyDriveProviderResetContext: Sendable, Equatable {
    public let seed: ScenarioSeed
    public let jointIDs: [String]
    public let driveSignalIDs: [String]
    public let jointRanges: [ClosedRange<Double>]

    public init(
        seed: ScenarioSeed,
        jointIDs: [String],
        driveSignalIDs: [String],
        jointRanges: [ClosedRange<Double>]
    ) {
        self.seed = seed
        self.jointIDs = jointIDs
        self.driveSignalIDs = driveSignalIDs
        self.jointRanges = jointRanges
    }
}

public protocol ArticulatedRigidBodyDriveProvider: Sendable {
    var providerID: String { get }

    mutating func reset(context: ArticulatedRigidBodyDriveProviderResetContext) throws
    mutating func driveIntents(context: ArticulatedRigidBodyDriveContext) throws -> [DriveIntent]
}

public extension ArticulatedRigidBodyDriveProvider {
    var providerID: String { String(reflecting: Self.self) }

    mutating func reset(context: ArticulatedRigidBodyDriveProviderResetContext) throws {
        _ = context
    }
}
