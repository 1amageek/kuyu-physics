import KuyuCore

public struct ArticulatedRigidBodyDriveContext: Sendable, Equatable {
    public let time: WorldTime
    public let jointIDs: [String]
    public let driveSignalIDs: [String]
    public let actuatorSignalIDs: [String]
    public let jointRanges: [ClosedRange<Double>]
    public let positions: [Double]
    public let velocities: [Double]
    public let targets: [Double]
    public let torques: [Double]

    public init(
        time: WorldTime,
        jointIDs: [String],
        driveSignalIDs: [String],
        actuatorSignalIDs: [String],
        jointRanges: [ClosedRange<Double>],
        positions: [Double],
        velocities: [Double],
        targets: [Double],
        torques: [Double]
    ) {
        self.time = time
        self.jointIDs = jointIDs
        self.driveSignalIDs = driveSignalIDs
        self.actuatorSignalIDs = actuatorSignalIDs
        self.jointRanges = jointRanges
        self.positions = positions
        self.velocities = velocities
        self.targets = targets
        self.torques = torques
    }
}
