import KuyuCore

/// Analytical state for a quadrotor.
/// Extracts pos(3) + vel(3) + quat(4) + omega(3) = 13 dimensions from PlantStateSnapshot.
public struct QuadrotorAnalyticalState: AnalyticalState {
    public let snapshot: PlantStateSnapshot

    public init(snapshot: PlantStateSnapshot) {
        self.snapshot = snapshot
    }

    public var dimensions: Int { 13 }

    public func toArray() -> [Float] {
        let root = snapshot.root
        return [
            Float(root.position.x), Float(root.position.y), Float(root.position.z),
            Float(root.velocity.x), Float(root.velocity.y), Float(root.velocity.z),
            Float(root.orientation.w), Float(root.orientation.x),
            Float(root.orientation.y), Float(root.orientation.z),
            Float(root.angularVelocity.x), Float(root.angularVelocity.y),
            Float(root.angularVelocity.z),
        ]
    }

    public func toPlantStateSnapshot() -> PlantStateSnapshot {
        snapshot
    }
}
