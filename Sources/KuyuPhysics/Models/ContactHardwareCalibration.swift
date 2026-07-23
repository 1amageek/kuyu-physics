public struct ContactHardwareCalibration: Sendable, Codable, Equatable {
    public let contactPairID: String
    public let linkID: String
    public let materialID: String
    public let staticFriction: Double
    public let dynamicFriction: Double
    public let normalStiffness: Double
    public let normalDamping: Double
    public let samples: [ContactCalibrationSample]

    public init(
        contactPairID: String,
        linkID: String,
        materialID: String,
        staticFriction: Double,
        dynamicFriction: Double,
        normalStiffness: Double,
        normalDamping: Double,
        samples: [ContactCalibrationSample]
    ) {
        self.contactPairID = contactPairID
        self.linkID = linkID
        self.materialID = materialID
        self.staticFriction = staticFriction
        self.dynamicFriction = dynamicFriction
        self.normalStiffness = normalStiffness
        self.normalDamping = normalDamping
        self.samples = samples
    }
}

public struct ContactCalibrationSample: Sendable, Codable, Equatable {
    public let normalForceNewtons: Double
    public let tangentialForceNewtons: Double
    public let slipVelocityMetersPerSecond: Double
    public let penetrationMeters: Double
    public let timestampSeconds: Double

    public init(
        normalForceNewtons: Double,
        tangentialForceNewtons: Double,
        slipVelocityMetersPerSecond: Double,
        penetrationMeters: Double,
        timestampSeconds: Double
    ) {
        self.normalForceNewtons = normalForceNewtons
        self.tangentialForceNewtons = tangentialForceNewtons
        self.slipVelocityMetersPerSecond = slipVelocityMetersPerSecond
        self.penetrationMeters = penetrationMeters
        self.timestampSeconds = timestampSeconds
    }
}
