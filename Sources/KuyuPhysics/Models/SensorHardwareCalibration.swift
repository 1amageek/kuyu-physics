public struct SensorHardwareCalibration: Sendable, Codable, Equatable {
    public let sensorID: String
    public let channelIDs: [String]
    public let measuredLatencySeconds: Double
    public let latencyToleranceSeconds: Double
    public let samples: [SensorCalibrationSample]

    public init(
        sensorID: String,
        channelIDs: [String],
        measuredLatencySeconds: Double,
        latencyToleranceSeconds: Double,
        samples: [SensorCalibrationSample]
    ) {
        self.sensorID = sensorID
        self.channelIDs = channelIDs
        self.measuredLatencySeconds = measuredLatencySeconds
        self.latencyToleranceSeconds = latencyToleranceSeconds
        self.samples = samples
    }
}

public struct SensorCalibrationSample: Sendable, Codable, Equatable {
    public let channelID: String
    public let expectedValue: Double
    public let measuredValue: Double
    public let stimulusTimeSeconds: Double
    public let observedTimeSeconds: Double

    public init(
        channelID: String,
        expectedValue: Double,
        measuredValue: Double,
        stimulusTimeSeconds: Double,
        observedTimeSeconds: Double
    ) {
        self.channelID = channelID
        self.expectedValue = expectedValue
        self.measuredValue = measuredValue
        self.stimulusTimeSeconds = stimulusTimeSeconds
        self.observedTimeSeconds = observedTimeSeconds
    }
}
