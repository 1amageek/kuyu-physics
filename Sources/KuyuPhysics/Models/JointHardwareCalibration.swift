public struct JointHardwareCalibration: Sendable, Codable, Equatable {
    public let jointID: String
    public let actuatorID: String
    public let commandDirection: Double
    public let mechanicalReductionRatio: Double
    public let identifiedDynamics: IdentifiedJointDynamics
    public let samples: [JointCalibrationSample]

    public init(
        jointID: String,
        actuatorID: String,
        commandDirection: Double,
        mechanicalReductionRatio: Double,
        identifiedDynamics: IdentifiedJointDynamics,
        samples: [JointCalibrationSample]
    ) {
        self.jointID = jointID
        self.actuatorID = actuatorID
        self.commandDirection = commandDirection
        self.mechanicalReductionRatio = mechanicalReductionRatio
        self.identifiedDynamics = identifiedDynamics
        self.samples = samples
    }
}

public struct IdentifiedJointDynamics: Sendable, Codable, Equatable {
    public let latencySeconds: Double
    public let timeConstantSeconds: Double
    public let deadbandRadians: Double
    public let backlashRadians: Double
    public let viscousDamping: Double
    public let coulombFriction: Double
    public let meanAbsoluteErrorRadians: Double
    public let maxObservedErrorRadians: Double

    public init(
        latencySeconds: Double,
        timeConstantSeconds: Double,
        deadbandRadians: Double,
        backlashRadians: Double,
        viscousDamping: Double,
        coulombFriction: Double,
        meanAbsoluteErrorRadians: Double,
        maxObservedErrorRadians: Double
    ) {
        self.latencySeconds = latencySeconds
        self.timeConstantSeconds = timeConstantSeconds
        self.deadbandRadians = deadbandRadians
        self.backlashRadians = backlashRadians
        self.viscousDamping = viscousDamping
        self.coulombFriction = coulombFriction
        self.meanAbsoluteErrorRadians = meanAbsoluteErrorRadians
        self.maxObservedErrorRadians = maxObservedErrorRadians
    }
}

public struct JointCalibrationSample: Sendable, Codable, Equatable {
    public let commandedPositionRadians: Double
    public let measuredPositionRadians: Double?
    public let commandPulse: Int?
    public let commandTimeSeconds: Double
    public let observedTimeSeconds: Double?
    public let measuredVelocityRadiansPerSecond: Double?
    public let busVoltageVolts: Double?
    public let servoTemperatureCelsius: Double?
    public let loadEstimate: Double?

    public init(
        commandedPositionRadians: Double,
        measuredPositionRadians: Double? = nil,
        commandPulse: Int? = nil,
        commandTimeSeconds: Double,
        observedTimeSeconds: Double? = nil,
        measuredVelocityRadiansPerSecond: Double? = nil,
        busVoltageVolts: Double? = nil,
        servoTemperatureCelsius: Double? = nil,
        loadEstimate: Double? = nil
    ) {
        self.commandedPositionRadians = commandedPositionRadians
        self.measuredPositionRadians = measuredPositionRadians
        self.commandPulse = commandPulse
        self.commandTimeSeconds = commandTimeSeconds
        self.observedTimeSeconds = observedTimeSeconds
        self.measuredVelocityRadiansPerSecond = measuredVelocityRadiansPerSecond
        self.busVoltageVolts = busVoltageVolts
        self.servoTemperatureCelsius = servoTemperatureCelsius
        self.loadEstimate = loadEstimate
    }
}
