import Foundation

public struct DescriptorCorpusHardwareEvidence: Sendable, Codable, Equatable {
    public let reportID: String
    public let robotID: String
    public let bodyID: String
    public let embodimentContractID: String
    public let reportHash: String
    public let readinessLevel: ReadinessLevel
    public let measurementSystem: String
    public let deviceID: String?
    public let operatorID: String?
    public let jointCalibrationCount: Int
    public let jointSampleCount: Int
    public let measuredJointSampleCount: Int
    public let observedJointSampleCount: Int?
    public let sensorCalibrationCount: Int
    public let sensorSampleCount: Int
    public let observedSensorSampleCount: Int
    public let contactCalibrationCount: Int
    public let contactSampleCount: Int

    public init(
        reportID: String,
        robotID: String,
        bodyID: String,
        embodimentContractID: String,
        reportHash: String,
        readinessLevel: ReadinessLevel,
        measurementSystem: String,
        deviceID: String? = nil,
        operatorID: String? = nil,
        jointCalibrationCount: Int,
        jointSampleCount: Int,
        measuredJointSampleCount: Int,
        observedJointSampleCount: Int? = nil,
        sensorCalibrationCount: Int = 0,
        sensorSampleCount: Int = 0,
        observedSensorSampleCount: Int = 0,
        contactCalibrationCount: Int,
        contactSampleCount: Int
    ) {
        self.reportID = reportID
        self.robotID = robotID
        self.bodyID = bodyID
        self.embodimentContractID = embodimentContractID
        self.reportHash = reportHash
        self.readinessLevel = readinessLevel
        self.measurementSystem = measurementSystem
        self.deviceID = deviceID
        self.operatorID = operatorID
        self.jointCalibrationCount = jointCalibrationCount
        self.jointSampleCount = jointSampleCount
        self.measuredJointSampleCount = measuredJointSampleCount
        self.observedJointSampleCount = observedJointSampleCount
        self.sensorCalibrationCount = sensorCalibrationCount
        self.sensorSampleCount = sensorSampleCount
        self.observedSensorSampleCount = observedSensorSampleCount
        self.contactCalibrationCount = contactCalibrationCount
        self.contactSampleCount = contactSampleCount
    }

    private enum CodingKeys: String, CodingKey {
        case reportID
        case robotID
        case bodyID
        case embodimentContractID
        case reportHash
        case readinessLevel
        case measurementSystem
        case deviceID
        case operatorID
        case jointCalibrationCount
        case jointSampleCount
        case measuredJointSampleCount
        case observedJointSampleCount
        case sensorCalibrationCount
        case sensorSampleCount
        case observedSensorSampleCount
        case contactCalibrationCount
        case contactSampleCount
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        reportID = try container.decode(String.self, forKey: .reportID)
        robotID = try container.decode(String.self, forKey: .robotID)
        bodyID = try container.decode(String.self, forKey: .bodyID)
        embodimentContractID = try container.decode(String.self, forKey: .embodimentContractID)
        reportHash = try container.decode(String.self, forKey: .reportHash)
        readinessLevel = try container.decode(ReadinessLevel.self, forKey: .readinessLevel)
        measurementSystem = try container.decode(String.self, forKey: .measurementSystem)
        deviceID = try container.decodeIfPresent(String.self, forKey: .deviceID)
        operatorID = try container.decodeIfPresent(String.self, forKey: .operatorID)
        jointCalibrationCount = try container.decode(Int.self, forKey: .jointCalibrationCount)
        jointSampleCount = try container.decode(Int.self, forKey: .jointSampleCount)
        measuredJointSampleCount = try container.decode(Int.self, forKey: .measuredJointSampleCount)
        observedJointSampleCount = try container.decodeIfPresent(Int.self, forKey: .observedJointSampleCount)
        sensorCalibrationCount = try container.decodeIfPresent(Int.self, forKey: .sensorCalibrationCount) ?? 0
        sensorSampleCount = try container.decodeIfPresent(Int.self, forKey: .sensorSampleCount) ?? 0
        observedSensorSampleCount = try container.decodeIfPresent(Int.self, forKey: .observedSensorSampleCount) ?? 0
        contactCalibrationCount = try container.decode(Int.self, forKey: .contactCalibrationCount)
        contactSampleCount = try container.decode(Int.self, forKey: .contactSampleCount)
    }

    public func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(reportID, forKey: .reportID)
        try container.encode(robotID, forKey: .robotID)
        try container.encode(bodyID, forKey: .bodyID)
        try container.encode(embodimentContractID, forKey: .embodimentContractID)
        try container.encode(reportHash, forKey: .reportHash)
        try container.encode(readinessLevel, forKey: .readinessLevel)
        try container.encode(measurementSystem, forKey: .measurementSystem)
        try container.encodeIfPresent(deviceID, forKey: .deviceID)
        try container.encodeIfPresent(operatorID, forKey: .operatorID)
        try container.encode(jointCalibrationCount, forKey: .jointCalibrationCount)
        try container.encode(jointSampleCount, forKey: .jointSampleCount)
        try container.encode(measuredJointSampleCount, forKey: .measuredJointSampleCount)
        try container.encodeIfPresent(observedJointSampleCount, forKey: .observedJointSampleCount)
        try container.encode(sensorCalibrationCount, forKey: .sensorCalibrationCount)
        try container.encode(sensorSampleCount, forKey: .sensorSampleCount)
        try container.encode(observedSensorSampleCount, forKey: .observedSensorSampleCount)
        try container.encode(contactCalibrationCount, forKey: .contactCalibrationCount)
        try container.encode(contactSampleCount, forKey: .contactSampleCount)
    }
}

extension DescriptorCorpusHardwareEvidence {
    init(report: HardwareCalibrationReport, reportHash: String) {
        let jointSampleCount = report.jointCalibrations.reduce(0) { partial, calibration in
            partial + calibration.samples.count
        }
        let measuredJointSampleCount = report.jointCalibrations.reduce(0) { partial, calibration in
            partial + calibration.samples.filter { $0.measuredPositionRadians != nil }.count
        }
        let observedJointSampleCount = report.jointCalibrations.reduce(0) { partial, calibration in
            partial + calibration.samples.filter { $0.observedTimeSeconds != nil }.count
        }
        let sensorSampleCount = report.sensorCalibrations.reduce(0) { partial, calibration in
            partial + calibration.samples.count
        }
        let observedSensorSampleCount = report.sensorCalibrations.reduce(0) { partial, calibration in
            partial + calibration.samples.count
        }
        let contactSampleCount = report.contactCalibrations.reduce(0) { partial, calibration in
            partial + calibration.samples.count
        }
        self.init(
            reportID: report.reportID,
            robotID: report.robotID,
            bodyID: report.bodyID,
            embodimentContractID: report.embodimentContractID,
            reportHash: reportHash,
            readinessLevel: report.readinessLevel,
            measurementSystem: report.source.measurementSystem,
            deviceID: report.source.deviceID,
            operatorID: report.source.operatorID,
            jointCalibrationCount: report.jointCalibrations.count,
            jointSampleCount: jointSampleCount,
            measuredJointSampleCount: measuredJointSampleCount,
            observedJointSampleCount: observedJointSampleCount,
            sensorCalibrationCount: report.sensorCalibrations.count,
            sensorSampleCount: sensorSampleCount,
            observedSensorSampleCount: observedSensorSampleCount,
            contactCalibrationCount: report.contactCalibrations.count,
            contactSampleCount: contactSampleCount
        )
    }
}
