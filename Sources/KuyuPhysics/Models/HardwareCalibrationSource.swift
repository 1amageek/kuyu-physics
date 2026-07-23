public struct HardwareCalibrationSource: Sendable, Codable, Equatable {
    public let operatorID: String?
    public let deviceID: String?
    public let firmwareVersion: String?
    public let firmwareAttestationKeyID: String?
    public let firmwareAttestationPublicKeyX963: String?
    public let measurementSystem: String
    public let notes: String?

    public init(
        operatorID: String? = nil,
        deviceID: String? = nil,
        firmwareVersion: String? = nil,
        firmwareAttestationKeyID: String? = nil,
        firmwareAttestationPublicKeyX963: String? = nil,
        measurementSystem: String,
        notes: String? = nil
    ) {
        self.operatorID = operatorID
        self.deviceID = deviceID
        self.firmwareVersion = firmwareVersion
        self.firmwareAttestationKeyID = firmwareAttestationKeyID
        self.firmwareAttestationPublicKeyX963 = firmwareAttestationPublicKeyX963
        self.measurementSystem = measurementSystem
        self.notes = notes
    }
}
