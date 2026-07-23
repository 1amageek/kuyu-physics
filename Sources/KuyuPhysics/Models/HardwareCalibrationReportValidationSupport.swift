import Foundation

func activeJoints(body: KuyuBodyModel) -> [JointDefinition] {
    body.joints.filter { joint in
        joint.mimic == nil && (joint.kind == .revolute || joint.kind == .continuous || joint.kind == .prismatic)
    }
}

func validateDynamics(_ dynamics: IdentifiedJointDynamics, field: String) throws {
    try ensureNonNegative(dynamics.latencySeconds, "\(field).latencySeconds")
    try ensurePositive(dynamics.timeConstantSeconds, "\(field).timeConstantSeconds")
    try ensureNonNegative(dynamics.deadbandRadians, "\(field).deadbandRadians")
    try ensureNonNegative(dynamics.backlashRadians, "\(field).backlashRadians")
    try ensureNonNegative(dynamics.viscousDamping, "\(field).viscousDamping")
    try ensureNonNegative(dynamics.coulombFriction, "\(field).coulombFriction")
    try ensureNonNegative(dynamics.meanAbsoluteErrorRadians, "\(field).meanAbsoluteErrorRadians")
    try ensureNonNegative(dynamics.maxObservedErrorRadians, "\(field).maxObservedErrorRadians")
}

func validateSample(_ sample: JointCalibrationSample, field: String) throws {
    try ensureFinite(sample.commandedPositionRadians, "\(field).commandedPositionRadians")
    try ensureFinite(sample.commandTimeSeconds, "\(field).commandTimeSeconds")
    try ensureOptionalFinite(sample.measuredPositionRadians, "\(field).measuredPositionRadians")
    try ensureOptionalFinite(sample.observedTimeSeconds, "\(field).observedTimeSeconds")
    try ensureOptionalFinite(sample.measuredVelocityRadiansPerSecond, "\(field).measuredVelocityRadiansPerSecond")
    try ensureOptionalFinite(sample.busVoltageVolts, "\(field).busVoltageVolts")
    try ensureOptionalFinite(sample.servoTemperatureCelsius, "\(field).servoTemperatureCelsius")
    try ensureOptionalFinite(sample.loadEstimate, "\(field).loadEstimate")
    if let commandPulse = sample.commandPulse, !(0...4095).contains(commandPulse) {
        throw HardwareCalibrationValidationError.invalidRange("\(field).commandPulse")
    }
}

func validateSensorSample(
    _ sample: SensorCalibrationSample,
    channelIDs: Set<String>,
    field: String
) throws {
    if sample.channelID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
        throw HardwareCalibrationValidationError.empty("\(field).channelID")
    }
    if !channelIDs.contains(sample.channelID) {
        throw HardwareCalibrationValidationError.unknownReference("\(field).\(sample.channelID)")
    }
    try ensureFinite(sample.expectedValue, "\(field).expectedValue")
    try ensureFinite(sample.measuredValue, "\(field).measuredValue")
    try ensureFinite(sample.stimulusTimeSeconds, "\(field).stimulusTimeSeconds")
    try ensureFinite(sample.observedTimeSeconds, "\(field).observedTimeSeconds")
}

func ensureFinite(_ value: Double, _ field: String) throws {
    if !value.isFinite {
        throw HardwareCalibrationValidationError.nonFinite(field)
    }
}

func ensureOptionalFinite(_ value: Double?, _ field: String) throws {
    guard let value else { return }
    try ensureFinite(value, field)
}

func ensurePositive(_ value: Double, _ field: String) throws {
    try ensureFinite(value, field)
    if value <= 0 {
        throw HardwareCalibrationValidationError.invalidRange(field)
    }
}

func ensureNonNegative(_ value: Double, _ field: String) throws {
    try ensureFinite(value, field)
    if value < 0 {
        throw HardwareCalibrationValidationError.invalidRange(field)
    }
}
