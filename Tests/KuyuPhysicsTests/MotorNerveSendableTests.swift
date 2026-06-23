import KuyuPhysics
import Testing

@Test func motorNerveEndpointsExposeSendableContracts() {
    requireSendable(FixedQuadMotorNerve.self)
    requireSendable(FixedSinglePropMotorNerve.self)
    requireSendable(LiftMotorNerve.self)
    requireSendable(MotorNerveChain.self)
}

private func requireSendable<T: Sendable>(_ type: T.Type) {
    _ = type
}
