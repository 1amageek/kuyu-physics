public enum CanonicalOpcode: String, Sendable, Codable, Equatable, CaseIterable {
    case constant
    case add
    case subtract
    case multiply
    case multiplyComponents
    case divide
    case divideComponents
    case negate
    case component
    case composeVector3
    case cross3
    case length3
    case normalize3OrZero
    case quaternionRotate3
    case quaternionInverseRotate3
    case quaternionDerivative
}
