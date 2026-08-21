public enum CanonicalFloat64ExecutionError: Error, Equatable {
    case missingInput(CanonicalValueID)
    case missingOperand(CanonicalValueID)
    case typeMismatch(CanonicalValueID)
    case divisionByZero(CanonicalValueID)
    case unsupportedConstantShape(CanonicalValueID)
    case nonFiniteResult(CanonicalValueID)
    case missingOutput(graphID: String, outputID: String)
    case unknownForceTerm(CanonicalForceTermID)
}
