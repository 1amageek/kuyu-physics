public enum KuyuModelValidationError: Error, Equatable {
    case empty(String)
    case duplicate(String)
    case unknownReference(String)
    case invalidRange(String)
    case nonFinite(String)
    case nonPositive(String)
    case unsupportedReadiness(String)
}
