public enum CanonicalUnit: String, Sendable, Codable, Equatable, CaseIterable {
    case dimensionless = "1"
    case meter = "m"
    case meterPerSecond = "m/s"
    case meterPerSecondSquared = "m/s^2"
    case inverseSecond = "1/s"
    case radianPerSecond = "rad/s"
    case radianPerSecondSquared = "rad/s^2"
    case kilogram = "kg"
    case kilogramMeterSquared = "kg*m^2"
    case second = "s"
    case newton = "N"
    case newtonMeter = "N*m"
    case pascal = "Pa"
    case kelvin = "K"
    case squareMeter = "m^2"
    case cubicMeter = "m^3"
    case newtonMeterSecondPerRadian = "N*m*s/rad"
    case kilogramPerCubicMeter = "kg/m^3"
    case gasConstant = "m^2/(s^2*K)"

    public var dimension: CanonicalDimension {
        switch self {
        case .dimensionless:
            .dimensionless
        case .meter:
            CanonicalDimension(length: 1)
        case .meterPerSecond, .radianPerSecond:
            CanonicalDimension(length: self == .meterPerSecond ? 1 : 0, time: -1)
        case .meterPerSecondSquared, .radianPerSecondSquared:
            CanonicalDimension(length: self == .meterPerSecondSquared ? 1 : 0, time: -2)
        case .inverseSecond:
            CanonicalDimension(time: -1)
        case .kilogram:
            CanonicalDimension(mass: 1)
        case .kilogramMeterSquared:
            CanonicalDimension(mass: 1, length: 2)
        case .second:
            CanonicalDimension(time: 1)
        case .newton:
            CanonicalDimension(mass: 1, length: 1, time: -2)
        case .newtonMeter:
            CanonicalDimension(mass: 1, length: 2, time: -2)
        case .pascal:
            CanonicalDimension(mass: 1, length: -1, time: -2)
        case .kelvin:
            CanonicalDimension(temperature: 1)
        case .squareMeter:
            CanonicalDimension(length: 2)
        case .cubicMeter:
            CanonicalDimension(length: 3)
        case .newtonMeterSecondPerRadian:
            CanonicalDimension(mass: 1, length: 2, time: -1)
        case .kilogramPerCubicMeter:
            CanonicalDimension(mass: 1, length: -3)
        case .gasConstant:
            CanonicalDimension(length: 2, time: -2, temperature: -1)
        }
    }
}
