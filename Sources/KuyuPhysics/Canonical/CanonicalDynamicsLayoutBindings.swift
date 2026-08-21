public struct CanonicalDynamicsLayoutBindings: Sendable, Codable, Equatable {
    public enum ValidationError: Error, Equatable {
        case invalidLayoutID(String)
        case duplicateLayoutID(String)
    }

    public let state: String
    public let derivative: String
    public let parameters: String
    public let mixer: String
    public let control: String
    public let disturbance: String
    public let environment: String
    public let generalizedForce: String
    public let observables: String

    public init(
        state: String,
        derivative: String,
        parameters: String,
        mixer: String,
        control: String,
        disturbance: String,
        environment: String,
        generalizedForce: String,
        observables: String
    ) throws {
        let values = [
            state,
            derivative,
            parameters,
            mixer,
            control,
            disturbance,
            environment,
            generalizedForce,
            observables,
        ]
        for value in values where !CanonicalIdentifier.isValid(value) {
            throw ValidationError.invalidLayoutID(value)
        }
        var unique = Set<String>()
        for value in values where !unique.insert(value).inserted {
            throw ValidationError.duplicateLayoutID(value)
        }
        self.state = state
        self.derivative = derivative
        self.parameters = parameters
        self.mixer = mixer
        self.control = control
        self.disturbance = disturbance
        self.environment = environment
        self.generalizedForce = generalizedForce
        self.observables = observables
    }

    public var all: [String] {
        [
            state,
            derivative,
            parameters,
            mixer,
            control,
            disturbance,
            environment,
            generalizedForce,
            observables,
        ]
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            state: container.decode(String.self, forKey: .state),
            derivative: container.decode(String.self, forKey: .derivative),
            parameters: container.decode(String.self, forKey: .parameters),
            mixer: container.decode(String.self, forKey: .mixer),
            control: container.decode(String.self, forKey: .control),
            disturbance: container.decode(String.self, forKey: .disturbance),
            environment: container.decode(String.self, forKey: .environment),
            generalizedForce: container.decode(String.self, forKey: .generalizedForce),
            observables: container.decode(String.self, forKey: .observables)
        )
    }

    private enum CodingKeys: String, CodingKey {
        case state
        case derivative
        case parameters
        case mixer
        case control
        case disturbance
        case environment
        case generalizedForce
        case observables
    }
}
