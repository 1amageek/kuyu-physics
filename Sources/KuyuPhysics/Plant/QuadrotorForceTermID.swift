public enum QuadrotorForceTermID: String, Sendable, Codable, Equatable, Hashable, CaseIterable {
    case gravity
    case propulsion
    case thrustDensityScaling
    case disturbance
    case aerodynamicDrag
    case aerodynamicLift
    case buoyancy
    case angularDrag
    case gyroscopic
}
