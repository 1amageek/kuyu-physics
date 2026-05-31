import simd

public enum QuadrotorConstraintProjection: String, Sendable, Codable, Equatable {
    case free
    case verticalOnly

    public func project(derivative: ReferenceQuadrotorStateDerivative) -> ReferenceQuadrotorStateDerivative {
        switch self {
        case .free:
            return derivative
        case .verticalOnly:
            return ReferenceQuadrotorStateDerivative(
                position: SIMD3<Double>(0, 0, derivative.position.z),
                velocity: SIMD3<Double>(0, 0, derivative.velocity.z),
                orientation: SIMD4<Double>(repeating: 0),
                angularVelocity: SIMD3<Double>(repeating: 0)
            )
        }
    }

    public func project(state: ReferenceQuadrotorState) -> ReferenceQuadrotorState {
        switch self {
        case .free:
            return state.normalized()
        case .verticalOnly:
            return ReferenceQuadrotorState(
                uncheckedPosition: SIMD3<Double>(0, 0, state.position.z),
                uncheckedVelocity: SIMD3<Double>(0, 0, state.velocity.z),
                uncheckedOrientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
                uncheckedAngularVelocity: SIMD3<Double>(repeating: 0)
            )
        }
    }
}
