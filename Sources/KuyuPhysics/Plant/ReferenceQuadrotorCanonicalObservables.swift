import simd

public struct ReferenceQuadrotorCanonicalObservables: Sendable, Equatable {
    public let angularVelocityBody: SIMD3<Double>
    public let specificForceBody: SIMD3<Double>

    public init(
        angularVelocityBody: SIMD3<Double>,
        specificForceBody: SIMD3<Double>
    ) {
        self.angularVelocityBody = angularVelocityBody
        self.specificForceBody = specificForceBody
    }
}
