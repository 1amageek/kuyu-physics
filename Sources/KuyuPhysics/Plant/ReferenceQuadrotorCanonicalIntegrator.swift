import KuyuCore
import simd

public struct ReferenceQuadrotorCanonicalIntegrator: Sendable {
    public enum IntegrationError: Error, Equatable {
        case invalidTimeStep(Double)
        case duplicateForceTerms([QuadrotorForceTermID])
        case missingForceTerms([QuadrotorForceTermID])
        case unsupportedImplicitTerms([QuadrotorForceTermID])
    }

    public init() {}

    public func step(
        state: ReferenceQuadrotorState,
        model: ReferenceQuadrotorPhysicsModel,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState,
        fidelity: ReferenceQuadrotorFidelity,
        delta: Double
    ) throws -> ReferenceQuadrotorState {
        guard delta.isFinite, delta > 0 else {
            throw IntegrationError.invalidTimeStep(delta)
        }
        let duplicateTermIDs = model.duplicateTermIDs(fidelity: fidelity)
        guard duplicateTermIDs.isEmpty else {
            throw IntegrationError.duplicateForceTerms(duplicateTermIDs)
        }
        let missingTermIDs = model.missingTermIDs(fidelity: fidelity)
        guard missingTermIDs.isEmpty else {
            throw IntegrationError.missingForceTerms(missingTermIDs)
        }
        let implicitTermIDs = model.implicitTermIDs(fidelity: fidelity)
        guard implicitTermIDs.isEmpty else {
            throw IntegrationError.unsupportedImplicitTerms(implicitTermIDs)
        }
        let projectedState = fidelity.constraint.project(state: state)
        let k1 = try model.derivative(
            state: projectedState,
            motorThrusts: motorThrusts,
            disturbances: disturbances,
            fidelity: fidelity
        )
        let s2 = fidelity.constraint.project(state: projectedState.applying(derivative: k1, scale: delta * 0.5))
        let k2 = try model.derivative(state: s2, motorThrusts: motorThrusts, disturbances: disturbances, fidelity: fidelity)
        let s3 = fidelity.constraint.project(state: projectedState.applying(derivative: k2, scale: delta * 0.5))
        let k3 = try model.derivative(state: s3, motorThrusts: motorThrusts, disturbances: disturbances, fidelity: fidelity)
        let s4 = fidelity.constraint.project(state: projectedState.applying(derivative: k3, scale: delta))
        let k4 = try model.derivative(state: s4, motorThrusts: motorThrusts, disturbances: disturbances, fidelity: fidelity)

        let step = delta / 6.0
        let position = projectedState.position + weightedSum(k1.position, k2.position, k3.position, k4.position) * step
        let velocity = projectedState.velocity + weightedSum(k1.velocity, k2.velocity, k3.velocity, k4.velocity) * step
        let orientationVector = projectedState.orientation.vector
            + weightedSum(k1.orientation, k2.orientation, k3.orientation, k4.orientation) * step
        let angularVelocity = projectedState.angularVelocity
            + weightedSum(k1.angularVelocity, k2.angularVelocity, k3.angularVelocity, k4.angularVelocity) * step

        let next = ReferenceQuadrotorState(
            uncheckedPosition: position,
            uncheckedVelocity: velocity,
            uncheckedOrientation: simd_quatd(vector: orientationVector).normalizedQuat,
            uncheckedAngularVelocity: angularVelocity
        )
        return fidelity.constraint.project(state: next)
    }

    private func weightedSum(
        _ k1: SIMD3<Double>,
        _ k2: SIMD3<Double>,
        _ k3: SIMD3<Double>,
        _ k4: SIMD3<Double>
    ) -> SIMD3<Double> {
        k1 + (k2 * 2.0) + (k3 * 2.0) + k4
    }

    private func weightedSum(
        _ k1: SIMD4<Double>,
        _ k2: SIMD4<Double>,
        _ k3: SIMD4<Double>,
        _ k4: SIMD4<Double>
    ) -> SIMD4<Double> {
        k1 + (k2 * 2.0) + (k3 * 2.0) + k4
    }
}
