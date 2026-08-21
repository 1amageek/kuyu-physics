import KuyuCore

public protocol ReferenceQuadrotorCanonicalExecuting: Sendable {
    var executorVersion: String { get }

    func generalizedForce(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState,
        environment: WorldEnvironment,
        activeTerms: Set<QuadrotorForceTermID>
    ) throws -> QuadrotorGeneralizedForce

    func derivative(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        force: QuadrotorGeneralizedForce
    ) throws -> ReferenceQuadrotorStateDerivative

    func observables(
        program: CanonicalDynamicsProgram,
        state: ReferenceQuadrotorState,
        parameters: ReferenceQuadrotorParameters,
        environment: WorldEnvironment,
        force: QuadrotorGeneralizedForce
    ) throws -> ReferenceQuadrotorCanonicalObservables
}
