import KuyuCore
import simd

public struct ReferenceQuadrotorPhysicsModel: Sendable {
    public let parameters: ReferenceQuadrotorParameters
    public let mixer: ReferenceQuadrotorMixer
    public let environment: WorldEnvironment
    public let program: CanonicalDynamicsProgram
    public let executor: any ReferenceQuadrotorCanonicalExecuting

    public init(
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer,
        environment: WorldEnvironment = .standard,
        program: CanonicalDynamicsProgram,
        executor: any ReferenceQuadrotorCanonicalExecuting = ReferenceQuadrotorScalarDynamicsExecutor()
    ) {
        self.parameters = parameters
        self.mixer = mixer
        self.environment = environment
        self.program = program
        self.executor = executor
    }

    public func generalizedForce(
        state: ReferenceQuadrotorState,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState,
        fidelity: ReferenceQuadrotorFidelity
    ) throws -> QuadrotorGeneralizedForce {
        try executor.generalizedForce(
            program: program,
            state: state,
            parameters: parameters,
            mixer: mixer,
            motorThrusts: motorThrusts,
            disturbances: disturbances,
            environment: environment,
            activeTerms: fidelity.active
        )
    }

    public func residualTarget(
        low: ReferenceQuadrotorFidelity,
        high: ReferenceQuadrotorFidelity,
        state: ReferenceQuadrotorState,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState
    ) throws -> QuadrotorGeneralizedForce {
        try executor.generalizedForce(
            program: program,
            state: state,
            parameters: parameters,
            mixer: mixer,
            motorThrusts: motorThrusts,
            disturbances: disturbances,
            environment: environment,
            activeTerms: low.residualTargetIDs(toward: high)
        )
    }

    public func derivative(
        state: ReferenceQuadrotorState,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState,
        fidelity: ReferenceQuadrotorFidelity
    ) throws -> ReferenceQuadrotorStateDerivative {
        let force = try generalizedForce(
            state: state,
            motorThrusts: motorThrusts,
            disturbances: disturbances,
            fidelity: fidelity
        )
        let derivative = try executor.derivative(
            program: program,
            state: state,
            parameters: parameters,
            force: force
        )
        return fidelity.constraint.project(derivative: derivative)
    }

    public func observables(
        state: ReferenceQuadrotorState,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState,
        fidelity: ReferenceQuadrotorFidelity
    ) throws -> ReferenceQuadrotorCanonicalObservables {
        let force = try generalizedForce(
            state: state,
            motorThrusts: motorThrusts,
            disturbances: disturbances,
            fidelity: fidelity
        )
        return try executor.observables(
            program: program,
            state: state,
            parameters: parameters,
            environment: environment,
            force: force
        )
    }
}
