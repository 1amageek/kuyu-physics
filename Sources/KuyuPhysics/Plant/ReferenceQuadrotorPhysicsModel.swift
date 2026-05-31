import KuyuCore
import simd

public struct ReferenceQuadrotorPhysicsModel: Sendable {
    public enum ModelError: Error, Equatable {
        case duplicateForceTerms([QuadrotorForceTermID])
        case missingForceTerms([QuadrotorForceTermID])
    }

    public let parameters: ReferenceQuadrotorParameters
    public let mixer: ReferenceQuadrotorMixer
    public let environment: WorldEnvironment
    public let terms: [AnyQuadrotorForceTerm]
    public let availableMask: QuadrotorForceTermMask
    public let implicitMask: QuadrotorForceTermMask
    public let duplicateMask: QuadrotorForceTermMask

    public init(
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer,
        environment: WorldEnvironment = .standard,
        terms: [AnyQuadrotorForceTerm] = ReferenceQuadrotorForceTerms.canonical
    ) {
        self.parameters = parameters
        self.mixer = mixer
        self.environment = environment
        self.terms = terms
        var seenMask = QuadrotorForceTermMask()
        var duplicateMask = QuadrotorForceTermMask()
        for term in terms {
            if seenMask.contains(term.mask) {
                duplicateMask.formUnion(term.mask)
            }
            seenMask.formUnion(term.mask)
        }
        self.availableMask = seenMask
        self.implicitMask = terms.reduce([]) { partial, term in
            term.stiffness == .implicit ? partial.union(term.mask) : partial
        }
        self.duplicateMask = duplicateMask
    }

    public func generalizedForce(
        state: ReferenceQuadrotorState,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState,
        fidelity: ReferenceQuadrotorFidelity
    ) throws -> QuadrotorGeneralizedForce {
        let context = QuadrotorForceTermContext(
            state: state,
            motorThrusts: motorThrusts,
            disturbances: disturbances,
            parameters: parameters,
            mixer: mixer,
            environment: environment
        )
        return try generalizedForce(context: context, activeMask: fidelity.activeMask)
    }

    public func missingTermIDs(fidelity: ReferenceQuadrotorFidelity) -> [QuadrotorForceTermID] {
        fidelity.activeMask.subtracting(availableMask).orderedIDs
    }

    public func duplicateTermIDs(fidelity: ReferenceQuadrotorFidelity) -> [QuadrotorForceTermID] {
        fidelity.activeMask.intersection(duplicateMask).orderedIDs
    }

    public func implicitTermIDs(fidelity: ReferenceQuadrotorFidelity) -> [QuadrotorForceTermID] {
        fidelity.activeMask.intersection(implicitMask).orderedIDs
    }

    public func residualTarget(
        low: ReferenceQuadrotorFidelity,
        high: ReferenceQuadrotorFidelity,
        state: ReferenceQuadrotorState,
        motorThrusts: MotorThrusts,
        disturbances: DisturbanceState
    ) throws -> QuadrotorGeneralizedForce {
        let context = QuadrotorForceTermContext(
            state: state,
            motorThrusts: motorThrusts,
            disturbances: disturbances,
            parameters: parameters,
            mixer: mixer,
            environment: environment
        )
        return try generalizedForce(context: context, activeMask: low.residualTargetMask(toward: high))
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
        let derivative = ReferenceQuadrotorDynamics.derivative(
            state: state,
            force: force,
            parameters: parameters
        )
        return fidelity.constraint.project(derivative: derivative)
    }

    private func generalizedForce(
        context: QuadrotorForceTermContext,
        activeMask: QuadrotorForceTermMask
    ) throws -> QuadrotorGeneralizedForce {
        let duplicateTermIDs = activeMask.intersection(duplicateMask).orderedIDs
        guard duplicateTermIDs.isEmpty else {
            throw ModelError.duplicateForceTerms(duplicateTermIDs)
        }
        let missingTermIDs = activeMask.subtracting(availableMask).orderedIDs
        guard missingTermIDs.isEmpty else {
            throw ModelError.missingForceTerms(missingTermIDs)
        }
        var total = QuadrotorGeneralizedForce.zero
        for term in terms where activeMask.contains(term.mask) {
            total += term.generalizedForce(context: context)
        }
        return total
    }
}
