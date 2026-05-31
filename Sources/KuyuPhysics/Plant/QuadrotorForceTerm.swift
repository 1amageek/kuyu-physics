public protocol QuadrotorForceTerm: Sendable {
    var id: QuadrotorForceTermID { get }
    var stiffness: ForceTermStiffness { get }

    func generalizedForce(context: QuadrotorForceTermContext) -> QuadrotorGeneralizedForce
}
