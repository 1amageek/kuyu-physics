struct ReferenceQuadrotorCanonicalLayoutsSet {
    let state: CanonicalBufferLayout
    let derivative: CanonicalBufferLayout
    let parameters: CanonicalBufferLayout
    let mixer: CanonicalBufferLayout
    let control: CanonicalBufferLayout
    let disturbance: CanonicalBufferLayout
    let environment: CanonicalBufferLayout
    let generalizedForce: CanonicalBufferLayout
    let observables: CanonicalBufferLayout

    init() throws {
        state = try ReferenceQuadrotorCanonicalLayouts.stateLayout()
        derivative = try ReferenceQuadrotorCanonicalLayouts.derivativeLayout()
        parameters = try ReferenceQuadrotorCanonicalLayouts.parameterLayout()
        mixer = try ReferenceQuadrotorCanonicalLayouts.mixerLayout()
        control = try ReferenceQuadrotorCanonicalLayouts.controlLayout()
        disturbance = try ReferenceQuadrotorCanonicalLayouts.disturbanceLayout()
        environment = try ReferenceQuadrotorCanonicalLayouts.environmentLayout()
        generalizedForce = try ReferenceQuadrotorCanonicalLayouts.generalizedForceLayout()
        observables = try ReferenceQuadrotorCanonicalLayouts.observableLayout()
    }

    var all: [CanonicalBufferLayout] {
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
}
