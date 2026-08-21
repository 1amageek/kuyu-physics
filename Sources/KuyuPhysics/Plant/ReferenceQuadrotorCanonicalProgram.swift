public enum ReferenceQuadrotorCanonicalProgram {
    public static func make() throws -> CanonicalDynamicsProgram {
        let layouts = try ReferenceQuadrotorCanonicalLayoutsSet()
        let allTermIDs = try QuadrotorForceTermID.allCases.map(canonicalTermID)
        let remainingSinglePropTerms = try QuadrotorForceTermID.allCases
            .filter { ![.gravity, .propulsion, .disturbance].contains($0) }
            .map(canonicalTermID)

        let content = CanonicalDynamicsProgramContent(
            id: "reference_quadrotor_dynamics",
            schemaVersion: CanonicalDynamicsProgram.currentSchemaVersion,
            layouts: layouts.all,
            layoutBindings: try CanonicalDynamicsLayoutBindings(
                state: layouts.state.id,
                derivative: layouts.derivative.id,
                parameters: layouts.parameters.id,
                mixer: layouts.mixer.id,
                control: layouts.control.id,
                disturbance: layouts.disturbance.id,
                environment: layouts.environment.id,
                generalizedForce: layouts.generalizedForce.id,
                observables: layouts.observables.id
            ),
            controlSemantics: .realizedMotorThrust,
            forceTerms: try forceTerms(layouts: layouts),
            derivativeGraph: try derivativeGraph(layouts),
            observableGraph: try observableGraph(layouts),
            fidelities: [
                try CanonicalFidelityDefinition(
                    id: "full",
                    active: allTermIDs,
                    worldModelTargets: [],
                    ignored: [],
                    projection: .identity
                ),
                try CanonicalFidelityDefinition(
                    id: "single_prop",
                    active: [
                        canonicalTermID(.gravity),
                        canonicalTermID(.propulsion),
                        canonicalTermID(.disturbance),
                    ],
                    worldModelTargets: remainingSinglePropTerms,
                    ignored: [],
                    projection: .referenceQuadrotorVerticalOnly
                ),
            ],
            integration: try CanonicalIntegrationPlan(
                scheme: .rungeKutta4,
                projectionStages: CanonicalIntegrationStage.allCases
            )
        )
        return try CanonicalDynamicsProgram(content: content)
    }

    static func canonicalTermID(
        _ id: QuadrotorForceTermID
    ) throws -> CanonicalForceTermID {
        let rawValue: String
        switch id {
        case .gravity:
            rawValue = "gravity"
        case .propulsion:
            rawValue = "propulsion"
        case .thrustDensityScaling:
            rawValue = "thrust_density_scaling"
        case .disturbance:
            rawValue = "disturbance"
        case .aerodynamicDrag:
            rawValue = "aerodynamic_drag"
        case .aerodynamicLift:
            rawValue = "aerodynamic_lift"
        case .buoyancy:
            rawValue = "buoyancy"
        case .angularDrag:
            rawValue = "angular_drag"
        case .gyroscopic:
            rawValue = "gyroscopic"
        }
        return try CanonicalForceTermID(rawValue)
    }
}
