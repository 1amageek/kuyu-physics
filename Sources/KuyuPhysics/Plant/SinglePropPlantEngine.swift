import KuyuCore

public struct SinglePropPlantEngine: PlantEngine {
    public enum PlantError: Error, Equatable {
        case nonFiniteState
    }

    public var parameters: ReferenceQuadrotorParameters {
        didSet { model = Self.makeModel(parameters: parameters, environment: environment) }
    }
    public var store: ReferenceQuadrotorWorldStore
    public let timeStep: TimeStep
    public let environment: WorldEnvironment
    private var model: ReferenceQuadrotorPhysicsModel
    private let integrator: ReferenceQuadrotorCanonicalIntegrator

    public init(
        parameters: ReferenceQuadrotorParameters,
        store: ReferenceQuadrotorWorldStore,
        timeStep: TimeStep,
        environment: WorldEnvironment = .standard
    ) {
        self.parameters = parameters
        self.store = store
        self.timeStep = timeStep
        self.environment = environment
        self.model = Self.makeModel(parameters: parameters, environment: environment)
        self.integrator = ReferenceQuadrotorCanonicalIntegrator()
    }

    public mutating func integrate(time: WorldTime) throws {
        _ = time
        let next = try integrator.step(
            state: store.state,
            model: model,
            motorThrusts: store.motorThrusts,
            disturbances: store.disturbances,
            fidelity: .singleProp,
            delta: timeStep.delta
        )
        guard next.position.z.isFinite, next.velocity.z.isFinite else {
            throw PlantError.nonFiniteState
        }

        store.state = next
    }

    private static func makeModel(
        parameters: ReferenceQuadrotorParameters,
        environment: WorldEnvironment
    ) -> ReferenceQuadrotorPhysicsModel {
        ReferenceQuadrotorPhysicsModel(
            parameters: parameters,
            mixer: ReferenceQuadrotorMixer(
                armLength: parameters.armLength,
                yawCoefficient: parameters.yawCoefficient
            ),
            environment: environment
        )
    }

    public func snapshot() -> PlantStateSnapshot {
        let root = RigidBodySnapshot(
            id: "root",
            position: Axis3(x: store.state.position.x, y: store.state.position.y, z: store.state.position.z),
            velocity: Axis3(x: store.state.velocity.x, y: store.state.velocity.y, z: store.state.velocity.z),
            orientation: QuaternionSnapshot(orientation: store.state.orientation),
            angularVelocity: Axis3(
                x: store.state.angularVelocity.x,
                y: store.state.angularVelocity.y,
                z: store.state.angularVelocity.z
            )
        )
        return PlantStateSnapshot(root: root)
    }

    public func safetyTrace() -> SafetyTrace {
        SafetyTrace(root: snapshot().root)
    }
}
