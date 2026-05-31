import KuyuCore

public struct ReferenceQuadrotorPlantEngine: PlantEngine {
    public enum PlantError: Error, Equatable {
        case nonFiniteState
    }

    public var parameters: ReferenceQuadrotorParameters {
        didSet { model = Self.makeModel(parameters: parameters, mixer: mixer, environment: environment) }
    }
    public var mixer: ReferenceQuadrotorMixer {
        didSet { model = Self.makeModel(parameters: parameters, mixer: mixer, environment: environment) }
    }
    public var store: ReferenceQuadrotorWorldStore
    public let timeStep: TimeStep
    public let environment: WorldEnvironment
    private var model: ReferenceQuadrotorPhysicsModel
    private let integrator: ReferenceQuadrotorCanonicalIntegrator

    public init(
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer,
        store: ReferenceQuadrotorWorldStore,
        timeStep: TimeStep,
        environment: WorldEnvironment = .standard
    ) {
        self.parameters = parameters
        self.mixer = mixer
        self.store = store
        self.timeStep = timeStep
        self.environment = environment
        self.model = Self.makeModel(parameters: parameters, mixer: mixer, environment: environment)
        self.integrator = ReferenceQuadrotorCanonicalIntegrator()
    }

    public mutating func integrate(time: WorldTime) throws {
        _ = time
        let next = try integrator.step(
            state: store.state,
            model: model,
            motorThrusts: store.motorThrusts,
            disturbances: store.disturbances,
            fidelity: .full,
            delta: timeStep.delta
        )

        guard next.position.x.isFinite,
              next.position.y.isFinite,
              next.position.z.isFinite,
              next.velocity.x.isFinite,
              next.velocity.y.isFinite,
              next.velocity.z.isFinite,
              next.angularVelocity.x.isFinite,
              next.angularVelocity.y.isFinite,
              next.angularVelocity.z.isFinite,
              next.orientation.vector.x.isFinite,
              next.orientation.vector.y.isFinite,
              next.orientation.vector.z.isFinite,
              next.orientation.vector.w.isFinite else {
            throw PlantError.nonFiniteState
        }

        store.state = next
    }

    private static func makeModel(
        parameters: ReferenceQuadrotorParameters,
        mixer: ReferenceQuadrotorMixer,
        environment: WorldEnvironment
    ) -> ReferenceQuadrotorPhysicsModel {
        ReferenceQuadrotorPhysicsModel(
            parameters: parameters,
            mixer: mixer,
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
