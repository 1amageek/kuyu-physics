import EmbodimentContract
import KuyuCore

public struct ArticulatedRigidBodySimulationRequest: Sendable, Equatable {
    public let body: KuyuBodyModel
    public let world: KuyuWorldModel
    public let embodiment: EmbodimentContract
    public let compatibilityReport: CompatibilityReport?
    public let determinism: DeterminismConfig
    public let readinessLevel: ReadinessLevel
    public let duration: Double
    public let timeStep: TimeStep
    public let seed: ScenarioSeed

    public init(
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        embodiment: EmbodimentContract,
        compatibilityReport: CompatibilityReport? = nil,
        determinism: DeterminismConfig,
        readinessLevel: ReadinessLevel = .dynamicSimulation,
        duration: Double = 6.0,
        timeStep: TimeStep,
        seed: ScenarioSeed = ScenarioSeed(0)
    ) {
        self.body = body
        self.world = world
        self.embodiment = embodiment
        self.compatibilityReport = compatibilityReport
        self.determinism = determinism
        self.readinessLevel = readinessLevel
        self.duration = duration
        self.timeStep = timeStep
        self.seed = seed
    }
}
