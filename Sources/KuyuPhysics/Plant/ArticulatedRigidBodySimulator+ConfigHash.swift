import EmbodimentContract
import KuyuCore

extension ArticulatedRigidBodySimulator {
    func configHash(
        request: ArticulatedRigidBodySimulationRequest,
        providerID: String
    ) throws -> String {
        try ConfigHash.hash(ArticulatedSimulationConfigEnvelope(
            schemaVersion: "kuyu.articulated.simulation-config.v1",
            simulatorVersion: "articulated-dynamic-v3",
            body: request.body,
            world: request.world,
            embodiment: request.embodiment,
            compatibilityReport: request.compatibilityReport,
            determinism: request.determinism,
            readinessLevel: request.readinessLevel,
            duration: request.duration,
            timeStep: request.timeStep,
            seed: request.seed,
            driveProviderID: providerID
        ))
    }
}

private struct ArticulatedSimulationConfigEnvelope: Sendable, Encodable, Equatable {
    let schemaVersion: String
    let simulatorVersion: String
    let body: KuyuBodyModel
    let world: KuyuWorldModel
    let embodiment: EmbodimentContract
    let compatibilityReport: CompatibilityReport?
    let determinism: DeterminismConfig
    let readinessLevel: ReadinessLevel
    let duration: Double
    let timeStep: TimeStep
    let seed: ScenarioSeed
    let driveProviderID: String
}
