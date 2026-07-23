import EmbodimentContract
import Foundation
import KuyuCore

public struct DescriptorCorpusEntry: Sendable, Equatable {
    public let entryID: String
    public let robotID: String
    public let label: String
    public let body: KuyuBodyModel
    public let world: KuyuWorldModel
    public let embodiment: EmbodimentContract
    public let compatibilityReport: CompatibilityReport?
    public let hardwareReport: HardwareCalibrationReport?
    public let requiredReadiness: ReadinessLevel
    public let duration: Double
    public let timeStep: TimeStep
    public let seed: ScenarioSeed
    public let determinism: DeterminismConfig

    public init(
        entryID: String,
        robotID: String,
        label: String,
        body: KuyuBodyModel,
        world: KuyuWorldModel,
        embodiment: EmbodimentContract,
        compatibilityReport: CompatibilityReport? = nil,
        hardwareReport: HardwareCalibrationReport? = nil,
        requiredReadiness: ReadinessLevel = .dynamicSimulation,
        duration: Double,
        timeStep: TimeStep,
        seed: ScenarioSeed,
        determinism: DeterminismConfig
    ) {
        self.entryID = entryID
        self.robotID = robotID
        self.label = label
        self.body = body
        self.world = world
        self.embodiment = embodiment
        self.compatibilityReport = compatibilityReport
        self.hardwareReport = hardwareReport
        self.requiredReadiness = requiredReadiness
        self.duration = duration
        self.timeStep = timeStep
        self.seed = seed
        self.determinism = determinism
    }

    public init(
        loadedRobot: LoadedKuyuRobot,
        entryID: String? = nil,
        label: String? = nil,
        hardwareReport: HardwareCalibrationReport? = nil,
        requiredReadiness: ReadinessLevel = .dynamicSimulation,
        duration: Double,
        timeStep: TimeStep,
        seed: ScenarioSeed,
        determinism: DeterminismConfig
    ) {
        self.init(
            entryID: entryID ?? loadedRobot.manifest.robotID,
            robotID: loadedRobot.manifest.robotID,
            label: label ?? loadedRobot.manifest.name,
            body: loadedRobot.body,
            world: loadedRobot.world,
            embodiment: loadedRobot.embodiment,
            compatibilityReport: loadedRobot.compatibilityReport,
            hardwareReport: hardwareReport,
            requiredReadiness: requiredReadiness,
            duration: duration,
            timeStep: timeStep,
            seed: seed,
            determinism: determinism
        )
    }
}
