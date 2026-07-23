import Foundation
import KuyuCore

public struct LoadedRobotDescriptorCorpusAcceptanceRequest: Sendable, Equatable {
    public let corpusID: String
    public let loadedRobots: [LoadedKuyuRobot]
    public let outputDirectory: URL
    public let artifactRoot: URL?
    public let generatedAt: String?
    public let requiredReadiness: ReadinessLevel
    public let durationSteps: Int
    public let seedBase: UInt64
    public let determinism: DeterminismConfig

    public init(
        corpusID: String,
        loadedRobots: [LoadedKuyuRobot],
        outputDirectory: URL,
        artifactRoot: URL? = nil,
        generatedAt: String? = nil,
        requiredReadiness: ReadinessLevel = .dynamicSimulation,
        durationSteps: Int = 4,
        seedBase: UInt64 = 42,
        determinism: DeterminismConfig = .tier0Strict
    ) {
        self.corpusID = corpusID
        self.loadedRobots = loadedRobots
        self.outputDirectory = outputDirectory
        self.artifactRoot = artifactRoot
        self.generatedAt = generatedAt
        self.requiredReadiness = requiredReadiness
        self.durationSteps = durationSteps
        self.seedBase = seedBase
        self.determinism = determinism
    }
}

public struct LoadedRobotDescriptorCorpusAcceptancePublication: Sendable, Equatable {
    public let artifactURL: URL
    public let summary: DescriptorCorpusAcceptanceSummary

    public init(
        artifactURL: URL,
        summary: DescriptorCorpusAcceptanceSummary
    ) {
        self.artifactURL = artifactURL
        self.summary = summary
    }
}

public enum LoadedRobotDescriptorCorpusAcceptanceError: Error, Equatable {
    case emptyRobots
    case invalidDurationSteps(Int)
    case outputEscapesArtifactRoot(String)
}

public struct LoadedRobotDescriptorCorpusAcceptanceService: Sendable {
    public init() {}

    public func write(
        _ request: LoadedRobotDescriptorCorpusAcceptanceRequest
    ) async throws -> LoadedRobotDescriptorCorpusAcceptancePublication {
        try validateOutputDirectory(request.outputDirectory, artifactRoot: request.artifactRoot)
        let entries = try descriptorCorpusEntries(for: request)
        let summary = try await DescriptorCorpusAcceptanceService().accept(
            corpusID: request.corpusID,
            entries: entries,
            generatedAt: request.generatedAt
        )
        let artifactURL = try DescriptorCorpusAcceptanceArtifactStore().write(
            summary,
            to: request.outputDirectory
        )
        return LoadedRobotDescriptorCorpusAcceptancePublication(
            artifactURL: artifactURL,
            summary: summary
        )
    }

    private func descriptorCorpusEntries(
        for request: LoadedRobotDescriptorCorpusAcceptanceRequest
    ) throws -> [DescriptorCorpusEntry] {
        guard !request.loadedRobots.isEmpty else {
            throw LoadedRobotDescriptorCorpusAcceptanceError.emptyRobots
        }
        guard request.durationSteps > 0 else {
            throw LoadedRobotDescriptorCorpusAcceptanceError.invalidDurationSteps(request.durationSteps)
        }

        return try request.loadedRobots.enumerated().map { index, loadedRobot in
            let robotID = loadedRobot.manifest.robotID.trimmingCharacters(in: .whitespacesAndNewlines)
            let timeStep = try TimeStep(delta: loadedRobot.world.time.fixedStepSeconds)
            return DescriptorCorpusEntry(
                loadedRobot: loadedRobot,
                entryID: "\(robotID)-\(request.requiredReadiness.rawValue)",
                label: "\(loadedRobot.manifest.name) \(request.requiredReadiness.rawValue) descriptor",
                requiredReadiness: request.requiredReadiness,
                duration: timeStep.delta * Double(request.durationSteps),
                timeStep: timeStep,
                seed: ScenarioSeed(request.seedBase + UInt64(index)),
                determinism: request.determinism
            )
        }
    }

    private func validateOutputDirectory(_ outputDirectory: URL, artifactRoot: URL?) throws {
        guard let artifactRoot else {
            return
        }
        let rootPath = artifactRoot.standardizedFileURL.resolvingSymlinksInPath().path
        let outputPath = outputDirectory.standardizedFileURL.resolvingSymlinksInPath().path
        guard outputPath == rootPath || outputPath.hasPrefix(rootPath + "/") else {
            throw LoadedRobotDescriptorCorpusAcceptanceError.outputEscapesArtifactRoot(outputPath)
        }
    }
}
