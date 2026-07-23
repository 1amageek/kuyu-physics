import Foundation
import KuyuCore

extension DescriptorCorpusAcceptanceService {
    func replayEvidence(entry: DescriptorCorpusEntry) async throws -> DescriptorCorpusReplayEvidence {
        let first: SimulationLog
        let second: SimulationLog
        do {
            first = try await replayLog(entry: entry)
            second = try await replayLog(entry: entry)
        } catch {
            throw DescriptorCorpusAcceptanceError.simulationFailed(
                entryID: entry.entryID,
                reason: String(describing: error)
            )
        }

        let replayResult: ReplayCheckResult
        do {
            replayResult = try ReplayChecker().check(reference: first, candidate: second)
        } catch {
            throw DescriptorCorpusAcceptanceError.replayFailed(
                entryID: entry.entryID,
                issues: [String(describing: error)]
            )
        }

        let sortedJSONByteStable: Bool
        do {
            let encoder = JSONEncoder()
            encoder.outputFormatting = [.sortedKeys]
            sortedJSONByteStable = try encoder.encode(first) == encoder.encode(second)
        } catch {
            throw DescriptorCorpusAcceptanceError.replayFailed(
                entryID: entry.entryID,
                issues: ["sorted-json-encode-failed: \(error)"]
            )
        }

        let passed = replayResult.passed && sortedJSONByteStable
        if !passed {
            var issues = replayResult.issues
            if !sortedJSONByteStable {
                issues.append("sorted-json-byte-mismatch")
            }
            throw DescriptorCorpusAcceptanceError.replayFailed(entryID: entry.entryID, issues: issues)
        }

        let contact = contactReplayEvidence(log: first)
        if entry.requiredReadiness == .contactTraining {
            guard let contact, contact.maxActiveContactCount > 0 else {
                throw DescriptorCorpusAcceptanceError.replayFailed(
                    entryID: entry.entryID,
                    issues: ["contact-replay-not-observed"]
                )
            }
        }

        return DescriptorCorpusReplayEvidence(
            passed: passed,
            tier: replayResult.tier,
            stepCount: first.events.count,
            configHash: first.configHash,
            sortedJSONByteStable: sortedJSONByteStable,
            issues: replayResult.issues,
            residuals: replayResult.residuals,
            contact: contactForReplayEvidence(contact, requiredReadiness: entry.requiredReadiness)
        )
    }

    func contactForReplayEvidence(
        _ contact: DescriptorCorpusContactReplayEvidence?,
        requiredReadiness: ReadinessLevel
    ) -> DescriptorCorpusContactReplayEvidence? {
        guard requiredReadiness == .contactTraining || (contact?.maxActiveContactCount ?? 0) > 0 else {
            return nil
        }
        return contact
    }

    func contactReplayEvidence(log: SimulationLog) -> DescriptorCorpusContactReplayEvidence? {
        guard let activeContactCount = maxScalar(in: log, key: "contact.active.count"),
              let maxPenetration = maxScalar(in: log, key: "contact.penetration.max"),
              let maxNormalImpulse = maxScalar(in: log, key: "contact.normalImpulse.max"),
              let maxNormalForce = maxScalar(in: log, key: "contact.normalForce.max"),
              let maxSolverIterations = maxScalar(in: log, key: "contact.solver.iterations") else {
            return nil
        }
        return DescriptorCorpusContactReplayEvidence(
            maxActiveContactCount: activeContactCount,
            maxPenetration: maxPenetration,
            maxNormalImpulse: maxNormalImpulse,
            maxNormalForce: maxNormalForce,
            maxSolverIterations: maxSolverIterations
        )
    }

    func maxScalar(in log: SimulationLog, key: String) -> Double? {
        var maximum: Double?
        for event in log.events {
            guard let value = event.plantState.scalars[key] else { continue }
            maximum = max(maximum ?? value, value)
        }
        return maximum
    }

    func replayLog(entry: DescriptorCorpusEntry) async throws -> SimulationLog {
        switch replayPath(entry: entry) {
        case .articulated:
            let request = ArticulatedRigidBodySimulationRequest(
                body: entry.body,
                world: entry.world,
                embodiment: entry.embodiment,
                compatibilityReport: entry.compatibilityReport,
                determinism: entry.determinism,
                readinessLevel: entry.requiredReadiness == .hardwareParity ? .dynamicSimulation : entry.requiredReadiness,
                duration: entry.duration,
                timeStep: entry.timeStep,
                seed: entry.seed
            )
            return try await ArticulatedRigidBodySimulator().run(request: request)
        case .rigidActuator:
            return try rigidActuatorReplayLog(entry: entry)
        }
    }

    func replayPath(entry: DescriptorCorpusEntry) -> ReplayPath {
        let movableJointCount = entry.body.joints.filter { joint in
            joint.mimic == nil && (joint.kind == .revolute || joint.kind == .continuous || joint.kind == .prismatic)
        }.count
        if movableJointCount == 0, entry.body.actuatorAttachments.isEmpty, !entry.embodiment.actuators.isEmpty {
            return .rigidActuator
        }
        return .articulated
    }
}
