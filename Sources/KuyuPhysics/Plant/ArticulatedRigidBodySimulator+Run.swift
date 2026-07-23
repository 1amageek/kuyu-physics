import EmbodimentContract
import Foundation
import KuyuCore

public extension ArticulatedRigidBodySimulator {
    func run(
        request: ArticulatedRigidBodySimulationRequest,
        control: SimulationControl? = nil,
        telemetry: WorldStepTelemetry? = nil
    ) async throws -> SimulationLog {
        return try await run(
            request: request,
            driveProvider: ArticulatedSineDriveProvider(),
            control: control,
            telemetry: telemetry
        )
    }

    func run<DriveProvider: ArticulatedRigidBodyDriveProvider>(
        request: ArticulatedRigidBodySimulationRequest,
        driveProvider: DriveProvider,
        control: SimulationControl? = nil,
        telemetry: WorldStepTelemetry? = nil
    ) async throws -> SimulationLog {
        guard request.duration.isFinite, request.duration > 0 else {
            throw SimulationError.invalidDuration(request.duration)
        }
        let stepCount = try exactStepCount(duration: request.duration, timeStep: request.timeStep.delta)
        guard abs(request.timeStep.delta - request.world.time.fixedStepSeconds) <= 1e-12 else {
            throw SimulationError.timeStepMismatch(
                request: request.timeStep.delta,
                world: request.world.time.fixedStepSeconds
            )
        }
        try validateSupportedWorld(request.world)
        do {
            _ = try ReadinessGate().validate(
                body: request.body,
                world: request.world,
                embodiment: request.embodiment,
                report: request.compatibilityReport,
                requiredLevel: request.readinessLevel
            )
        } catch {
            throw SimulationError.readinessFailed(String(describing: error))
        }

        let movableJoints = request.body.joints.filter { joint in
            joint.mimic == nil && (joint.kind == .revolute || joint.kind == .continuous || joint.kind == .prismatic)
        }
        guard !movableJoints.isEmpty else {
            throw SimulationError.invalidBody("movable-joints")
        }
        let driveSignals = try orderedDriveSignals(from: request.embodiment)
        let actuatorSignals = request.embodiment.signals.actuator.sorted { $0.index < $1.index }
        guard driveSignals.count == movableJoints.count else {
            throw SimulationError.invalidBody("drive-joint-count")
        }

        var motorNerve = try MotorNerveChain(contract: request.embodiment)
        let jointBindings = try bindings(
            joints: movableJoints,
            body: request.body,
            embodiment: request.embodiment,
            actuatorSignals: actuatorSignals
        )
        let jointRanges = try ranges(bindings: jointBindings)
        try validateNumericalStability(
            bindings: jointBindings,
            substepDelta: request.timeStep.delta / Double(max(request.world.time.substeps, 1))
        )
        let jointIDs = jointBindings.map(\.joint.id)
        let driveSignalIDs = driveSignals.map(\.id)
        let actuatorSignalIDs = actuatorSignals.map(\.id)
        var provider = driveProvider
        let initialPositions = try initialPositions(bindings: jointBindings, ranges: jointRanges)
        try provider.reset(context: ArticulatedRigidBodyDriveProviderResetContext(
            seed: request.seed,
            jointIDs: jointIDs,
            driveSignalIDs: driveSignalIDs,
            jointRanges: jointRanges
        ))
        let stateModel = ArticulatedStateModel(body: request.body, world: request.world)
        let jointDynamics = try stateModel.dynamics(for: jointBindings)
        let snapshotTopology = try snapshotTopology(body: request.body)
        let contactSolver = try makeContactSolver(
            body: request.body,
            world: request.world,
            bindings: jointBindings,
            dynamics: jointDynamics,
            topology: snapshotTopology
        )
        try validateContactNumericalStability(
            world: request.world,
            dynamics: jointDynamics,
            substepDelta: request.timeStep.delta / Double(max(request.world.time.substeps, 1))
        )
        var state = ArticulatedState(
            position: initialPositions,
            velocity: Array(repeating: 0, count: jointBindings.count),
            torque: Array(repeating: 0, count: jointBindings.count)
        )
        var contactMetrics = ArticulatedContactMetrics.disabled
        var targets = initialPositions
        var logs: [WorldStepLog] = []
        logs.reserveCapacity(stepCount)

        for step in 0..<stepCount {
            if let control = control {
                try await control.checkpoint()
            }

            let time = try WorldTime(stepIndex: UInt64(step), time: Double(step) * request.timeStep.delta)
            let providerContext = ArticulatedRigidBodyDriveContext(
                time: time,
                jointIDs: jointIDs,
                driveSignalIDs: driveSignalIDs,
                actuatorSignalIDs: actuatorSignalIDs,
                jointRanges: jointRanges,
                positions: state.position,
                velocities: state.velocity,
                targets: targets,
                torques: state.torque
            )
            let drives = try provider.driveIntents(context: providerContext)
            try validateProviderDrives(drives, expectedCount: jointBindings.count)
            let actuatorValues = try motorNerve.update(
                input: drives,
                corrections: [],
                telemetry: MotorNerveTelemetry(actuatorTelemetry: try actuatorTelemetry(
                    state: state,
                    bindings: jointBindings,
                    actuatorSignals: actuatorSignals
                )),
                time: time
            )
            targets = try jointTargets(values: actuatorValues, bindings: jointBindings, actuatorSignals: actuatorSignals)
            let stepResult = try stateModel.step(
                state: state,
                targets: targets,
                dynamics: jointDynamics,
                deltaTime: request.timeStep.delta,
                contactSolver: contactSolver
            )
            state = stepResult.state
            contactMetrics = stepResult.contactMetrics

            let log = try makeStepLog(
                body: request.body,
                world: request.world,
                time: time,
                snapshotTopology: snapshotTopology,
                bindings: jointBindings,
                state: state,
                targets: targets,
                contactMetrics: contactMetrics,
                drives: drives,
                actuatorValues: actuatorValues,
                actuatorSignals: actuatorSignals,
                motorNerveTrace: motorNerve.lastTrace
            )
            telemetry?(log)
            logs.append(log)

            if (step % 20) == 0 {
                try Task.checkCancellation()
            }
            if (step % 200) == 0 || (control != nil && (step % 20) == 0) {
                await Task.yield()
            }
        }

        return SimulationLog(
            scenarioId: try ScenarioID("\(request.body.bodyID.uppercased())-DYN-1"),
            seed: request.seed,
            timeStep: request.timeStep,
            determinism: request.determinism,
            configHash: try configHash(request: request, providerID: provider.providerID),
            events: logs
        )
    }
}
