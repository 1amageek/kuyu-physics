import Foundation
import KuyuCore

extension ArticulatedRigidBodySimulator {
    func exactStepCount(duration: Double, timeStep: Double) throws -> Int {
        let rawStepCount = duration / timeStep
        let roundedStepCount = rawStepCount.rounded()
        guard abs(rawStepCount - roundedStepCount) <= 1e-9 else {
            throw SimulationError.durationStepMismatch(duration: duration, timeStep: timeStep)
        }
        let stepCount = Int(roundedStepCount)
        guard stepCount > 0 else {
            throw SimulationError.invalidDuration(duration)
        }
        return stepCount
    }

    func validateSupportedWorld(_ world: KuyuWorldModel) throws {
        guard world.integrator.kind == .semiImplicitEuler else {
            throw SimulationError.unsupportedWorld("integrator.\(world.integrator.kind.rawValue)")
        }
        switch world.contact.mode {
        case .disabled:
            guard world.solver.kind == .disabledContact else {
                throw SimulationError.unsupportedWorld(
                    "contact.disabled.solver.\(world.solver.kind.rawValue)"
                )
            }
        case .penalty, .constraint:
            guard world.solver.kind == .deterministicConstraint else {
                throw SimulationError.unsupportedWorld(
                    "contact.\(world.contact.mode.rawValue).solver.\(world.solver.kind.rawValue)"
                )
            }
        }
    }

    func validateNumericalStability(
        bindings: [ArticulatedJointBinding],
        substepDelta: Double
    ) throws {
        guard substepDelta.isFinite, substepDelta > 0 else {
            throw SimulationError.nonFiniteState("substepDelta")
        }
        for binding in bindings {
            let timeConstant = binding.actuator.dynamics?.timeConstantSeconds ?? 0.001
            guard substepDelta <= timeConstant + 1e-12 else {
                throw SimulationError.unstableTimeStep(
                    substep: substepDelta,
                    timeConstant: timeConstant,
                    actuator: binding.actuator.id
                )
            }
        }
    }

    func validateContactNumericalStability(
        world: KuyuWorldModel,
        dynamics: [ArticulatedJointDynamics],
        substepDelta: Double
    ) throws {
        guard world.contact.mode == .penalty else { return }
        guard let stiffness = world.contact.stiffness else {
            throw SimulationError.unsupportedWorld("contact.penalty.stiffness")
        }
        let minimumInertia = dynamics.map(\.effectiveInertia).min() ?? 1e-6
        let contactTimeConstant = sqrt(max(minimumInertia, 1e-9) / stiffness)
        guard substepDelta <= contactTimeConstant * 0.25 + 1e-12 else {
            throw SimulationError.unstableTimeStep(
                substep: substepDelta,
                timeConstant: contactTimeConstant * 0.25,
                actuator: "contact.penalty"
            )
        }
    }

    func validateProviderDrives(_ drives: [DriveIntent], expectedCount: Int) throws {
        guard drives.count == expectedCount else {
            throw SimulationError.invalidDriveProviderOutput(
                "drive-count expected=\(expectedCount) actual=\(drives.count)"
            )
        }
        for (index, drive) in drives.enumerated() {
            guard drive.index.rawValue == UInt32(index) else {
                throw SimulationError.invalidDriveProviderOutput(
                    "drive-index expected=\(index) actual=\(drive.index.rawValue)"
                )
            }
            guard drive.activation.isFinite else {
                throw SimulationError.nonFiniteState("drive[\(index)]")
            }
        }
    }
}
