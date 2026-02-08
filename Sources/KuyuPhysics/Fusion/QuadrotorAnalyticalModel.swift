import Foundation
import KuyuCore

/// Adapter that wraps ReferenceQuadrotorPlantEngine to conform to AnalyticalModel.
/// Converts ActuatorValue actions to motor thrusts and integrates the ODE.
///
/// Time tracking uses plant.timeStep.delta (the fixed integration step)
/// because ReferenceQuadrotorPlantEngine.integrate() uses timeStep.delta internally for RK4.
/// The `dt` parameter in predict() is ignored — the physics step size is fixed at construction.
public struct QuadrotorAnalyticalModel: AnalyticalModel {
    public typealias State = QuadrotorAnalyticalState

    public enum ModelError: Error, Equatable {
        case invalidActuatorIndex(Int)
    }

    private var plant: ReferenceQuadrotorPlantEngine
    private let initialState: ReferenceQuadrotorState
    private let initialMotorThrusts: MotorThrusts
    private var stepIndex: UInt64 = 0
    private var time: TimeInterval = 0

    public init(plant: ReferenceQuadrotorPlantEngine) {
        self.plant = plant
        self.initialState = plant.store.state
        self.initialMotorThrusts = plant.store.motorThrusts
    }

    public var currentState: QuadrotorAnalyticalState {
        QuadrotorAnalyticalState(snapshot: plant.snapshot())
    }

    public mutating func predict(action: [ActuatorValue], dt: TimeInterval) throws -> QuadrotorAnalyticalState {
        var thrusts: [Double] = [0, 0, 0, 0]
        for actuator in action {
            let idx = Int(actuator.index.rawValue)
            guard idx >= 0, idx < 4 else {
                throw ModelError.invalidActuatorIndex(idx)
            }
            thrusts[idx] = max(0, actuator.value)
        }
        plant.store.motorThrusts = try MotorThrusts(
            f1: thrusts[0], f2: thrusts[1], f3: thrusts[2], f4: thrusts[3]
        )

        stepIndex += 1
        time += plant.timeStep.delta
        let worldTime = try WorldTime(stepIndex: stepIndex, time: time)
        try plant.integrate(time: worldTime)

        return QuadrotorAnalyticalState(snapshot: plant.snapshot())
    }

    public mutating func reset() throws {
        stepIndex = 0
        time = 0
        plant.store.state = initialState
        plant.store.motorThrusts = initialMotorThrusts
        plant.store.disturbances = .zero
    }
}
