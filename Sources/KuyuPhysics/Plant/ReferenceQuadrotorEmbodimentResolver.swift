import EmbodimentContract
import Foundation
import KuyuCore

public struct ReferenceQuadrotorEmbodimentResolver: ReferenceQuadrotorEmbodimentResolving, Sendable {
    public enum ResolutionError: Error, Equatable {
        case unsupportedActuatorCount(Int)
        case invalidActuatorMinimum(id: String, value: Double)
        case invalidActuatorMaximum(id: String, value: Double)
        case invalidActuatorRateLimit(id: String, value: Double)
        case missingActuatorDynamics(String)
        case invalidActuatorTimeConstant(id: String, value: Double)
        case inconsistentActuatorMaximums([Double])
        case inconsistentActuatorTimeConstants([Double])
        case invalidGravity(Axis3)
    }

    public init() {}

    public func resolution(for robot: LoadedKuyuRobot) throws -> ReferenceQuadrotorEmbodimentResolution {
        let actuators = robot.embodiment.actuators
        guard actuators.count == 1 || actuators.count == 4 else {
            throw ResolutionError.unsupportedActuatorCount(actuators.count)
        }

        for actuator in actuators {
            guard actuator.limits.min.isFinite, abs(actuator.limits.min) <= 1e-12 else {
                throw ResolutionError.invalidActuatorMinimum(
                    id: actuator.id,
                    value: actuator.limits.min
                )
            }
            guard actuator.limits.max.isFinite, actuator.limits.max > 0 else {
                throw ResolutionError.invalidActuatorMaximum(
                    id: actuator.id,
                    value: actuator.limits.max
                )
            }
            guard actuator.limits.rateLimitPerSecond.isFinite,
                  actuator.limits.rateLimitPerSecond > 0 else {
                throw ResolutionError.invalidActuatorRateLimit(
                    id: actuator.id,
                    value: actuator.limits.rateLimitPerSecond
                )
            }
            guard let dynamics = actuator.dynamics else {
                throw ResolutionError.missingActuatorDynamics(actuator.id)
            }
            guard dynamics.timeConstantSeconds.isFinite,
                  dynamics.timeConstantSeconds > 0 else {
                throw ResolutionError.invalidActuatorTimeConstant(
                    id: actuator.id,
                    value: dynamics.timeConstantSeconds
                )
            }
        }

        let maximums = actuators.map(\.limits.max)
        guard valuesAreUniform(maximums) else {
            throw ResolutionError.inconsistentActuatorMaximums(maximums)
        }
        let timeConstants = actuators.compactMap(\.dynamics?.timeConstantSeconds)
        guard valuesAreUniform(timeConstants) else {
            throw ResolutionError.inconsistentActuatorTimeConstants(timeConstants)
        }

        let gravityVector = Axis3(
            x: robot.world.gravity.acceleration.x,
            y: robot.world.gravity.acceleration.y,
            z: robot.world.gravity.acceleration.z
        )
        let gravity = sqrt(
            gravityVector.x * gravityVector.x
                + gravityVector.y * gravityVector.y
                + gravityVector.z * gravityVector.z
        )
        guard robot.world.gravity.kind == .uniform, gravity.isFinite, gravity > 0 else {
            throw ResolutionError.invalidGravity(gravityVector)
        }

        let inertial = try KuyuModelLoader().loadPlantInertialProperties(robot: robot)
        let baseline = ReferenceQuadrotorParameters.baseline
        let parameters = try ReferenceQuadrotorParameters(
            mass: inertial.mass,
            inertia: inertial.inertia,
            armLength: baseline.armLength,
            motorTimeConstant: timeConstants[0],
            maxThrust: maximums[0],
            yawCoefficient: baseline.yawCoefficient,
            gravity: gravity,
            aerodynamics: baseline.aerodynamics
        )
        let normalizedRateLimit = zip(actuators, maximums).map { actuator, maximum in
            actuator.limits.rateLimitPerSecond / maximum
        }.min() ?? 0
        return ReferenceQuadrotorEmbodimentResolution(
            parameters: parameters,
            normalizedActuatorRateLimitPerSecond: normalizedRateLimit
        )
    }

    private func valuesAreUniform(_ values: [Double]) -> Bool {
        guard let first = values.first else { return false }
        return values.dropFirst().allSatisfy { abs($0 - first) <= 1e-12 }
    }
}
