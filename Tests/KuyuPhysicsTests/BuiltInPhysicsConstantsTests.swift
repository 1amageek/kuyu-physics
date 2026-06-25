import Foundation
import KuyuCore
import simd
import Testing
@testable import KuyuPhysics

@Test func builtInMotorThrustsZeroMatchesValidatedConstruction() throws {
    let validated = try MotorThrusts(f1: 0, f2: 0, f3: 0, f4: 0)

    #expect(MotorThrusts.zero == validated)
}

@Test func builtInIMU6NoiseZeroMatchesValidatedConstruction() throws {
    let validated = try IMU6NoiseConfig(
        gyroNoiseStdDev: 0,
        gyroBias: 0,
        gyroRandomWalkSigma: 0,
        accelNoiseStdDev: 0,
        accelBias: 0,
        accelRandomWalkSigma: 0,
        delaySteps: 0
    )

    #expect(IMU6NoiseConfig.zero == validated)
}

@Test func builtInAerodynamicsConstantsMatchValidatedConstruction() throws {
    let zero = try AerodynamicsParameters(
        dragCoefficient: 0,
        referenceArea: 0,
        liftCoefficient: 0,
        bodyVolume: 0,
        angularDrag: Axis3(x: 0, y: 0, z: 0)
    )
    let baseline = try AerodynamicsParameters(
        dragCoefficient: 1.1,
        referenceArea: 0.05,
        liftCoefficient: 0.2,
        bodyVolume: 0.003,
        angularDrag: Axis3(x: 0.02, y: 0.02, z: 0.04)
    )

    #expect(AerodynamicsParameters.zero == zero)
    #expect(AerodynamicsParameters.baseline == baseline)
}

@Test func builtInReferenceQuadrotorBaselineMatchesValidatedConstruction() throws {
    let validated = try ReferenceQuadrotorParameters(
        mass: 1.0,
        inertia: Axis3(x: 0.005, y: 0.005, z: 0.009),
        armLength: 0.12,
        motorTimeConstant: 0.030,
        maxThrust: 6.0,
        yawCoefficient: 0.020,
        gravity: 9.80665,
        aerodynamics: .baseline
    )

    #expect(ReferenceQuadrotorParameters.baseline == validated)
}

@Test func referenceQuadrotorActuatorEngineBuildsDefaultMotorMaxThrustsWithoutThrowing() throws {
    let parameters = ReferenceQuadrotorParameters.baseline
    let state = try ReferenceQuadrotorState(
        position: SIMD3<Double>(0, 0, 0),
        velocity: SIMD3<Double>(0, 0, 0),
        orientation: simd_quatd(angle: 0, axis: SIMD3<Double>(0, 0, 1)),
        angularVelocity: SIMD3<Double>(0, 0, 0)
    )
    let store = ReferenceQuadrotorWorldStore(state: state, motorThrusts: .zero)
    let engine = ReferenceQuadrotorActuatorEngine(
        parameters: parameters,
        store: store,
        timeStep: try TimeStep(delta: 0.001)
    )

    let expected = try MotorMaxThrusts.uniform(parameters.maxThrust)
    #expect(engine.motorMaxThrusts == expected)
}

@Test func builtInPhysicsTypesRejectInvalidDecodedPayloads() {
    #expect(throws: MotorThrusts.ValidationError.negative("f1")) {
        _ = try decode(MotorThrusts.self, #"{"f1":-1,"f2":0,"f3":0,"f4":0}"#)
    }

    #expect(throws: MotorMaxThrusts.ValidationError.negative("f1")) {
        _ = try decode(MotorMaxThrusts.self, #"{"f1":-1,"f2":0,"f3":0,"f4":0}"#)
    }

    #expect(throws: IMU6NoiseConfig.ValidationError.negative("gyroNoiseStdDev")) {
        _ = try decode(
            IMU6NoiseConfig.self,
            """
            {"gyroNoiseStdDev":-1,"gyroBias":0,"gyroRandomWalkSigma":0,"accelNoiseStdDev":0,"accelBias":0,"accelRandomWalkSigma":0,"delaySteps":0}
            """
        )
    }

    #expect(throws: AerodynamicsParameters.ValidationError.negative("dragCoefficient")) {
        _ = try decode(
            AerodynamicsParameters.self,
            """
            {"dragCoefficient":-1,"referenceArea":0,"liftCoefficient":0,"bodyVolume":0,"angularDrag":{"x":0,"y":0,"z":0}}
            """
        )
    }

    #expect(throws: ReferenceQuadrotorParameters.ValidationError.nonPositive("maxThrust")) {
        _ = try decode(
            ReferenceQuadrotorParameters.self,
            """
            {"mass":1,"inertia":{"x":0.005,"y":0.005,"z":0.009},"armLength":0.12,"motorTimeConstant":0.03,"maxThrust":-1,"yawCoefficient":0.02,"gravity":9.80665,"aerodynamics":{"dragCoefficient":0,"referenceArea":0,"liftCoefficient":0,"bodyVolume":0,"angularDrag":{"x":0,"y":0,"z":0}}}
            """
        )
    }
}

private func decode<T: Decodable>(_ type: T.Type, _ json: String) throws -> T {
    try JSONDecoder().decode(T.self, from: Data(json.utf8))
}
