import Foundation
import KuyuPhysics
import Testing

@Test(.timeLimit(.minutes(1))) func canonicalBufferLayoutDerivesContiguousRanges() throws {
    let layout = try CanonicalBufferLayout(
        id: "reference_quadrotor_state",
        version: 1,
        fields: [
            try CanonicalBufferField(
                id: "position",
                offset: 0,
                shape: .vector3,
                unit: "m",
                role: .state,
                differentiability: .differentiable
            ),
            try CanonicalBufferField(
                id: "orientation",
                offset: 3,
                shape: .quaternion,
                unit: "1",
                role: .state,
                differentiability: .differentiable
            ),
        ]
    )

    #expect(layout.elementCount == 7)
    #expect(layout.range(ofFieldNamed: "position") == 0..<3)
    #expect(layout.range(ofFieldNamed: "orientation") == 3..<7)
    #expect(layout.range(ofFieldNamed: "missing") == nil)
}

@Test(.timeLimit(.minutes(1))) func canonicalBufferLayoutRejectsGapsAndOverlaps() throws {
    let position = try CanonicalBufferField(
        id: "position",
        offset: 0,
        shape: .vector3,
        unit: "m",
        role: .state,
        differentiability: .differentiable
    )
    let velocityWithGap = try CanonicalBufferField(
        id: "velocity",
        offset: 4,
        shape: .vector3,
        unit: "m/s",
        role: .state,
        differentiability: .differentiable
    )

    #expect(
        throws: CanonicalBufferLayout.ValidationError.nonContiguousField(
            fieldID: "velocity",
            expectedOffset: 3,
            actualOffset: 4
        )
    ) {
        _ = try CanonicalBufferLayout(
            id: "reference_quadrotor_state",
            version: 1,
            fields: [position, velocityWithGap]
        )
    }
}

@Test(.timeLimit(.minutes(1))) func canonicalBufferLayoutRejectsDuplicateFieldIDs() throws {
    let first = try CanonicalBufferField(
        id: "position",
        offset: 0,
        shape: .vector3,
        unit: "m",
        role: .state,
        differentiability: .differentiable
    )
    let duplicate = try CanonicalBufferField(
        id: "position",
        offset: 3,
        shape: .vector3,
        unit: "m",
        role: .state,
        differentiability: .differentiable
    )

    #expect(throws: CanonicalBufferLayout.ValidationError.duplicateFieldID("position")) {
        _ = try CanonicalBufferLayout(
            id: "reference_quadrotor_state",
            version: 1,
            fields: [first, duplicate]
        )
    }
}

@Test(.timeLimit(.minutes(1))) func canonicalBufferLayoutDecodingCannotBypassValidation() throws {
    let invalidJSON = Data(
        #"{"id":"reference_quadrotor_state","version":1,"fields":[{"id":"position","offset":1,"shape":{"kind":"vector","length":3},"unit":"m","role":"state","differentiability":"differentiable"}]}"#.utf8
    )

    #expect(
        throws: CanonicalBufferLayout.ValidationError.nonContiguousField(
            fieldID: "position",
            expectedOffset: 0,
            actualOffset: 1
        )
    ) {
        _ = try JSONDecoder().decode(CanonicalBufferLayout.self, from: invalidJSON)
    }
}

@Test(.timeLimit(.minutes(1))) func canonicalValueShapeRejectsAmbiguousVectorLengths() {
    #expect(throws: CanonicalValueShape.ValidationError.invalidVectorLength(1)) {
        _ = try CanonicalValueShape.vector(1)
    }
}

@Test(.timeLimit(.minutes(1))) func canonicalProgramDigestRequiresLowercaseSHA256Form() throws {
    let validValue = String(repeating: "a", count: 64)
    let digest = try CanonicalProgramDigest(validValue)

    #expect(digest.rawValue == validValue)
    #expect(throws: CanonicalProgramDigest.ValidationError.invalidFormat(String(repeating: "A", count: 64))) {
        _ = try CanonicalProgramDigest(String(repeating: "A", count: 64))
    }
    #expect(throws: CanonicalProgramDigest.ValidationError.invalidFormat("abc")) {
        _ = try CanonicalProgramDigest("abc")
    }
}

@Test(.timeLimit(.minutes(1))) func referenceQuadrotorCanonicalLayoutsMatchRuntimeShapes() throws {
    let state = try ReferenceQuadrotorCanonicalLayouts.stateLayout()
    let derivative = try ReferenceQuadrotorCanonicalLayouts.derivativeLayout()
    let parameters = try ReferenceQuadrotorCanonicalLayouts.parameterLayout()
    let mixer = try ReferenceQuadrotorCanonicalLayouts.mixerLayout()
    let controls = try ReferenceQuadrotorCanonicalLayouts.controlLayout()
    let disturbances = try ReferenceQuadrotorCanonicalLayouts.disturbanceLayout()
    let environment = try ReferenceQuadrotorCanonicalLayouts.environmentLayout()
    let generalizedForce = try ReferenceQuadrotorCanonicalLayouts.generalizedForceLayout()
    let observables = try ReferenceQuadrotorCanonicalLayouts.observableLayout()

    #expect(state.elementCount == 13)
    #expect(derivative.elementCount == 13)
    #expect(parameters.elementCount == 16)
    #expect(mixer.elementCount == 7)
    #expect(controls.elementCount == 4)
    #expect(disturbances.elementCount == 6)
    #expect(environment.elementCount == 9)
    #expect(generalizedForce.elementCount == 9)
    #expect(observables.elementCount == 6)
    #expect(state.field(named: "orientation")?.shape == .quaternion)
    #expect(derivative.field(named: "orientation_rate")?.shape == .vector4)
    #expect(environment.field(named: "use_atmosphere")?.differentiability == .nonDifferentiable)
}
