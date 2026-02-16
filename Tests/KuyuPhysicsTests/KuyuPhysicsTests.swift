import Testing
@testable import KuyuPhysics
import KuyuCore

@Test func referenceQuadrotorParametersExist() {
    let params = ReferenceQuadrotorParameters.baseline
    #expect(params.mass > 0)
    #expect(params.armLength > 0)
}
