import Testing
@testable import KuyuPhysics
import KuyuCore

@Test func referenceQuadrotorParametersExist() {
    let params = ReferenceQuadrotorParameters.nominal
    #expect(params.mass > 0)
    #expect(params.armLength > 0)
}
