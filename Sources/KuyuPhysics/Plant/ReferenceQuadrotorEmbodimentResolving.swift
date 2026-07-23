public protocol ReferenceQuadrotorEmbodimentResolving: Sendable {
    func resolution(for robot: LoadedKuyuRobot) throws -> ReferenceQuadrotorEmbodimentResolution
}
