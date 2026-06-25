import KuyuCore

public struct AerodynamicsParameters: Sendable, Equatable, Codable {
    public enum ValidationError: Error, Equatable {
        case nonFinite(String)
        case negative(String)
    }

    public let dragCoefficient: Double
    public let referenceArea: Double
    public let liftCoefficient: Double
    public let bodyVolume: Double
    public let angularDrag: Axis3

    public init(
        dragCoefficient: Double,
        referenceArea: Double,
        liftCoefficient: Double,
        bodyVolume: Double,
        angularDrag: Axis3
    ) throws {
        guard dragCoefficient.isFinite else { throw ValidationError.nonFinite("dragCoefficient") }
        guard referenceArea.isFinite else { throw ValidationError.nonFinite("referenceArea") }
        guard liftCoefficient.isFinite else { throw ValidationError.nonFinite("liftCoefficient") }
        guard bodyVolume.isFinite else { throw ValidationError.nonFinite("bodyVolume") }
        guard angularDrag.x.isFinite,
              angularDrag.y.isFinite,
              angularDrag.z.isFinite else {
            throw ValidationError.nonFinite("angularDrag")
        }

        guard dragCoefficient >= 0 else { throw ValidationError.negative("dragCoefficient") }
        guard referenceArea >= 0 else { throw ValidationError.negative("referenceArea") }
        guard liftCoefficient >= 0 else { throw ValidationError.negative("liftCoefficient") }
        guard bodyVolume >= 0 else { throw ValidationError.negative("bodyVolume") }
        guard angularDrag.x >= 0,
              angularDrag.y >= 0,
              angularDrag.z >= 0 else {
            throw ValidationError.negative("angularDrag")
        }

        self.dragCoefficient = dragCoefficient
        self.referenceArea = referenceArea
        self.liftCoefficient = liftCoefficient
        self.bodyVolume = bodyVolume
        self.angularDrag = angularDrag
    }

    public static let zero = AerodynamicsParameters(
        uncheckedDragCoefficient: 0,
        uncheckedReferenceArea: 0,
        uncheckedLiftCoefficient: 0,
        uncheckedBodyVolume: 0,
        uncheckedAngularDrag: Axis3(x: 0, y: 0, z: 0)
    )

    public static let baseline = AerodynamicsParameters(
        uncheckedDragCoefficient: 1.1,
        uncheckedReferenceArea: 0.05,
        uncheckedLiftCoefficient: 0.2,
        uncheckedBodyVolume: 0.003,
        uncheckedAngularDrag: Axis3(x: 0.02, y: 0.02, z: 0.04)
    )

    private enum CodingKeys: String, CodingKey {
        case dragCoefficient
        case referenceArea
        case liftCoefficient
        case bodyVolume
        case angularDrag
    }

    public init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        try self.init(
            dragCoefficient: try container.decode(Double.self, forKey: .dragCoefficient),
            referenceArea: try container.decode(Double.self, forKey: .referenceArea),
            liftCoefficient: try container.decode(Double.self, forKey: .liftCoefficient),
            bodyVolume: try container.decode(Double.self, forKey: .bodyVolume),
            angularDrag: try container.decode(Axis3.self, forKey: .angularDrag)
        )
    }

    private init(
        uncheckedDragCoefficient dragCoefficient: Double,
        uncheckedReferenceArea referenceArea: Double,
        uncheckedLiftCoefficient liftCoefficient: Double,
        uncheckedBodyVolume bodyVolume: Double,
        uncheckedAngularDrag angularDrag: Axis3
    ) {
        self.dragCoefficient = dragCoefficient
        self.referenceArea = referenceArea
        self.liftCoefficient = liftCoefficient
        self.bodyVolume = bodyVolume
        self.angularDrag = angularDrag
    }
}
