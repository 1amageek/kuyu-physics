public struct KuyuVector3: Sendable, Codable, Equatable {
    public let x: Double
    public let y: Double
    public let z: Double

    public init(x: Double, y: Double, z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }

    public init(from decoder: Decoder) throws {
        var container = try decoder.unkeyedContainer()
        let x = try container.decode(Double.self)
        let y = try container.decode(Double.self)
        let z = try container.decode(Double.self)
        if !container.isAtEnd {
            throw DecodingError.dataCorruptedError(
                in: container,
                debugDescription: "KuyuVector3 expects exactly 3 elements"
            )
        }
        self.x = x
        self.y = y
        self.z = z
    }

    public func encode(to encoder: Encoder) throws {
        var container = encoder.unkeyedContainer()
        try container.encode(x)
        try container.encode(y)
        try container.encode(z)
    }
}

public struct KuyuPose: Sendable, Codable, Equatable {
    public let xyz: KuyuVector3
    public let rpy: KuyuVector3

    public init(
        xyz: KuyuVector3 = KuyuVector3(x: 0, y: 0, z: 0),
        rpy: KuyuVector3 = KuyuVector3(x: 0, y: 0, z: 0)
    ) {
        self.xyz = xyz
        self.rpy = rpy
    }
}

public struct KuyuInertiaTensor: Sendable, Codable, Equatable {
    public let ixx: Double
    public let ixy: Double
    public let ixz: Double
    public let iyy: Double
    public let iyz: Double
    public let izz: Double

    public init(
        ixx: Double,
        ixy: Double = 0,
        ixz: Double = 0,
        iyy: Double,
        iyz: Double = 0,
        izz: Double
    ) {
        self.ixx = ixx
        self.ixy = ixy
        self.ixz = ixz
        self.iyy = iyy
        self.iyz = iyz
        self.izz = izz
    }
}
