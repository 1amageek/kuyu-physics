// swift-tools-version: 6.2

import PackageDescription

let package = Package(
    name: "kuyu-physics",
    platforms: [
        .macOS(.v26)
    ],
    products: [
        .library(
            name: "KuyuPhysics",
            targets: ["KuyuPhysics"]
        ),
    ],
    dependencies: [
        .package(path: "../kuyu-core"),
    ],
    targets: [
        .target(
            name: "KuyuPhysics",
            dependencies: [
                .product(name: "KuyuCore", package: "kuyu-core"),
            ]
        ),
        .testTarget(
            name: "KuyuPhysicsTests",
            dependencies: ["KuyuPhysics"]
        ),
    ]
)
