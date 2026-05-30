import Foundation

public struct URDFBodyImportResult: Sendable, Equatable {
    public let body: KuyuBodyModel
    public let report: CompatibilityReport

    public init(body: KuyuBodyModel, report: CompatibilityReport) {
        self.body = body
        self.report = report
    }
}

public struct URDFBodyImporter: Sendable {
    public enum ImportError: Error, Equatable {
        case parseFailed(String)
        case missingInertial(String)
        case invalidModel(String)
    }

    public init() {}

    public func importBody(url: URL, bodyID: String, category: String) throws -> URDFBodyImportResult {
        let parser = XMLParser(contentsOf: url)
        guard let parser else {
            throw ImportError.parseFailed("Unable to read \(url.path)")
        }
        let delegate = URDFBodyXMLParser(sourcePath: url.path)
        parser.delegate = delegate
        guard parser.parse() else {
            throw ImportError.parseFailed(parser.parserError?.localizedDescription ?? "unknown")
        }

        let parsed = try delegate.model()
        var mappings: [CompatibilityMapping] = [
            CompatibilityMapping(source: "urdf.robot", target: "KuyuBodyModel", status: .exact)
        ]
        mappings.append(contentsOf: parsed.mappings)
        var links: [LinkDefinition] = []
        links.reserveCapacity(parsed.links.count)
        for link in parsed.links {
            guard let inertial = link.inertial else {
                throw ImportError.missingInertial(link.id)
            }
            links.append(
                LinkDefinition(
                    id: link.id,
                    mass: inertial.mass,
                    centerOfMass: inertial.origin.xyz,
                    inertia: inertial.inertia,
                    visuals: link.visuals,
                    collisions: link.collisions,
                    materialID: nil
                )
            )
            mappings.append(CompatibilityMapping(source: "urdf.link.\(link.id)", target: "body.links.\(link.id)", status: .exact))
        }

        let body = KuyuBodyModel(
            schemaVersion: "1.0",
            bodyID: bodyID,
            name: parsed.name,
            category: category,
            provenance: [
                ModelProvenance(source: url.lastPathComponent, format: "urdf", notes: "Imported from expanded URDF")
            ],
            links: links,
            joints: parsed.joints
        )
        do {
            try body.validate()
        } catch {
            throw ImportError.invalidModel(String(describing: error))
        }

        let report = CompatibilityReport(
            schemaVersion: "1.0",
            reportID: "\(bodyID)-urdf-import",
            sourceFormat: "urdf",
            targetContract: "KuyuBodyModel",
            mappings: mappings,
            readinessLevel: .dynamicSimulation
        )
        return URDFBodyImportResult(body: body, report: report)
    }
}

private struct ParsedURDFBody {
    let name: String
    let links: [ParsedLink]
    let joints: [JointDefinition]
    let mappings: [CompatibilityMapping]
}

private struct ParsedLink {
    let id: String
    var inertial: ParsedInertial?
    var visuals: [GeometryInstance]
    var collisions: [GeometryInstance]
}

private struct ParsedInertial {
    let origin: KuyuPose
    let mass: Double
    let inertia: KuyuInertiaTensor
}

private final class URDFBodyXMLParser: NSObject, XMLParserDelegate {
    private let sourcePath: String
    private var robotName = "robot"
    private var links: [ParsedLink] = []
    private var joints: [JointDefinition] = []
    private var currentLink: ParsedLink?
    private var currentJoint: JointBuilder?
    private var elementStack: [String] = []
    private var currentGeometryKind: GeometryKind?
    private var currentGeometryAttributes: [String: String] = [:]
    private var currentGeometryPose = KuyuPose()
    private var currentGeometryRole: GeometryRole?
    private var currentInertialOrigin = KuyuPose()
    private var currentMass: Double?
    private var currentInertia: KuyuInertiaTensor?
    private var parseError: URDFBodyImporter.ImportError?
    private var compatibilityMappings: [CompatibilityMapping] = []

    init(sourcePath: String) {
        self.sourcePath = sourcePath
    }

    func model() throws -> ParsedURDFBody {
        if let parseError {
            throw parseError
        }
        return ParsedURDFBody(name: robotName, links: links, joints: joints, mappings: compatibilityMappings)
    }

    func parser(
        _ parser: XMLParser,
        didStartElement elementName: String,
        namespaceURI: String?,
        qualifiedName qName: String?,
        attributes attributeDict: [String: String] = [:]
    ) {
        _ = namespaceURI
        _ = qName
        elementStack.append(elementName)
        do {
            try handleStart(elementName, attributes: attributeDict)
        } catch let error as URDFBodyImporter.ImportError {
            parseError = error
            parser.abortParsing()
        } catch {
            parseError = .parseFailed("\(error)")
            parser.abortParsing()
        }
    }

    func parser(
        _ parser: XMLParser,
        didEndElement elementName: String,
        namespaceURI: String?,
        qualifiedName qName: String?
    ) {
        _ = parser
        _ = namespaceURI
        _ = qName
        do {
            try handleEnd(elementName)
        } catch let error as URDFBodyImporter.ImportError {
            parseError = error
        } catch {
            parseError = .parseFailed("\(error)")
        }
        _ = elementStack.popLast()
    }

    private func handleStart(_ elementName: String, attributes: [String: String]) throws {
        switch elementName {
        case "robot":
            robotName = attributes["name"] ?? robotName
        case "link":
            guard let name = attributes["name"] else {
                throw URDFBodyImporter.ImportError.parseFailed("link.name")
            }
            currentLink = ParsedLink(id: name, inertial: nil, visuals: [], collisions: [])
        case "inertial":
            currentInertialOrigin = KuyuPose()
            currentMass = nil
            currentInertia = nil
        case "visual":
            currentGeometryRole = .visual
            currentGeometryPose = KuyuPose()
        case "collision":
            currentGeometryRole = .collision
            currentGeometryPose = KuyuPose()
        case "origin":
            let pose = try parsePose(attributes)
            if elementStack.contains("inertial") {
                currentInertialOrigin = pose
            } else if currentGeometryRole != nil {
                currentGeometryPose = pose
            } else if currentJoint != nil {
                currentJoint?.origin = pose
            }
        case "mass":
            if elementStack.contains("inertial") {
                currentMass = try parseRequiredDouble(attributes["value"], field: "mass.value")
            }
        case "inertia":
            if elementStack.contains("inertial") {
                currentInertia = try KuyuInertiaTensor(
                    ixx: parseRequiredDouble(attributes["ixx"], field: "inertia.ixx"),
                    ixy: parseRequiredDouble(attributes["ixy"], field: "inertia.ixy"),
                    ixz: parseRequiredDouble(attributes["ixz"], field: "inertia.ixz"),
                    iyy: parseRequiredDouble(attributes["iyy"], field: "inertia.iyy"),
                    iyz: parseRequiredDouble(attributes["iyz"], field: "inertia.iyz"),
                    izz: parseRequiredDouble(attributes["izz"], field: "inertia.izz")
                )
            }
        case "box", "cylinder", "sphere", "mesh":
            currentGeometryKind = GeometryKind(rawValue: elementName)
            currentGeometryAttributes = attributes
        case "joint":
            guard let name = attributes["name"], let rawKind = attributes["type"] else {
                throw URDFBodyImporter.ImportError.parseFailed("joint")
            }
            currentJoint = JointBuilder(id: name, kind: try jointKind(rawKind))
        case "parent":
            currentJoint?.parentLinkID = attributes["link"]
        case "child":
            currentJoint?.childLinkID = attributes["link"]
        case "axis":
            currentJoint?.axis = try parseVector(attributes["xyz"] ?? "", field: "axis.xyz")
            currentJoint?.hasAxis = true
        case "limit":
            currentJoint?.lowerLimit = try parseOptionalDouble(attributes["lower"], field: "limit.lower")
            currentJoint?.upperLimit = try parseOptionalDouble(attributes["upper"], field: "limit.upper")
            currentJoint?.effortLimit = try parseOptionalDouble(attributes["effort"], field: "limit.effort")
            currentJoint?.velocityLimit = try parseOptionalDouble(attributes["velocity"], field: "limit.velocity")
        case "dynamics":
            currentJoint?.hasDynamics = true
            currentJoint?.damping = try parseOptionalDouble(attributes["damping"], field: "dynamics.damping") ?? 0
            currentJoint?.coulombFriction = try parseOptionalDouble(attributes["friction"], field: "dynamics.friction") ?? 0
        default:
            break
        }
    }

    private func handleEnd(_ elementName: String) throws {
        switch elementName {
        case "link":
            if let currentLink {
                links.append(currentLink)
            }
            currentLink = nil
        case "inertial":
            guard let mass = currentMass, let inertia = currentInertia else {
                throw URDFBodyImporter.ImportError.parseFailed("inertial")
            }
            currentLink?.inertial = ParsedInertial(origin: currentInertialOrigin, mass: mass, inertia: inertia)
        case "visual", "collision":
            if let geometry = try finishGeometry(role: currentGeometryRole) {
                switch currentGeometryRole {
                case .visual:
                    currentLink?.visuals.append(geometry)
                case .collision:
                    currentLink?.collisions.append(geometry)
                case nil:
                    break
                }
            }
            currentGeometryRole = nil
            currentGeometryKind = nil
            currentGeometryAttributes = [:]
        case "joint":
            guard let builder = currentJoint, let joint = try currentJoint?.build() else {
                throw URDFBodyImporter.ImportError.parseFailed("joint")
            }
            if !builder.hasDynamics {
                compatibilityMappings.append(CompatibilityMapping(
                    source: "urdf.joint.\(builder.id).dynamics",
                    target: "body.joints.\(builder.id).damping/friction",
                    status: .supplemented,
                    notes: "URDF dynamics were absent; imported damping and Coulomb friction as explicit zero values."
                ))
            }
            joints.append(joint)
            currentJoint = nil
        default:
            break
        }
    }

    private func finishGeometry(role: GeometryRole?) throws -> GeometryInstance? {
        guard let kind = currentGeometryKind, let role else { return nil }
        let id = "\(currentLink?.id ?? "link")-\(role.rawValue)-\(currentLinkGeometryCount(role: role))"
        switch kind {
        case .box:
            let size = try parseVector(currentGeometryAttributes["size"] ?? "", field: "box.size")
            return GeometryInstance(id: id, kind: .box, pose: currentGeometryPose, size: size)
        case .cylinder:
            return GeometryInstance(
                id: id,
                kind: .cylinder,
                pose: currentGeometryPose,
                radius: try parseRequiredDouble(currentGeometryAttributes["radius"], field: "cylinder.radius"),
                length: try parseRequiredDouble(currentGeometryAttributes["length"], field: "cylinder.length")
            )
        case .sphere:
            return GeometryInstance(
                id: id,
                kind: .sphere,
                pose: currentGeometryPose,
                radius: try parseRequiredDouble(currentGeometryAttributes["radius"], field: "sphere.radius")
            )
        case .mesh:
            return GeometryInstance(
                id: id,
                kind: .mesh,
                pose: currentGeometryPose,
                meshPath: currentGeometryAttributes["filename"],
                meshFormat: meshFormat(currentGeometryAttributes["filename"]),
                scale: try currentGeometryAttributes["scale"].map { try parseVector($0, field: "mesh.scale") }
            )
        }
    }

    private func currentLinkGeometryCount(role: GeometryRole) -> Int {
        switch role {
        case .visual:
            return currentLink?.visuals.count ?? 0
        case .collision:
            return currentLink?.collisions.count ?? 0
        }
    }

    private func parsePose(_ attributes: [String: String]) throws -> KuyuPose {
        KuyuPose(
            xyz: try parseVector(attributes["xyz"] ?? "0 0 0", field: "origin.xyz"),
            rpy: try parseVector(attributes["rpy"] ?? "0 0 0", field: "origin.rpy")
        )
    }

    private func parseVector(_ raw: String, field: String) throws -> KuyuVector3 {
        let parts = raw.split(separator: " ").map(String.init)
        guard parts.count == 3,
              let x = Double(parts[0]),
              let y = Double(parts[1]),
              let z = Double(parts[2]) else {
            throw URDFBodyImporter.ImportError.parseFailed(field)
        }
        return KuyuVector3(x: x, y: y, z: z)
    }

    private func parseRequiredDouble(_ raw: String?, field: String) throws -> Double {
        guard let raw, let value = Double(raw), value.isFinite else {
            throw URDFBodyImporter.ImportError.parseFailed(field)
        }
        return value
    }

    private func parseOptionalDouble(_ raw: String?, field: String) throws -> Double? {
        guard let raw else { return nil }
        guard let value = Double(raw), value.isFinite else {
            throw URDFBodyImporter.ImportError.parseFailed(field)
        }
        return value
    }

    private func jointKind(_ raw: String) throws -> JointKind {
        switch raw {
        case "revolute":
            return .revolute
        case "fixed":
            return .fixed
        case "continuous":
            return .continuous
        case "prismatic":
            return .prismatic
        default:
            throw URDFBodyImporter.ImportError.parseFailed("joint.type.\(raw)")
        }
    }

    private func meshFormat(_ path: String?) -> RenderMeshFormat? {
        guard let path else { return nil }
        let ext = URL(fileURLWithPath: path).pathExtension.lowercased()
        return RenderMeshFormat(rawValue: ext)
    }

    private enum GeometryRole: String {
        case visual
        case collision
    }

    private struct JointBuilder {
        let id: String
        let kind: JointKind
        var parentLinkID: String?
        var childLinkID: String?
        var origin = KuyuPose()
        var axis = KuyuVector3(x: 0, y: 0, z: 0)
        var hasAxis = false
        var hasDynamics = false
        var lowerLimit: Double?
        var upperLimit: Double?
        var effortLimit: Double?
        var velocityLimit: Double?
        var damping: Double = 0
        var coulombFriction: Double = 0

        func build() throws -> JointDefinition {
            guard let parentLinkID, let childLinkID else {
                throw URDFBodyImporter.ImportError.parseFailed("joint.links")
            }
            if kind != .fixed && !hasAxis {
                throw URDFBodyImporter.ImportError.parseFailed("joint.axis.\(id)")
            }
            return JointDefinition(
                id: id,
                kind: kind,
                parentLinkID: parentLinkID,
                childLinkID: childLinkID,
                origin: origin,
                axis: axis,
                lowerLimit: lowerLimit,
                upperLimit: upperLimit,
                effortLimit: effortLimit,
                velocityLimit: velocityLimit,
                damping: damping,
                coulombFriction: coulombFriction
            )
        }
    }
}
