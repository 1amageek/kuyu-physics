import Foundation
import KuyuCore

public struct URDFKinematicModel: Sendable, Equatable {
    public let links: [URDFLink]
    public let joints: [URDFJoint]

    public init(links: [URDFLink], joints: [URDFJoint]) {
        self.links = links
        self.joints = joints
    }

    public var rootLinkNames: [String] {
        let childLinks = Set(joints.map(\.child))
        return links.map(\.name).filter { !childLinks.contains($0) }
    }
}

public struct URDFLink: Sendable, Equatable {
    public let name: String
    public let visuals: [URDFVisual]

    public init(name: String, visuals: [URDFVisual]) {
        self.name = name
        self.visuals = visuals
    }
}

public struct URDFVisual: Sendable, Equatable {
    public let origin: URDFPose
    public let geometry: URDFGeometry

    public init(origin: URDFPose, geometry: URDFGeometry) {
        self.origin = origin
        self.geometry = geometry
    }
}

public struct URDFJoint: Sendable, Equatable {
    public let name: String
    public let type: URDFJointType
    public let parent: String
    public let child: String
    public let origin: URDFPose
    public let axis: Axis3

    public init(
        name: String,
        type: URDFJointType,
        parent: String,
        child: String,
        origin: URDFPose,
        axis: Axis3
    ) {
        self.name = name
        self.type = type
        self.parent = parent
        self.child = child
        self.origin = origin
        self.axis = axis
    }
}

public enum URDFJointType: String, Sendable, Equatable {
    case continuous
    case fixed
    case floating
    case planar
    case prismatic
    case revolute
}

public struct URDFPose: Sendable, Equatable {
    public let xyz: Axis3
    public let rpy: Axis3

    public init(
        xyz: Axis3 = Axis3(x: 0, y: 0, z: 0),
        rpy: Axis3 = Axis3(x: 0, y: 0, z: 0)
    ) {
        self.xyz = xyz
        self.rpy = rpy
    }
}

public enum URDFGeometry: Sendable, Equatable {
    case box(size: Axis3)
    case cylinder(radius: Double, length: Double)
    case mesh(filename: String, scale: Axis3?)
    case sphere(radius: Double)
}

public struct URDFKinematicParser: Sendable {
    public enum ParserError: Error, Equatable {
        case readFailed(String)
        case parseFailed
        case invalidAttribute(String)
        case incompleteLink(String)
        case incompleteJoint(String)
    }

    public init() {}

    public func parse(url: URL) throws -> URDFKinematicModel {
        let data: Data
        do {
            data = try Data(contentsOf: url)
        } catch {
            throw ParserError.readFailed("\(error)")
        }

        let delegate = URDFKinematicXMLParser()
        let parser = XMLParser(data: data)
        parser.delegate = delegate
        guard parser.parse() else {
            throw ParserError.parseFailed
        }
        return try delegate.model()
    }
}

private final class URDFKinematicXMLParser: NSObject, XMLParserDelegate {
    private struct LinkDraft {
        let name: String
        var visuals: [URDFVisual] = []
    }

    private struct VisualDraft {
        var origin = URDFPose()
        var geometry: URDFGeometry?
    }

    private struct JointDraft {
        let name: String
        let type: URDFJointType
        var parent: String?
        var child: String?
        var origin = URDFPose()
        var axis = Axis3(x: 1, y: 0, z: 0)
    }

    private var links: [URDFLink] = []
    private var joints: [URDFJoint] = []
    private var currentLink: LinkDraft?
    private var currentVisual: VisualDraft?
    private var currentJoint: JointDraft?
    private var isInsideGeometry = false
    private var parseError: URDFKinematicParser.ParserError?

    func model() throws -> URDFKinematicModel {
        if let parseError {
            throw parseError
        }
        return URDFKinematicModel(links: links, joints: joints)
    }

    func parser(
        _ parser: XMLParser,
        didStartElement elementName: String,
        namespaceURI: String?,
        qualifiedName qName: String?,
        attributes attributeDict: [String: String] = [:]
    ) {
        guard parseError == nil else { return }

        do {
            switch elementName {
            case "link":
                let name = try required(attributeDict["name"], field: "link.name")
                currentLink = LinkDraft(name: name)
            case "visual":
                currentVisual = VisualDraft()
            case "geometry":
                isInsideGeometry = true
            case "origin":
                let pose = try parsePose(attributeDict)
                if currentVisual != nil {
                    currentVisual?.origin = pose
                } else if currentJoint != nil {
                    currentJoint?.origin = pose
                }
            case "box":
                if isInsideGeometry, currentVisual != nil {
                    let size = try parseAxis3(attributeDict["size"], field: "box.size")
                    currentVisual?.geometry = .box(size: size)
                }
            case "cylinder":
                if isInsideGeometry, currentVisual != nil {
                    let radius = try parseDouble(attributeDict["radius"], field: "cylinder.radius")
                    let length = try parseDouble(attributeDict["length"], field: "cylinder.length")
                    currentVisual?.geometry = .cylinder(radius: radius, length: length)
                }
            case "sphere":
                if isInsideGeometry, currentVisual != nil {
                    let radius = try parseDouble(attributeDict["radius"], field: "sphere.radius")
                    currentVisual?.geometry = .sphere(radius: radius)
                }
            case "mesh":
                if isInsideGeometry, currentVisual != nil {
                    let filename = try required(attributeDict["filename"], field: "mesh.filename")
                    let scale = try attributeDict["scale"].map { try parseAxis3($0, field: "mesh.scale") }
                    currentVisual?.geometry = .mesh(filename: filename, scale: scale)
                }
            case "joint":
                let name = try required(attributeDict["name"], field: "joint.name")
                let rawType = try required(attributeDict["type"], field: "joint.type")
                guard let type = URDFJointType(rawValue: rawType) else {
                    throw URDFKinematicParser.ParserError.invalidAttribute("joint.type")
                }
                currentJoint = JointDraft(name: name, type: type)
            case "parent":
                if currentJoint != nil {
                    currentJoint?.parent = try required(attributeDict["link"], field: "joint.parent.link")
                }
            case "child":
                if currentJoint != nil {
                    currentJoint?.child = try required(attributeDict["link"], field: "joint.child.link")
                }
            case "axis":
                if currentJoint != nil {
                    currentJoint?.axis = try parseAxis3(attributeDict["xyz"], field: "joint.axis.xyz")
                }
            default:
                break
            }
        } catch let error as URDFKinematicParser.ParserError {
            parseError = error
            parser.abortParsing()
        } catch {
            parseError = .parseFailed
            parser.abortParsing()
        }
    }

    func parser(
        _ parser: XMLParser,
        didEndElement elementName: String,
        namespaceURI: String?,
        qualifiedName qName: String?
    ) {
        guard parseError == nil else { return }

        switch elementName {
        case "geometry":
            isInsideGeometry = false
        case "visual":
            guard let visual = currentVisual else { return }
            if let geometry = visual.geometry {
                currentLink?.visuals.append(URDFVisual(origin: visual.origin, geometry: geometry))
            }
            currentVisual = nil
        case "link":
            guard let link = currentLink else { return }
            links.append(URDFLink(name: link.name, visuals: link.visuals))
            currentLink = nil
        case "joint":
            guard let draft = currentJoint,
                  let parent = draft.parent,
                  let child = draft.child else {
                parseError = .incompleteJoint(currentJoint?.name ?? "unknown")
                parser.abortParsing()
                return
            }
            joints.append(URDFJoint(
                name: draft.name,
                type: draft.type,
                parent: parent,
                child: child,
                origin: draft.origin,
                axis: draft.axis
            ))
            currentJoint = nil
        default:
            break
        }
    }

    private func required(_ value: String?, field: String) throws -> String {
        guard let value, !value.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty else {
            throw URDFKinematicParser.ParserError.invalidAttribute(field)
        }
        return value
    }

    private func parsePose(_ attributes: [String: String]) throws -> URDFPose {
        let xyz = try attributes["xyz"].map { try parseAxis3($0, field: "origin.xyz") }
            ?? Axis3(x: 0, y: 0, z: 0)
        let rpy = try attributes["rpy"].map { try parseAxis3($0, field: "origin.rpy") }
            ?? Axis3(x: 0, y: 0, z: 0)
        return URDFPose(xyz: xyz, rpy: rpy)
    }

    private func parseAxis3(_ value: String?, field: String) throws -> Axis3 {
        guard let value else {
            throw URDFKinematicParser.ParserError.invalidAttribute(field)
        }
        let parts = value.split(whereSeparator: { $0 == " " || $0 == "\t" || $0 == "\n" })
        guard parts.count == 3,
              let x = Double(parts[0]),
              let y = Double(parts[1]),
              let z = Double(parts[2]) else {
            throw URDFKinematicParser.ParserError.invalidAttribute(field)
        }
        return Axis3(x: x, y: y, z: z)
    }

    private func parseDouble(_ value: String?, field: String) throws -> Double {
        guard let value, let parsed = Double(value) else {
            throw URDFKinematicParser.ParserError.invalidAttribute(field)
        }
        return parsed
    }
}
