import Foundation

public struct SDFWorldImportResult: Sendable, Equatable {
    public let world: KuyuWorldModel
    public let report: CompatibilityReport

    public init(world: KuyuWorldModel, report: CompatibilityReport) {
        self.world = world
        self.report = report
    }
}

public struct SDFWorldImporter: Sendable {
    public enum ImportError: Error, Equatable {
        case parseFailed(String)
        case invalidModel(String)
    }

    public init() {}

    public func importWorld(url: URL, worldID: String) throws -> SDFWorldImportResult {
        guard let parser = XMLParser(contentsOf: url) else {
            throw ImportError.parseFailed("Unable to read \(url.path)")
        }
        let delegate = SDFWorldXMLParser()
        parser.delegate = delegate
        guard parser.parse() else {
            throw ImportError.parseFailed(parser.parserError?.localizedDescription ?? "unknown")
        }

        let parsed = try delegate.model()
        guard let timeStepSeconds = parsed.timeStepSeconds else {
            throw ImportError.parseFailed("physics.max_step_size")
        }
        guard let gravity = parsed.gravity else {
            throw ImportError.parseFailed("gravity")
        }
        let world = KuyuWorldModel(
            schemaVersion: "1.0",
            worldID: worldID,
            time: TimeModel(fixedStepSeconds: timeStepSeconds),
            integrator: IntegratorModel(kind: parsed.integrator),
            solver: SolverModel(kind: parsed.contactMode == .disabled ? .disabledContact : .deterministicConstraint, iterations: 20, tolerance: 1e-8),
            gravity: gravity,
            atmosphere: AtmosphereModel(kind: .none),
            wind: WindModel(kind: .none),
            surfaces: parsed.surfaces,
            materials: parsed.materials,
            contact: ContactModel(mode: parsed.contactMode, stiffness: parsed.contactStiffness, damping: parsed.contactDamping),
            nap: NegligibilityApproximationPolicy(
                forceAbsoluteThreshold: 0,
                forceRelativeThreshold: 0,
                torqueAbsoluteThreshold: 0,
                torqueRelativeThreshold: 0
            ),
            randomness: RandomnessModel(seed: 0, deterministicReplay: true)
        )
        do {
            try world.validate()
        } catch {
            throw ImportError.invalidModel(String(describing: error))
        }

        let report = CompatibilityReport(
            schemaVersion: "1.0",
            reportID: "\(worldID)-sdf-import",
            sourceFormat: "sdf",
            targetContract: "KuyuWorldModel",
            mappings: [
                CompatibilityMapping(source: "sdf.world", target: "KuyuWorldModel", status: .exact),
                CompatibilityMapping(source: "sdf.physics", target: "world.time/solver/contact", status: .approximated)
            ] + parsed.mappings,
            readinessLevel: parsed.contactMode == .disabled ? .dynamicSimulation : .contactTraining
        )
        return SDFWorldImportResult(world: world, report: report)
    }
}

private struct ParsedSDFWorld {
    var timeStepSeconds: Double?
    var integrator: IntegratorKind = .semiImplicitEuler
    var gravity: GravityModel?
    var materials: [WorldMaterial] = []
    var surfaces: [WorldSurface] = []
    var contactMode: ContactMode = .disabled
    var contactStiffness: Double?
    var contactDamping: Double?
    var mappings: [CompatibilityMapping] = [
        CompatibilityMapping(
            source: "sdf.contact",
            target: "world.contact",
            status: .ignored,
            notes: "No contact declaration was found; contact remains explicitly disabled."
        )
    ]
}

private final class SDFWorldXMLParser: NSObject, XMLParserDelegate {
    private var parsed = ParsedSDFWorld()
    private var elementStack: [String] = []
    private var textBuffer = ""
    private var parseError: SDFWorldImporter.ImportError?

    func model() throws -> ParsedSDFWorld {
        if let parseError {
            throw parseError
        }
        return parsed
    }

    func parser(
        _ parser: XMLParser,
        didStartElement elementName: String,
        namespaceURI: String?,
        qualifiedName qName: String?,
        attributes attributeDict: [String: String] = [:]
    ) {
        _ = parser
        _ = namespaceURI
        _ = qName
        _ = attributeDict
        elementStack.append(elementName)
        textBuffer = ""
    }

    func parser(_ parser: XMLParser, foundCharacters string: String) {
        _ = parser
        textBuffer += string
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
            try handleEnd(elementName, text: textBuffer.trimmingCharacters(in: .whitespacesAndNewlines))
        } catch let error as SDFWorldImporter.ImportError {
            parseError = error
        } catch {
            parseError = .parseFailed("\(error)")
        }
        _ = elementStack.popLast()
        textBuffer = ""
    }

    private func handleEnd(_ elementName: String, text: String) throws {
        switch elementName {
        case "max_step_size":
            if let value = Double(text), value.isFinite, value > 0 {
                parsed.timeStepSeconds = value
            } else {
                throw SDFWorldImporter.ImportError.parseFailed("max_step_size")
            }
        case "gravity":
            parsed.gravity = GravityModel(kind: .uniform, acceleration: try parseVector(text, field: "gravity"))
        case "static_friction":
            let value = try parseDouble(text, field: "static_friction")
            parsed.materials = [WorldMaterial(id: "sdf-default", staticFriction: value, dynamicFriction: value, restitution: 0)]
        case "dynamic_friction":
            let value = try parseDouble(text, field: "dynamic_friction")
            let existing = parsed.materials.first
            parsed.materials = [
                WorldMaterial(
                    id: "sdf-default",
                    staticFriction: existing?.staticFriction ?? value,
                    dynamicFriction: value,
                    restitution: existing?.restitution ?? 0
                )
            ]
        case "stiffness":
            parsed.contactMode = .penalty
            parsed.contactStiffness = try parseDouble(text, field: "stiffness")
            removeDisabledContactMapping()
        case "damping":
            parsed.contactMode = .penalty
            parsed.contactDamping = try parseDouble(text, field: "damping")
            removeDisabledContactMapping()
        default:
            break
        }
    }

    private func removeDisabledContactMapping() {
        parsed.mappings.removeAll { $0.source == "sdf.contact" && $0.status == .ignored }
    }

    private func parseVector(_ raw: String, field: String) throws -> KuyuVector3 {
        let parts = raw.split(separator: " ").map(String.init)
        guard parts.count == 3,
              let x = Double(parts[0]),
              let y = Double(parts[1]),
              let z = Double(parts[2]) else {
            throw SDFWorldImporter.ImportError.parseFailed(field)
        }
        return KuyuVector3(x: x, y: y, z: z)
    }

    private func parseDouble(_ raw: String, field: String) throws -> Double {
        guard let value = Double(raw), value.isFinite else {
            throw SDFWorldImporter.ImportError.parseFailed(field)
        }
        return value
    }
}
