public struct CanonicalDynamicsProgramValidator: CanonicalDynamicsProgramValidating, Sendable {
    public enum ValidationError: Error, Equatable {
        case invalidProgramID(String)
        case unsupportedSchemaVersion(Int)
        case duplicateLayoutID(String)
        case missingBoundLayout(String)
        case duplicateForceTermID(CanonicalForceTermID)
        case emptyForceTerms
        case unsupportedImplicitTerm(CanonicalForceTermID)
        case duplicateFidelityID(String)
        case emptyFidelities
        case unknownFidelityTerm(fidelityID: String, termID: CanonicalForceTermID)
        case incompleteFidelityPartition(fidelityID: String)
        case disallowedInputLayout(graphID: String, layoutID: String)
        case unsupportedOutputUnit(graphID: String, fieldID: String, unit: String)
        case outputCountMismatch(graphID: String, expected: Int, actual: Int)
        case missingOutput(graphID: String, fieldID: String)
        case outputShapeMismatch(graphID: String, fieldID: String)
        case outputUnitMismatch(graphID: String, fieldID: String)
    }

    private let graphValidator: any CanonicalOperationGraphValidating

    public init(
        graphValidator: any CanonicalOperationGraphValidating = CanonicalOperationGraphValidator()
    ) {
        self.graphValidator = graphValidator
    }

    public func validate(_ content: CanonicalDynamicsProgramContent) throws {
        guard CanonicalIdentifier.isValid(content.id) else {
            throw ValidationError.invalidProgramID(content.id)
        }
        guard content.schemaVersion == CanonicalDynamicsProgram.currentSchemaVersion else {
            throw ValidationError.unsupportedSchemaVersion(content.schemaVersion)
        }

        let layouts = try indexedLayouts(content.layouts)
        for layoutID in content.layoutBindings.all where layouts[layoutID] == nil {
            throw ValidationError.missingBoundLayout(layoutID)
        }

        guard !content.forceTerms.isEmpty else {
            throw ValidationError.emptyForceTerms
        }
        var termIDs = Set<CanonicalForceTermID>()
        let forceLayout = layouts[content.layoutBindings.generalizedForce]!
        let termInputLayouts: Set<String> = [
            content.layoutBindings.state,
            content.layoutBindings.parameters,
            content.layoutBindings.mixer,
            content.layoutBindings.control,
            content.layoutBindings.disturbance,
            content.layoutBindings.environment,
        ]
        for term in content.forceTerms {
            guard termIDs.insert(term.id).inserted else {
                throw ValidationError.duplicateForceTermID(term.id)
            }
            guard term.stiffness == .explicit else {
                throw ValidationError.unsupportedImplicitTerm(term.id)
            }
            try validate(
                graph: term.graph,
                outputLayout: forceLayout,
                allowedInputLayouts: termInputLayouts,
                layouts: content.layouts
            )
        }

        try validate(
            graph: content.derivativeGraph,
            outputLayout: layouts[content.layoutBindings.derivative]!,
            allowedInputLayouts: [
                content.layoutBindings.state,
                content.layoutBindings.parameters,
                content.layoutBindings.generalizedForce,
            ],
            layouts: content.layouts
        )
        try validate(
            graph: content.observableGraph,
            outputLayout: layouts[content.layoutBindings.observables]!,
            allowedInputLayouts: [
                content.layoutBindings.state,
                content.layoutBindings.parameters,
                content.layoutBindings.generalizedForce,
                content.layoutBindings.environment,
            ],
            layouts: content.layouts
        )

        guard !content.fidelities.isEmpty else {
            throw ValidationError.emptyFidelities
        }
        var fidelityIDs = Set<String>()
        for fidelity in content.fidelities {
            guard fidelityIDs.insert(fidelity.id).inserted else {
                throw ValidationError.duplicateFidelityID(fidelity.id)
            }
            let partition = fidelity.active + fidelity.worldModelTargets + fidelity.ignored
            for termID in partition where !termIDs.contains(termID) {
                throw ValidationError.unknownFidelityTerm(
                    fidelityID: fidelity.id,
                    termID: termID
                )
            }
            guard Set(partition) == termIDs else {
                throw ValidationError.incompleteFidelityPartition(fidelityID: fidelity.id)
            }
        }
    }

    private func indexedLayouts(
        _ layouts: [CanonicalBufferLayout]
    ) throws -> [String: CanonicalBufferLayout] {
        var result: [String: CanonicalBufferLayout] = [:]
        for layout in layouts {
            guard result.updateValue(layout, forKey: layout.id) == nil else {
                throw ValidationError.duplicateLayoutID(layout.id)
            }
        }
        return result
    }

    private func validate(
        graph: CanonicalOperationGraph,
        outputLayout: CanonicalBufferLayout,
        allowedInputLayouts: Set<String>,
        layouts: [CanonicalBufferLayout]
    ) throws {
        for input in graph.inputs where !allowedInputLayouts.contains(input.layoutID) {
            throw ValidationError.disallowedInputLayout(
                graphID: graph.id,
                layoutID: input.layoutID
            )
        }
        _ = try graphValidator.signatures(for: graph, layouts: layouts)

        guard graph.outputs.count == outputLayout.fields.count else {
            throw ValidationError.outputCountMismatch(
                graphID: graph.id,
                expected: outputLayout.fields.count,
                actual: graph.outputs.count
            )
        }
        let outputs = Dictionary(uniqueKeysWithValues: graph.outputs.map { ($0.id, $0) })
        for field in outputLayout.fields {
            guard let output = outputs[field.id] else {
                throw ValidationError.missingOutput(graphID: graph.id, fieldID: field.id)
            }
            guard output.shape == field.shape else {
                throw ValidationError.outputShapeMismatch(graphID: graph.id, fieldID: field.id)
            }
            guard let fieldUnit = CanonicalUnit(rawValue: field.unit) else {
                throw ValidationError.unsupportedOutputUnit(
                    graphID: graph.id,
                    fieldID: field.id,
                    unit: field.unit
                )
            }
            guard output.unit.dimension == fieldUnit.dimension else {
                throw ValidationError.outputUnitMismatch(graphID: graph.id, fieldID: field.id)
            }
        }
    }
}
