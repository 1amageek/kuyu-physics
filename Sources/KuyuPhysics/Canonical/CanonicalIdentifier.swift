enum CanonicalIdentifier {
    static func isValid(_ value: String) -> Bool {
        guard let first = value.utf8.first, isLowercaseASCII(first) else {
            return false
        }

        return value.utf8.dropFirst().allSatisfy { byte in
            isLowercaseASCII(byte)
                || (48...57).contains(byte)
                || byte == 45
                || byte == 46
                || byte == 95
        }
    }

    private static func isLowercaseASCII(_ byte: UInt8) -> Bool {
        (97...122).contains(byte)
    }
}
