import Foundation
import Security
let service = "ugrp.typesafe.ai"
let account = "jev"
let query: [String: Any] = [kSecClass as String: kSecClassGenericPassword, kSecAttrService as String: service, kSecAttrAccount as String: account]
if CommandLine.arguments.count > 1 && CommandLine.arguments[1] == "store" {
    let data = FileHandle.standardInput.readDataToEndOfFile()
    guard !data.isEmpty else { exit(2) }
    var attributes = query
    attributes[kSecValueData as String] = data
    var status = SecItemAdd(attributes as CFDictionary, nil)
    if status == errSecDuplicateItem { status = SecItemUpdate(query as CFDictionary, [kSecValueData as String: data] as CFDictionary) }
    guard status == errSecSuccess else { fputs("keychain status \(status)\n", stderr); exit(1) }
    var check = query
    check[kSecReturnData as String] = true
    var value: CFTypeRef?
    guard SecItemCopyMatching(check as CFDictionary, &value) == errSecSuccess, let actual = value as? Data, actual == data else { exit(3) }
    print("Stored and read-back verified in macOS Keychain")
} else {
    var check = query
    check[kSecReturnData as String] = true
    var value: CFTypeRef?
    let status = SecItemCopyMatching(check as CFDictionary, &value)
    guard status == errSecSuccess, let data = value as? Data else { fputs("keychain read status \(status)\n", stderr); exit(1) }
    FileHandle.standardOutput.write(data)
}
