#ifndef CREO2URDF_COMPONENT_IDENTITY_H
#define CREO2URDF_COMPONENT_IDENTITY_H

#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

/// Ordered component feature IDs from the exported root assembly to an occurrence.
using ComponentId = std::vector<int>;

/**
 * @brief Compare model definitions independently of Object Toolkit wrapper addresses.
 * @param left First model handle, providing GetType() and GetFullName().
 * @param right Second model handle with the same interface.
 * @return True if both handles are non-null and their model types and full names match.
 * @note This validates a path's root or leaf, not occurrence identity: repeated
 * instances of the same model are distinguished by their ComponentId paths.
 */
template<class ModelHandle>
bool sameComponentModel(const ModelHandle& left, const ModelHandle& right) {
    return left && right && left->GetType() == right->GetType() &&
           std::string(left->GetFullName()) == std::string(right->GetFullName());
}

/**
 * @brief Serialize an occurrence path for use in names and diagnostics.
 * @param id Ordered component feature IDs relative to the exported root assembly.
 * @return IDs separated by underscores (for example, {40, 12} becomes "40_12");
 * an empty path produces an empty string.
 */
inline std::string componentIdString(const ComponentId& id) {
    std::string result;
    for (int value : id) {
        if (!result.empty()) {
            result += "_";
        }
        result += std::to_string(value);
    }
    return result;
}

/**
 * @brief Build CAD occurrence names independently of URDF aliases and renames.
 * @param compID_cadName_map Map from occurrence paths to original CAD model names.
 * @return A unique CAD name for every occurrence, used by the inventory and joint keys.
 *
 * Unique model names are preserved. Repeated models receive an underscore and
 * their full path. Colliding generated names additionally receive "_occ_<path>"
 * and, if necessary, trailing underscores. Reserved names include the original
 * candidates and explicit path-qualified selectors. Assignment is deterministic
 * for a given model map because occurrences are visited in path order.
 */
inline std::map<ComponentId, std::string> resolveCadComponentNames(const std::map<ComponentId, std::string>& compID_cadName_map) {
    // Count original CAD model names to identify models used by multiple occurrences.
    std::map<std::string, size_t> cadName_occurrenceCount_map, candidateName_occurrenceCount_map;
    for (const auto& componentModelEntry : compID_cadName_map) {
        ++cadName_occurrenceCount_map[componentModelEntry.second];
    }
    // Build candidate names: preserve unique model names and append the full path
    // to repeated ones. Reserve all candidates before resolving any collisions.
    std::map<ComponentId, std::string> compID_cadOccurrenceName_map;
    std::set<std::string> reservedNames_set;
    for (const auto& componentModelEntry : compID_cadName_map) {
        auto candidateCadName = componentModelEntry.second;
        if (cadName_occurrenceCount_map.at(candidateCadName) > 1) {
            candidateCadName += "_" + componentIdString(componentModelEntry.first);
        }
        compID_cadOccurrenceName_map.emplace(componentModelEntry.first, candidateCadName);
        ++candidateName_occurrenceCount_map[candidateCadName];
        reservedNames_set.insert(candidateCadName);
        // Also avoid stealing any explicit path-qualified selector.
        reservedNames_set.insert(componentModelEntry.second + "_" + componentIdString(componentModelEntry.first));
    }
    // Resolve collisions between generated candidates and existing CAD names,
    // including collisions between two generated candidates from nested paths.
    for (auto& componentCadNameEntry : compID_cadOccurrenceName_map) {
        if (candidateName_occurrenceCount_map.at(componentCadNameEntry.second) == 1) {
            continue;
        }
        // Preserve actual unique CAD names; disambiguate generated names only.
        if (cadName_occurrenceCount_map.at(compID_cadName_map.at(componentCadNameEntry.first)) == 1) {
            continue;
        }
        auto candidateCadName = componentCadNameEntry.second + "_occ_" + componentIdString(componentCadNameEntry.first);
        // Insertion succeeds only for an unused name and reserves it immediately.
        while (!reservedNames_set.insert(candidateCadName).second) {
            candidateCadName += "_";
        }
        componentCadNameEntry.second = candidateCadName;
    }
    return compID_cadOccurrenceName_map;
}

/**
 * @brief Resolve and validate the final URDF link name of each occurrence.
 * @param compID_cadName_map Map from occurrence paths to original CAD model names.
 * @param compID_URDFCompNames_map Explicit path-to-name assignments from YAML componentNames.
 * @param renameKey_URDFName_map YAML rename map, accepting path-qualified names, resolved
 * CAD occurrence names and unique original CAD names. Unrelated keys are ignored.
 * @return Occurrence-to-URDF-name map with nonempty, unique output names.
 * @throws std::runtime_error If an alias has an unknown path or empty name, a
 * rename selector is ambiguous, or final link names are empty or duplicated.
 *
 * Explicit aliases take precedence over path-qualified renames, followed by
 * resolved CAD-name renames and unique original CAD-name renames. Otherwise the
 * CAD occurrence name is used. A bare rename for a repeated model is rejected
 * when an occurrence has no more specific assignment. Ambiguous selectors are
 * rejected before applying aliases, even if all occurrences have aliases.
 */
inline std::map<ComponentId, std::string> resolveComponentNames(const std::map<ComponentId, std::string>& compID_cadName_map, const std::map<ComponentId, std::string>& compID_URDFCompNames_map, const std::map<std::string, std::string>& renameKey_URDFName_map) {
    // Prepare model multiplicities and default CAD occurrence names independently
    // of the user's URDF naming choices.
    std::map<std::string, size_t> cadName_occurrenceCount_map;
    for (const auto& componentModelEntry : compID_cadName_map) {
        ++cadName_occurrenceCount_map[componentModelEntry.second];
    }
    const auto compID_cadOccurrenceName_map = resolveCadComponentNames(compID_cadName_map);

    // Validate explicit componentNames assignments before resolving rename keys.
    for (const auto& componentAlias : compID_URDFCompNames_map) {
        if (!compID_cadName_map.count(componentAlias.first) || componentAlias.second.empty()) {
            throw std::runtime_error("Invalid componentNames entry: " + componentIdString(componentAlias.first));
        }
    }
    // Map every supported textual selector to the occurrences it can identify.
    // A set avoids counting the same occurrence twice when selector forms coincide.
    std::map<std::string, std::set<ComponentId>> renameKey_compIDs_map;
    for (const auto& componentModelEntry : compID_cadName_map) {
        renameKey_compIDs_map[componentModelEntry.second + "_" + componentIdString(componentModelEntry.first)].insert(componentModelEntry.first);
        if (cadName_occurrenceCount_map.at(componentModelEntry.second) == 1) {
            renameKey_compIDs_map[componentModelEntry.second].insert(componentModelEntry.first);
        }
        renameKey_compIDs_map[compID_cadOccurrenceName_map.at(componentModelEntry.first)].insert(componentModelEntry.first);
    }
    // Reject ambiguous keys only when present in rename. This validation runs
    // before alias precedence: an explicit alias does not excuse an ambiguous key.
    for (const auto& renameSelectorEntry : renameKey_compIDs_map) {
        if (renameSelectorEntry.second.size() > 1 && renameKey_URDFName_map.count(renameSelectorEntry.first)) {
            throw std::runtime_error("Ambiguous occurrence rename key: " + renameSelectorEntry.first + "; use componentNames with explicit paths");
        }
    }
    // Assign final URDF names per occurrence, tracking names already assigned
    // so that explicit aliases and renames cannot produce duplicate links.
    std::map<ComponentId, std::string> compID_URDFName_map;
    std::set<std::string> assignedURDFNames_set;
    for (const auto& componentModelEntry : compID_cadName_map) {
        const auto componentAlias = compID_URDFCompNames_map.find(componentModelEntry.first);
        const auto cadModelRename = renameKey_URDFName_map.find(componentModelEntry.second);
        const auto pathQualifiedCadName = componentModelEntry.second + "_" + componentIdString(componentModelEntry.first);
        const auto pathQualifiedRename = renameKey_URDFName_map.find(pathQualifiedCadName);
        const auto cadOccurrenceRename = renameKey_URDFName_map.find(compID_cadOccurrenceName_map.at(componentModelEntry.first));
        // Precedence: componentNames alias, path-qualified rename, resolved CAD
        // occurrence rename, then unique model naming or the automatic default.
        std::string urdfLinkName;
        if (componentAlias != compID_URDFCompNames_map.end()) {
            urdfLinkName = componentAlias->second;
        } else if (pathQualifiedRename != renameKey_URDFName_map.end()) {
            urdfLinkName = pathQualifiedRename->second;
        } else if (cadOccurrenceRename != renameKey_URDFName_map.end()) {
            urdfLinkName = cadOccurrenceRename->second;
        } else if (cadName_occurrenceCount_map.at(componentModelEntry.second) == 1) {
            urdfLinkName = cadModelRename == renameKey_URDFName_map.end() ? componentModelEntry.second : cadModelRename->second;
        } else {
            if (cadModelRename != renameKey_URDFName_map.end()) {
                throw std::runtime_error("Ambiguous rename for " + componentModelEntry.second + "; use occurrence keys such as " + pathQualifiedCadName + " in rename, or componentNames");
            }
            urdfLinkName = compID_cadOccurrenceName_map.at(componentModelEntry.first);
        }
        // Reject invalid final names rather than silently altering user assignments.
        if (urdfLinkName.empty() || !assignedURDFNames_set.insert(urdfLinkName).second) {
            throw std::runtime_error("Duplicate or empty exported link name: " + urdfLinkName);
        }
        compID_URDFName_map.emplace(componentModelEntry.first, urdfLinkName);
    }
    return compID_URDFName_map;
}

/**
 * @brief Compose the CAD joint key used by YAML rename and as the default joint name.
 * @param parent Parent link occurrence path.
 * @param child Child link occurrence path.
 * @param cadNames Occurrence names produced by resolveCadComponentNames().
 * @return "<parent-cadName>--<child-cadName>", independent of URDF link aliases.
 * @throws std::out_of_range If either occurrence is absent from cadNames.
 */
inline std::string cadJointName(const ComponentId& parent, const ComponentId& child, const std::map<ComponentId, std::string>& cadNames) {
    return cadNames.at(parent) + "--" + cadNames.at(child);
}
#endif
