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
 * @param models Map from occurrence paths to original CAD model names.
 * @return A unique CAD name for every occurrence, used by the inventory and joint keys.
 *
 * Unique model names are preserved. Repeated models receive an underscore and
 * their full path. Colliding generated names additionally receive "_occ_<path>"
 * and, if necessary, trailing underscores. Reserved names include the original
 * candidates and explicit path-qualified selectors. Assignment is deterministic
 * for a given model map because occurrences are visited in path order.
 */
inline std::map<ComponentId, std::string> resolveCadComponentNames(const std::map<ComponentId, std::string>& models) {
    std::map<std::string, size_t> counts, candidateCounts;
    for (const auto& model : models) {
        ++counts[model.second];
    }
    std::map<ComponentId, std::string> result;
    std::set<std::string> reserved;
    for (const auto& model : models) {
        auto name = model.second;
        if (counts.at(name) > 1) {
            name += "_" + componentIdString(model.first);
        }
        result.emplace(model.first, name);
        ++candidateCounts[name];
        reserved.insert(name);
        // Also avoid stealing any explicit path-qualified selector.
        reserved.insert(model.second + "_" + componentIdString(model.first));
    }
    for (auto& entry : result) {
        if (candidateCounts.at(entry.second) == 1) {
            continue;
        }
        // Preserve actual unique CAD names; disambiguate generated names only.
        if (counts.at(models.at(entry.first)) == 1) {
            continue;
        }
        auto name = entry.second + "_occ_" + componentIdString(entry.first);
        while (!reserved.insert(name).second) {
            name += "_";
        }
        entry.second = name;
    }
    return result;
}

/**
 * @brief Resolve and validate the final URDF link name of each occurrence.
 * @param models Map from occurrence paths to original CAD model names.
 * @param aliases Explicit path-to-name assignments from YAML componentNames.
 * @param legacyRename YAML rename map, accepting path-qualified names, resolved
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
inline std::map<ComponentId, std::string> resolveComponentNames(const std::map<ComponentId, std::string>& models, const std::map<ComponentId, std::string>& aliases, const std::map<std::string, std::string>& legacyRename) {
    std::map<std::string, size_t> counts;
    for (const auto& model : models) {
        ++counts[model.second];
    }
    const auto cadNames = resolveCadComponentNames(models);
    for (const auto& alias : aliases) {
        if (!models.count(alias.first) || alias.second.empty()) {
            throw std::runtime_error("Invalid componentNames entry: " + componentIdString(alias.first));
        }
    }
    // A qualified key can also be an actual CAD model name. Never let one
    // rename entry silently select two different occurrences.
    std::map<std::string, std::set<ComponentId>> selectors;
    for (const auto& model : models) {
        selectors[model.second + "_" + componentIdString(model.first)].insert(model.first);
        if (counts.at(model.second) == 1) {
            selectors[model.second].insert(model.first);
        }
        selectors[cadNames.at(model.first)].insert(model.first);
    }
    for (const auto& selector : selectors) {
        if (selector.second.size() > 1 && legacyRename.count(selector.first)) {
            throw std::runtime_error("Ambiguous occurrence rename key: " + selector.first + "; use componentNames with explicit paths");
        }
    }
    std::map<ComponentId, std::string> result;
    std::set<std::string> used;
    for (const auto& model : models) {
        const auto alias = aliases.find(model.first);
        const auto rename = legacyRename.find(model.second);
        const auto qualifiedName = model.second + "_" + componentIdString(model.first);
        const auto occurrenceRename = legacyRename.find(qualifiedName);
        const auto cadRename = legacyRename.find(cadNames.at(model.first));
        std::string name;
        if (alias != aliases.end()) {
            name = alias->second;
        } else if (occurrenceRename != legacyRename.end()) {
            name = occurrenceRename->second;
        } else if (cadRename != legacyRename.end()) {
            name = cadRename->second;
        } else if (counts.at(model.second) == 1) {
            name = rename == legacyRename.end() ? model.second : rename->second;
        } else {
            if (rename != legacyRename.end()) {
                throw std::runtime_error("Ambiguous rename for " + model.second + "; use occurrence keys such as " + qualifiedName + " in rename, or componentNames");
            }
            name = cadNames.at(model.first);
        }
        if (name.empty() || !used.insert(name).second) {
            throw std::runtime_error("Duplicate or empty exported link name: " + name);
        }
        result.emplace(model.first, name);
    }
    return result;
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
