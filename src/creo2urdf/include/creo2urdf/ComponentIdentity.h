#ifndef CREO2URDF_COMPONENT_IDENTITY_H
#define CREO2URDF_COMPONENT_IDENTITY_H

#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

// IDs of component features, relative to the export's root assembly.
using ComponentId = std::vector<int>;

// Compare model definitions, not the addresses of Object Toolkit wrappers.
// This only validates a path's root/leaf; occurrence identity remains the ID path.
template<class ModelHandle>
bool sameComponentModel(const ModelHandle& left, const ModelHandle& right) {
    return left && right && left->GetType() == right->GetType() &&
           std::string(left->GetFullName()) == std::string(right->GetFullName());
}

inline std::string componentIdString(const ComponentId& id) {
    std::string result;
    for (int value : id) {
        if (!result.empty()) result += "_";
        result += std::to_string(value);
    }
    return result;
}

inline std::map<ComponentId, std::string> resolveCadComponentNames(
    const std::map<ComponentId, std::string>& models) {
    std::map<std::string, size_t> counts, candidateCounts;
    for (const auto& model : models) ++counts[model.second];
    std::map<ComponentId, std::string> result;
    std::set<std::string> reserved;
    for (const auto& model : models) {
        auto name = model.second;
        if (counts.at(name) > 1) name += "_" + componentIdString(model.first);
        result.emplace(model.first, name);
        ++candidateCounts[name];
        reserved.insert(name);
        // Also avoid stealing any explicit path-qualified selector.
        reserved.insert(model.second + "_" + componentIdString(model.first));
    }
    for (auto& entry : result) {
        if (candidateCounts.at(entry.second) == 1) continue;
        // Preserve actual unique CAD names; disambiguate generated names only.
        if (counts.at(models.at(entry.first)) == 1) continue;
        auto name = entry.second + "_occ_" + componentIdString(entry.first);
        while (!reserved.insert(name).second) name += "_";
        entry.second = name;
    }
    return result;
}

inline std::map<ComponentId, std::string> resolveComponentNames(
    const std::map<ComponentId, std::string>& models,
    const std::map<ComponentId, std::string>& aliases,
    const std::map<std::string, std::string>& legacyRename) {
    std::map<std::string, size_t> counts;
    for (const auto& model : models) ++counts[model.second];
    const auto cadNames = resolveCadComponentNames(models);
    for (const auto& alias : aliases) {
        if (!models.count(alias.first) || alias.second.empty())
            throw std::runtime_error("Invalid componentNames entry: " + componentIdString(alias.first));
    }
    // A qualified key can also be an actual CAD model name. Never let one
    // rename entry silently select two different occurrences.
    std::map<std::string, std::set<ComponentId>> selectors;
    for (const auto& model : models) {
        selectors[model.second + "_" + componentIdString(model.first)].insert(model.first);
        if (counts.at(model.second) == 1) selectors[model.second].insert(model.first);
        selectors[cadNames.at(model.first)].insert(model.first);
    }
    for (const auto& selector : selectors) {
        if (selector.second.size() > 1 && legacyRename.count(selector.first))
            throw std::runtime_error("Ambiguous occurrence rename key: " + selector.first + "; use componentNames with explicit paths");
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
        if (alias != aliases.end()) name = alias->second;
        else if (occurrenceRename != legacyRename.end()) name = occurrenceRename->second;
        else if (cadRename != legacyRename.end()) name = cadRename->second;
        else if (counts.at(model.second) == 1)
            name = rename == legacyRename.end() ? model.second : rename->second;
        else {
            if (rename != legacyRename.end())
                throw std::runtime_error("Ambiguous rename for " + model.second + "; use occurrence keys such as " + qualifiedName + " in rename, or componentNames");
            name = cadNames.at(model.first);
        }
        if (name.empty() || !used.insert(name).second)
            throw std::runtime_error("Duplicate or empty exported link name: " + name);
        result.emplace(model.first, name);
    }
    return result;
}
inline std::string cadJointName(const ComponentId& parent, const ComponentId& child,
                               const std::map<ComponentId, std::string>& cadNames) {
    return cadNames.at(parent) + "--" + cadNames.at(child);
}
#endif
