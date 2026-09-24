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

inline std::map<ComponentId, std::string> resolveComponentNames(
    const std::map<ComponentId, std::string>& models,
    const std::map<ComponentId, std::string>& aliases,
    const std::map<std::string, std::string>& legacyRename) {
    std::map<std::string, size_t> counts;
    for (const auto& model : models) ++counts[model.second];
    for (const auto& alias : aliases) {
        if (!models.count(alias.first) || alias.second.empty())
            throw std::runtime_error("Invalid componentNames entry: " + componentIdString(alias.first));
    }
    std::map<ComponentId, std::string> result;
    std::set<std::string> used;
    for (const auto& model : models) {
        const auto alias = aliases.find(model.first);
        const auto rename = legacyRename.find(model.second);
        std::string name;
        if (alias != aliases.end()) name = alias->second;
        else if (counts.at(model.second) == 1)
            name = rename == legacyRename.end() ? model.second : rename->second;
        else {
            if (rename != legacyRename.end())
                throw std::runtime_error("Ambiguous rename for " + model.second + "; use componentNames for each occurrence");
            name = model.second + "__" + componentIdString(model.first);
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
