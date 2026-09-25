/**
 * @file ElementTreeManager.cpp
 * @brief Contains definitions for the ElementTreeManager class.
 * 
 * @copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 * 
 */

#include <creo2urdf/ElementTreeManager.h>

ElementTreeManager::ElementTreeManager()
{}

ElementTreeManager::~ElementTreeManager() {}

bool ElementTreeManager::populateJointInfoFromElementTree(pfcFeature_ptr feat, std::map<std::string, JointInfo>& joint_info_map, const ComponentId& ownerId, const std::map<ComponentId, pfcModel_ptr>& contexts)
{
    wfeat = wfcWFeature::cast(feat);

    try
    {
        tree = wfeat->GetElementTree(nullptr, wfcFEAT_EXTRACT_NO_OPTS);
    }
    xcatchbegin
    xcatchcip(pfcXToolkitInvalidType)
    {
        return false;
    }
    xcatchend
    
    JointInfo joint;


    if (!retrieveSolidReferences(ownerId, contexts)) {
        return false;
    }
    joint.child_link_id = child_id;
    joint.parent_link_id = parent_id;
    auto featureId = ownerId;
    featureId.push_back(feat->GetId());
    std::string joint_name = componentIdString(featureId);
    const auto type = proAsmCompSetType_to_JointType.find(static_cast<ProAsmcompSetType>(getConstraintType()));
    if (type == proAsmCompSetType_to_JointType.end()) return false;
    joint.type = type->second;

    if (joint.type == JointType::Revolute || joint.type == JointType::Linear)
    {
        // We assume that one axis is used to defined the revolute or linear joint
        joint.datum_name = getConstraintDatum(feat, 
            pfcComponentConstraintType::pfcASM_CONSTRAINT_ALIGN,
            pfcModelItemType::pfcITEM_AXIS);
    }
    else if (joint.type == JointType::Fixed)
    {
        joint.datum_name = getConstraintDatum(feat,
            pfcComponentConstraintType::pfcASM_CONSTRAINT_CSYS,
            pfcModelItemType::pfcITEM_COORD_SYS);
    }
    else if (joint.type == JointType::Spherical)
    {
        joint.datum_name = getConstraintDatum(feat,
            pfcComponentConstraintType::pfcASM_CONSTRAINT_ALIGN,
            pfcModelItemType::pfcITEM_POINT);
    }
    else
    {
        printToMessageWindow("Joint type not supported!", c2uLogLevel::WARN);
        return false;
    }

    if (!joint_info_map.emplace(joint_name, joint).second)
        throw std::runtime_error("Duplicate joint occurrence: " + joint_name);

    return true;
}

int ElementTreeManager::getConstraintType()
{
    if (tree == nullptr)
    {
        return -1;
    }

    wfcElemPathItems_ptr elemItems = wfcElemPathItems::create();
    wfcElemPathItem_ptr Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_SETS);
    elemItems->append(Item);
    auto sets = tree->GetElement(wfcElementPath::Create(elemItems));
    if (!sets) return -1;
    auto children = sets->GetChildren();
    if (children && children->getarraysize() > 1)
        throw std::runtime_error("Multiple constraint sets per component are not supported");
    Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_SET);
    elemItems->append(Item);
    Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_SET_TYPE);
    elemItems->append(Item);

    try {
        return tree->GetElement(wfcElementPath::Create(elemItems))->GetValue()->GetIntValue();
    }
    xcatchbegin
    xcatchcip(pfcXBadGetArgValue)
    {
        printToMessageWindow("Invalid constraint data type in element tree!", c2uLogLevel::WARN);
        return -1;
    }
    xcatchend
}

string ElementTreeManager::getConstraintDatum(pfcFeature_ptr feat, pfcComponentConstraintType constraint_type, pfcModelItemType datum_type)
{
    auto constr = constraints;

    if (!constr) return "";
    for (int i = 0; i < constr->getarraysize(); i++)
    {
        auto c = constr->get(i);
        if (!c) continue;

        if (c->GetType() == constraint_type && c->GetAssemblyReference() && c->GetAssemblyReference()->GetSelItem() &&
            c->GetAssemblyReference()->GetSelItem()->GetType() == datum_type)
        {
            auto s = string(c->GetAssemblyReference()->GetSelItem()->GetName());
            return s;
        }
    }

    return "";
}

bool ElementTreeManager::retrieveSolidReferences(const ComponentId& ownerId,
    const std::map<ComponentId, pfcModel_ptr>& contexts)
{
    // A reference path is relative to its own root. Resolve that root only
    // along this feature's occurrence ancestry, never by a global model name.
    auto componentId = ownerId;
    componentId.push_back(wfeat->GetId());
    std::string failure;
    auto normalize = [&](pfcSelection_ptr selection, const ComponentId& fallback, ComponentId& result) {
        const auto reject = [&](const std::string& reason) { failure = reason; return false; };
        if (!selection || !selection->GetSelItem()) return reject("missing selected item");
        const pfcModel_ptr selectedModel = pfcModel::cast(selection->GetSelItem()->GetDBParent());
        if (!selectedModel) return reject("selected item has no model owner");
        const auto selectedName = std::string(selectedModel->GetFullName());
        auto path = selection->GetPath();
        if (!path || !path->GetRoot()) {
            if (path && path->GetComponentIds() && path->GetComponentIds()->getarraysize() != 0)
                return reject("path has IDs but no root; selected model=" + selectedName);
            const auto context = contexts.find(fallback);
            if (context == contexts.end() || !sameComponentModel(context->second, selectedModel))
                return reject("local reference owner mismatch; selected model=" + selectedName +
                              "; expected occurrence=[" + componentIdString(fallback) + "]");
            result = fallback;
            return true;
        }
        const pfcModel_ptr rootModel = pfcModel::cast(path->GetRoot());
        auto prefix = componentId;
        bool found = false;
        for (;;) {
            const auto context = contexts.find(prefix);
            if (context != contexts.end() && sameComponentModel(context->second, rootModel)) {
                if (found) return reject("ambiguous path root in occurrence ancestry");
                result = prefix;
                found = true;
            }
            if (prefix.empty()) break;
            prefix.pop_back();
        }
        if (!found) return reject("path root is outside occurrence ancestry; root=" +
                                  std::string(rootModel->GetFullName()) + "; selected model=" + selectedName);
        auto ids = path->GetComponentIds();
        if (ids) for (int i = 0; i < ids->getarraysize(); ++i) result.push_back(ids->get(i));
        const auto leaf = contexts.find(result);
        if (leaf == contexts.end()) return reject("occurrence not in inventory: [" + componentIdString(result) +
                                                  "]; selected model=" + selectedName);
        if (!sameComponentModel(leaf->second, selectedModel))
            return reject("path leaf mismatch at [" + componentIdString(result) + "]; inventory model=" +
                          std::string(leaf->second->GetFullName()) + "; selected model=" + selectedName);
        return true;
    };

    // Creo expects the path to the assembly owning the feature, not to the
    // feature itself. This preserves external references in nested assemblies.
    auto ownerIds = xintsequence::create();
    for (int id : ownerId) ownerIds->append(id);
    auto ownerPath = pfcCreateComponentPath(pfcAssembly::cast(contexts.at(ComponentId{})), ownerIds);
    constraints = nullptr;
    try {
        constraints = pfcComponentFeat::cast(wfeat)->GetConstraintsWithCompPath(ownerPath);
    }
    xcatchbegin
    xcatchcip(pfcXToolkitNotFound)
    {
        // Packaged components have no placement constraints (including a base).
        return false;
    }
    xcatchend
    if (!constraints) return false;
    bool found = false;
    for (int i = 0; i < constraints->getarraysize(); ++i) {
        auto constraint = constraints->get(i);
        if (!constraint) continue;
        auto parent = constraint->GetAssemblyReference();
        auto child = constraint->GetComponentReference();
        if (!parent || !child) continue;
        ComponentId p, c;
        const auto errorPrefix = "Cannot resolve joint references at component " + componentIdString(componentId) +
                                 ", constraint " + std::to_string(i);
        if (!normalize(parent, ownerId, p))
            throw std::runtime_error(errorPrefix + " (assembly reference): " + failure);
        if (!normalize(child, componentId, c))
            throw std::runtime_error(errorPrefix + " (component reference): " + failure);
        // A valid reference to an intentionally excluded solid is not an
        // invalid occurrence. No URDF joint can be created for this feature.
        if (pfcSolid::cast(contexts.at(p))->GetIsSkeleton() ||
            pfcSolid::cast(contexts.at(c))->GetIsSkeleton()) return false;
        if (p == c) throw std::runtime_error("Joint references the same occurrence twice: " + componentIdString(p));
        if (found && (p != parent_id || c != child_id))
            throw std::runtime_error("Multiple link pairs/constraint sets are unsupported at component " + componentIdString(componentId));
        parent_id = p;
        child_id = c;
        found = true;
    }
    return found;
}

std::string ElementTreeManager::retrievePartName()
{
    if (tree == nullptr)
    {
        return "";
    }

    wfcElemPathItems_ptr elemItems = wfcElemPathItems::create();
    elemItems->append(wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_MODEL));

    // The element tree shows the child name, aka the current part as 
    // <PRO_E_COMPONENT_MODEL type = "pointer" application = "model"> <name_of_part>.prt < / PRO_E_COMPONENT_MODEL>
    // and it cannot be retrieved with GetValue() because it throws an exception
    try {
        wfcElement_ptr element = tree->GetElement(wfcElementPath::Create(elemItems));
        return std::string(element->GetSpecialValueElem()->GetComponentModel()->GetFullName());
    }
    xcatchbegin
    xcatchcip(defaultEx)
    {
        return "";
    }
    xcatchend
}


pfcTransform3D_ptr ElementTreeManager::retrieveTransform(pfcFeature_ptr feat) {
    pfcTransform3D_ptr parentCsys_H_childCsys = nullptr;
    
    wfcElemPathItems_ptr elemItems = wfcElemPathItems::create();
    wfcElemPathItem_ptr Item;
    wfcElement_ptr element;
    
    Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_INIT_POS);
    elemItems->append(Item);

    wfcElementPath_ptr transform_path = wfcElementPath::Create(elemItems);
    element = tree->GetElement(transform_path);

    auto value_ptr = element->GetValue();
    if(!value_ptr)
        printToMessageWindow("wfcPRO_E_COMPONENT_INIT_POS value is null");
    else {
        parentCsys_H_childCsys = value_ptr->GetTransformValue();
        // Because the transform is from child to parent, we need to invert it
        parentCsys_H_childCsys->Invert();
    }
    
    return parentCsys_H_childCsys;
}


std::pair<double, double> ElementTreeManager::retrieveLimits(pfcFeature_ptr feat)
{
    wfcElemPathItems_ptr elemItems = wfcElemPathItems::create();
    wfcElemPathItem_ptr Item;
    wfcElement_ptr element;

    Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_SETS);
    elemItems->append(Item);
    Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_SET);
    elemItems->append(Item);
    Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_JAS_SETS);
    elemItems->append(Item);
    Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_JAS_SET);
    elemItems->append(Item);
    Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_JAS_MAX_LIMIT);
    elemItems->append(Item);
    Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_JAS_MAX_LIMIT_VAL);
    elemItems->append(Item);

    wfcElementPath_ptr limitpath = wfcElementPath::Create(elemItems);
    element = tree->GetElement(limitpath);

    double max = element->GetValue()->GetDoubleValue();

     elemItems->clear();

     Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_SETS);
     elemItems->append(Item);
     Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_SET);
     elemItems->append(Item);
     Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_JAS_SETS);
     elemItems->append(Item);
     Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_JAS_SET);
     elemItems->append(Item);
     Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_JAS_MIN_LIMIT);
     elemItems->append(Item);
     Item = wfcElemPathItem::Create(wfcELEM_PATH_ITEM_TYPE_ID, wfcPRO_E_COMPONENT_JAS_MIN_LIMIT_VAL);
     elemItems->append(Item);

     limitpath = wfcElementPath::Create(elemItems);
     element = tree->GetElement(limitpath);

     double min = element->GetValue()->GetDoubleValue();

    return std::pair<double, double>(min, max);
}
