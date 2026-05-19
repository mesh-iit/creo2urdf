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
#include <pfcAssembly.h>

ElementTreeManager::ElementTreeManager()
{}

ElementTreeManager::ElementTreeManager(pfcFeature_ptr feat, JointInfo& joint_info)
{
    if (!populateJointInfoFromElementTree(feat, joint_info))
    {
        printToMessageWindow("Feature does not support element trees!", c2uLogLevel::WARN);
    }
}

ElementTreeManager::~ElementTreeManager() {}

bool ElementTreeManager::populateJointInfoFromElementTree(pfcFeature_ptr feat, JointInfo& joint_info)
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


    if (!retrieveSolidReferences()) {
        return false;
    }
    joint.child_link_name = getChildName();
    joint.parent_link_name = getParentName();
    joint.child_link_key = child_link_key;
    joint.parent_link_key = parent_link_key;
    joint.base_name = joint.parent_link_name + "--" + joint.child_link_name;
    joint.type = proAsmCompSetType_to_JointType.at(static_cast<ProAsmcompSetType>(getConstraintType()));

    if (joint.type == JointType::Revolute || joint.type == JointType::Linear)
    {
        // We assume that one axis is used to defined the revolute or linear joint
        joint.datum_name = getConstraintDatum(feat, 
            pfcComponentConstraintType::pfcASM_CONSTRAINT_ALIGN,
            pfcModelItemType::pfcITEM_AXIS);
    }
    else if (joint.type == JointType::Fixed || joint.type == JointType::Spherical)
    {
        joint.datum_name = getConstraintDatum(feat,
            pfcComponentConstraintType::pfcASM_CONSTRAINT_CSYS,
            pfcModelItemType::pfcITEM_COORD_SYS);
    }
    else {
        printToMessageWindow("Joint type not supported!", c2uLogLevel::WARN);
        return false;
    }

    joint_info = joint;

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
    auto compfeat = pfcComponentFeat::cast(feat);
    auto constr = compfeat->GetConstraints();

    for (int i = 0; i < constr->getarraysize(); i++)
    {
        auto c = constr->get(i);

        if (c->GetType() == constraint_type &&
            c->GetAssemblyReference()->GetSelItem()->GetType() == datum_type)
        {
            auto s = string(c->GetAssemblyReference()->GetSelItem()->GetName());
            return s;
        }
    }

    return "";
}

std::string ElementTreeManager::getParentName()
{
    if (!tree || !parent_solid)
    {
        printToMessageWindow("Tree or parent solid is null!", c2uLogLevel::WARN);
        return "";
    }
    try{
        return  std::string(parent_solid->GetFullName());
    }
    xcatchbegin
    xcatchcip(defaultEx)
    {
        return "";
    }
    xcatchend
}

std::string ElementTreeManager::getChildName()
{
    if (!tree || !child_solid)
    {
        printToMessageWindow("Tree or child solid is null!", c2uLogLevel::WARN);
        return "";
    }
    try {

        return std::string(child_solid->GetFullName());
    }
    xcatchbegin
    xcatchcip(defaultEx)
    {
        return "";
    }
    xcatchend
}

bool ElementTreeManager::retrieveSolidReferences()
{
    auto parents = wfeat->GetExternalParents(wfcExternalReferenceType::wfcALL_REF_TYPES);

    if (parents == NULL) {

        return false;
    }

    for (int l = 0; l < parents->getarraysize(); l++)
    {
        auto extrefs = parents->get(l)->GetExtRefs();

        if (extrefs == NULL) {
            return false;
        }

        // each element in this array is given by a constraint and
        // the number of parts that compose it
        // e.g. 2 parts x 3 constraints = size(extrefs) = 6
        // We assume there are only two parts and they are named differently
        if (extrefs->getarraysize() == 0)
            return false;

        for (int m = 0; m < extrefs->getarraysize(); m++) {
            auto asm_components = extrefs->get(m)->GetAsmcomponents();
            auto path_to_ref = asm_components->GetPathToRef();
            if (!path_to_ref) {
                continue;
            }
            auto extref = path_to_ref->GetLeaf();
            // While defining a constraint the first part is the parent link and the second part is the child link
            if (extref && !parent_solid) {
                parent_solid = extref;
                parent_link_key = componentPathToKey(path_to_ref->GetComponentIds());
            }
            else if (extref && !child_solid && parent_solid) {
                child_solid = extref;
                child_link_key = componentPathToKey(path_to_ref->GetComponentIds());
            }
            else {
                break;
            }
        }
    }
    return true;
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
