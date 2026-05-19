/** @file ElementTreeManager.h
 *  @brief Contains declarations for the ElementTreeManager class.
 * 
 * This file contains the declarations for the ElementTreeManager class,
 * which is used to extract the ElementTree object of an assembly part, to
 * get information about the joints and datums between parts.
 * 
 *  @bug No known bugs.
 * 
 * @copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <creo2urdf/Utils.h>

#include <wfcFeature.h>
#include <wfcElemIds.h>

#include <ProAsmcomp.h>


/**
 * @brief Mapping from ProAsmcompSetType to JointType.
 *
 * This map associates ProAsmcompSetType constants with their corresponding JointType values.
 */
static const std::map<ProAsmcompSetType, JointType> proAsmCompSetType_to_JointType
{
    {PRO_ASM_SET_TYPE_PIN, JointType::Revolute},              ///< Mapping for Pin joint type (1 rotational DOF).
    {PRO_ASM_SET_TYPE_SLIDER, JointType::Linear},             ///< Mapping for Slider joint type (1 translational DOF).
    {PRO_ASM_SET_TYPE_BALL, JointType::Spherical},            ///< Mapping for Ball joint type (3 rotational DOFs).
    {PRO_ASM_SET_TYPE_FIXED, JointType::Fixed},               ///< Mapping for Fixed joint type (zero DOFs).
    {PRO_ASM_SET_TYPE_WELD, JointType::Fixed},                ///< Mapping for Weld joint type (assumed as Fixed).
    {PRO_ASM_SET_USER_DEFINED_TYPE, JointType::Fixed}         ///< Mapping for User-Defined joint type (assumed as Fixed).
};

/**
 * @brief The ElementTreeManager class extracts the ElementTree of a part and provides methods to extract information from it.
 */
class ElementTreeManager {
public:
    /**
     * @brief Default constructor for ElementTreeManager.
     */
    ElementTreeManager();

    /**
     * @brief Constructor for ElementTreeManager that extracts the element tree from the part and builds the joint info map.
     * @param[in] feat A pointer to a part casted as feature.
     * @param[out] joint_info_map A map containing joint information.
     */
    ElementTreeManager(pfcFeature_ptr feat, JointInfo& joint_info);

    /**
     * @brief Destructor for ElementTreeManager.
     */
    ~ElementTreeManager();

    /**
     * @brief Populates joint information from the given ElementTree.
     * @param[in] feat A pointer to a part casted as feature.
     * @param[out] joint_info_map A map containing joint information.
     * @return True if successful, false otherwise.
     */
    bool populateJointInfoFromElementTree(pfcFeature_ptr feat, JointInfo& joint_info);

    /**
     * @brief Gets the constraint type between two assembled parts.
     * @return The constraint type.
     */
    int getConstraintType();

    /**
     * @brief Gets the name of the parent element of the two assembled parts.
     * @return The name of the parent element.
     */
    std::string getParentName();

    /**
     * @brief Gets the name of the child  of the two assembled parts.
     * @return The name of the child element.
     */
    std::string getChildName();

private:
    wfcElementTree_ptr tree{ nullptr }; ///< Pointer to the ElementTree of the part as feature.
    wfcWFeature_ptr wfeat{ nullptr };   ///< Pointer to the part as feature.
    pfcSolid_ptr parent_solid{ nullptr };         ///< Pointer to the parent solid.
    pfcSolid_ptr child_solid{ nullptr };          ///< Pointer to the child solid.
    std::string parent_link_key{ "" };           ///< Unique key of the parent link occurrence.
    std::string child_link_key{ "" };            ///< Unique key of the child link occurrence.

    /*
     * @brief Retrieves the name of a common datum for the given model item type.
     * @param type The model item type.
     * @return The name of the common datum.
     */
    /*std::string retrieveCommonDatumName(pfcModelItemType type);*/

    /**
     * @brief Gets the characterizing constraint datum used for assembling the two parts, depending on the joint.
     * For a revolute joint, the datum is Axis, for a fixed joint the datum is CSys.
     * 
     * @param feat A pointer to a part casted as feature.
     * @param constraint_type The constraint type.
     * @param datum_type The datum type.
     * @return The constraint datum.
     */
    std::string getConstraintDatum(pfcFeature_ptr feat, pfcComponentConstraintType constraint_type, pfcModelItemType datum_type);

    /**
     * @brief Retrieves references to the parent and child solids. We assume there are only two parts and they are named differently.
     * @return True if successful, false otherwise.
     */
    bool retrieveSolidReferences();

    /**
     * @brief Retrieves the name of the part associated with the ElementTree.
     * @return The name of the part.
     */
    std::string retrievePartName();

    /**
     * @brief Retrieves the min and max limits for the joint created during assembling the parts.
     * @param feat A pointer to a part casted as feature.
     * @return A pair representing the limits (min, max).
     */
    std::pair<double, double> retrieveLimits(pfcFeature_ptr feat);

    // SEEMS to work but we need to investigate further, using this may allow to refactor the code deeply

    pfcTransform3D_ptr retrieveTransform(pfcFeature_ptr feat);
};
