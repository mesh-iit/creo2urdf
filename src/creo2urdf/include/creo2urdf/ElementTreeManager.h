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
     * @brief Destructor for ElementTreeManager.
     */
    ~ElementTreeManager();

    /**
     * @brief Extracts a component feature's element tree and appends its joint information.
     * @param[in] feat Component feature whose placement constraints define the joint.
     * @param[in,out] joint_info_map Joint information indexed by the serialized component
     * feature path; existing entries are preserved and duplicate keys are rejected.
     * @param[in] ownerId Path from the exported root to the assembly owning feat,
     * excluding feat's own ID. An empty path identifies the root assembly.
     * @param[in] contexts Map from root-relative occurrence paths to Creo model handles,
     * including the root at the empty path, subassemblies, parts and excluded skeletons.
     * Used to normalize and validate constraint reference paths.
     * @return True if a joint was inserted; false if the tree type is invalid,
     * no usable reference pair exists, a skeleton is referenced or the joint type is unsupported.
     * @throws std::runtime_error If references are invalid or ambiguous, multiple
     * link pairs or constraint sets are found, or the joint occurrence already exists.
     */
    bool populateJointInfoFromElementTree(pfcFeature_ptr feat, std::map<std::string, JointInfo>& joint_info_map, const ComponentId& ownerId, const std::map<ComponentId, pfcModel_ptr>& contexts);

    /**
     * @brief Gets the constraint type between two assembled parts.
     * @return The constraint type.
     */
    int getConstraintType();

private:
    wfcElementTree_ptr tree{ nullptr }; ///< Pointer to the ElementTree of the part as feature.
    wfcWFeature_ptr wfeat{ nullptr };   ///< Pointer to the part as feature.
    pfcComponentConstraints_ptr constraints{ nullptr }; ///< Constraints in the owning occurrence's context.

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
     * @brief Resolves the constraint selections to parent and child occurrences.
     * @param[in] ownerId Root-relative path to the assembly owning wfeat, excluding
     * the component feature's own ID; empty for the exported root assembly.
     * @param[in] contexts Occurrence-path-to-model map containing the root at the
     * empty path and the collected assembly, part and skeleton handles. Reference
     * roots are matched along the feature's ancestry and leaf models are validated.
     * @return True if parent_id and child_id identify a usable reference pair;
     * false if no usable constraints exist or a reference belongs to a skeleton.
     * @throws std::runtime_error If reference paths cannot be resolved, both endpoints
     * identify the same occurrence or constraints identify different link pairs.
     * @note Stores the constraints in the owning assembly occurrence's context for
     * subsequent datum lookup. Parent and child IDs are valid only on success.
     */
    bool retrieveSolidReferences(const ComponentId& ownerId, const std::map<ComponentId, pfcModel_ptr>& contexts);
    ComponentId parent_id, child_id;

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
