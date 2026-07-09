/** @file Creo2Urdf.h
 *  @brief Contains declarations for the Creo2Urdf class, the main component of the plugin.
 *
 *  @bug No known bugs.
 * 
 * @copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef CREO2URDF_H
#define CREO2URDF_H

#include <creo2urdf/Utils.h>
#include <creo2urdf/Sensorizer.h>
#include <creo2urdf/ElementTreeManager.h>

#include <pfcShrinkwrap.h>
#include <pfcAssembly.h>

#include <iDynTree/ModelIO/ModelExporter.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Traversal.h>

#include <rapidcsv.h>


/**
 * @class Creo2Urdf
 * @brief A class that exports a Creo mechanism assembly to URDF.
 *
 * This class inherits from pfcUICommandActionListener and provides functionality
 * to export a Creo model to URDF format, incorporating options and configurations.
 */
class Creo2Urdf : public pfcUICommandActionListener {
public:
    /**
     * @brief Callback function triggered when the button is clicked.
     * 
     * @details This function is ran when the user clicks on the Creo2Urdf button, and
     * contains the main loop of the plugin. 
     * By reading the kinematic and dynamic information of the assembly, it creates an iDynTree model. 
     * The ModelExporter class of iDynTree is then used to create the URDF file.  
     * The order of operations is the following:
     *  - Prompt the user to select a .yaml file containing the export config
     *  - Prompt the user to select a .csv file containing joint info
     *  - Populates the data members of creo2urdf from the config
     *  - For each element in the assembly
     *      -# Create the elementTree and store the joint info between part and parent
     *      -# Get the mass properties of the current part
     *      -# Instantiate an iDynTree link from the current part
     *      -# Add mesh to the link
     *  - For each element in the joint info map
     *      -# Create a iDynTree joint between parts
     *  - Add the sensors to the iDynTree Model
     *  - Add the exported frames to the links
     *  - Add the export options to the iDynTree model exporter
     *  - Export the iDynTree model to urdf file
     */
    void OnCommand() override;

    Creo2Urdf() = default;
    ~Creo2Urdf() = default;
    Creo2Urdf(const std::string& yaml_path, const std::string& csv_path, const std::string& output_path, pfcModel_ptr asm_model_ptr) : m_yaml_path(yaml_path),
                                                                                                                                       m_csv_path(csv_path),
                                                                                                                                       m_output_path(output_path),
                                                                                                                                       m_root_asm_model_ptr(asm_model_ptr) { }

private:
    /**
     * @brief Export the iDynTree model to URDF format if it is valid.
     * @param mdl The iDynTree model to be exported.
     * @param options The exporter options for configuring the export process.
     * @return True if the export is successful, false otherwise.
     */
    bool exportModelToUrdf(iDynTree::Model mdl, iDynTree::ModelExporterOptions options);

    /**
     * @brief Compute spatial inertia from Creo mass properties. 
     * The mass properties are overridden by the YAML configuration if present in the file.
     * 
     * @param mass_prop The Creo mass properties.
     * @param H The 3D transform matrix to express the center of mass in the link frame.
     * @param link_name The name of the link.
     * @return The computed spatial inertia.
     */
    iDynTree::SpatialInertia computeSpatialInertiafromCreo(pfcMassProperty_ptr mass_prop, iDynTree::Transform H, const std::string& link_name);

    /**
     * @brief Populate the exported frame information map from the Creo model handle.
     * @param modelhdl The Creo model handle.
     */
    void populateExportedFrameInfoMap(LinkInfo& link_info);

    /**
     * @brief Read assigned inertias from the loaded YAML configuration.
     */
    void readAssignedInertiasFromConfig();

    /**
     * @brief Read assigned collision geometry from the loaded YAML configuration.
     */
    void readAssignedCollisionGeometryFromConfig();

    /**
     * @brief Read exported frames from the loaded YAML configuration.
     */
    void readExportedFramesFromConfig();

    /**
     * @brief Creates a mesh file from the Creo model in the form defined in the configuration file.
     * @param component_handle The part as a Creo model.
     * @param mesh_transform The 3D transform associated to the mesh.
     * @return True if successful, false otherwise.
     */
    bool addMeshAndExport(const LinkInfo& link_info);

    /**
     * @brief Load YAML configuration from a file.
     * @param filename The name of the YAML configuration file.
     * @return True if successful, false otherwise.
     */
    bool loadYamlConfig(const std::string& filename);

    bool processAsmItems(pfcModelItems_ptr asmListItems, pfcModel_ptr model_owner, iDynTree::Transform parentAsm_H_csysAsm = iDynTree::Transform::Identity());

    bool setJointParametersFromCsv(const rapidcsv::Document& csv, const std::string& joint_name, 
        iDynTree::IJoint& joint, double conversion_factor);

    /**
     * @brief Get the renamed element from the configuration.
     * @param elem_name The original element name.
     * @return The renamed element name.
     */
    std::string getRenameElementFromConfig(const std::string& elem_name);

    iDynTree::Model idyn_model; /**< The iDynTree model representing the mechanism tree. */
    std::map<std::string, JointInfo> joint_info_map; /**< Map storing information about joints. */
    std::map<std::string, LinkInfo> link_info_map; /**< Map storing information about links. */
    std::map<std::string, ExportedFrameInfo> exported_frame_info_map; /**< Map storing information about exported frames. */
    std::map<std::string, std::array<double,3>> assigned_inertias_map; /**< Map storing assigned inertias. 0 -> xx, 1 -> yy, 2 -> zz. */
    std::map<std::string, CollisionGeometryInfo> assigned_collision_geometry_map; /**< Map storing assigned collision geometries. */
    YAML::Node config; /**< YAML configuration node, storing the content of the configuration file. */
    bool exportAllUseradded{ false }; /**< Flag indicating whether to export all user-added frames. */
    bool exportFirstBaseLinkAdditionalFrameAsFakeURDFBase{ false };  /**< Flag to export the first additional frame attached to the base link as fake urdf base. */
    
    std::array<double, 3> scale{ 1.0, 1.0, 1.0 }; /**< Scale factor for the exported model. Useful for converting between m and mm and viceversa. */
    std::array<double, 3> originXYZ {0.0, 0.0, 0.0}; /**< Offset of the root link in XYZ (meters) wrt the world frame. */
    std::array<double, 3> originRPY {0.0, 0.0, 0.0}; /**< Orientation of the root link in Roll-Pitch-Yaw wrt the world frame. */
    bool warningsAreFatal{ true }; /**< Flag indicating whether warnings are treated as fatal errors. */
    int urdfNumericalPrecision{ -1 }; /**< Number of decimal places for numerical values in the exported URDF. */
    std::string m_yaml_path{ "" }; /**< Path to the YAML configuration file. */
    std::string m_csv_path{ "" }; /**< Path to the CSV file containing joint information. */
    std::string m_output_path{ "" }; /**< Output path for the exported URDF file. */
    pfcModel_ptr m_root_asm_model_ptr{ nullptr }; /**< Handle to the Creo model. */
    pfcSession_ptr m_session_ptr{ nullptr }; /**< Handle to the Creo session. */
    bool m_need_to_move_link_frames_to_be_compatible_with_URDF{ false }; /**< Flag indicating whether to move link frames to be compatible with URDF. */
};

class Creo2UrdfAccess : public pfcUICommandAccessListener {
public:
    pfcCommandAccess OnCommandAccess(xbool AllowErrorMessages) override;
};

#endif // !CREO2URDF_H
