/**
 * @file Creo2Urdf.cpp
 * @brief Contains definitions for the Creo2Urdf class.
 * @copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <creo2urdf/Creo2Urdf.h>
#include <creo2urdf/Utils.h>
#include <pfcExceptions.h>

#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/SphericalJoint.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTransformers.h>

#include <Eigen/Core>

bool Creo2Urdf::processAsmItems(pfcModelItems_ptr asmListItems, pfcModel_ptr model_owner, iDynTree::Transform parentAsm_H_csysAsm, ComponentId ownerId, bool collectOnly) {

    for (int i = 0; i < asmListItems->getarraysize(); i++)
    {
        bool ret{ false };
        auto asmItemAsFeat = pfcFeature::cast(asmListItems->get(i));
        if (asmItemAsFeat->GetFeatType() != pfcFeatureType::pfcFEATTYPE_COMPONENT)
        {
            continue;
        }

        auto component_handle = m_session_ptr->RetrieveModel(pfcComponentFeat::cast(asmItemAsFeat)->GetModelDescr());

        if (component_handle == nullptr) {
            return false;
        }

        if(pfcSolid::cast(component_handle)->GetIsSkeleton())
        {   
            printToMessageWindow(std::string(component_handle->GetFullName()) + " is a skeleton, skipping", c2uLogLevel::INFO);
            continue;
        }

        //printToMessageWindow("Processing " + string(component_handle->GetFullName()) + " Owner " + string(model_owner->GetFullName()));

        auto componentId = ownerId;
        componentId.push_back(asmItemAsFeat->GetId());
        if (collectOnly) {
            component_handles.emplace(componentId, component_handle);
            if (component_handle->GetType() == pfcMDL_ASSEMBLY) {
                if (!processAsmItems(component_handle->ListItems(pfcITEM_FEATURE), component_handle,
                                     iDynTree::Transform::Identity(), componentId, true)) return false;
            } else {
                component_models.emplace(componentId, std::string(component_handle->GetFullName()));
                ++model_counts[std::string(component_handle->GetFullName())];
            }
            continue;
        }
        xintsequence_ptr seq = xintsequence::create();
        seq->append(asmItemAsFeat->GetId());

        ElementTreeManager element_tree_manager;
        element_tree_manager.populateJointInfoFromElementTree(asmItemAsFeat, joint_info_map, ownerId, component_handles);

        pfcComponentPath_ptr comp_path = pfcCreateComponentPath(pfcAssembly::cast(model_owner), seq);

        iDynTree::Transform csysAsm_H_linkFrame = iDynTree::Transform::Identity();
        iDynTree::Transform csysPart_H_link_frame = iDynTree::Transform::Identity();
        iDynTree::Transform parentAsm_H_linkFrame = iDynTree::Transform::Identity();
        
        std::string link_frame_name{ "" };
        auto link_name = string(component_handle->GetFullName());
        std::string urdf_link_name { "" };

        auto type = component_handle->GetType();

        if (type == pfcMDL_ASSEMBLY) {
            iDynTree::Transform root_H_assembly;
            try {
                root_H_assembly = parentAsm_H_csysAsm * fromCreo(comp_path->GetTransform(xtrue), scale);
            }
            xcatchbegin
            xcatchcip(defaultEx)
            {
                throw std::runtime_error("Cannot obtain assembly transform at " + componentIdString(componentId));
            }
            xcatchend
            if (!processAsmItems(component_handle->ListItems(pfcITEM_FEATURE), component_handle,
                                 root_H_assembly, componentId)) return false;
            continue;
        }
        {
            link_frame_name = "";
            urdf_link_name = component_names.at(componentId);
            for (const auto& lf : config["linkFrames"]) {
                if (lf["linkName"].Scalar() != urdf_link_name)
                {
                    continue;
                }
                link_frame_name = lf["frameName"].Scalar();
            }

            if (link_frame_name.empty()) {
                std::tie(ret, link_frame_name) = getFirstCoordinateSystemName(component_handle);
                
                if (!ret) return false;

                printToMessageWindow(link_name + " misses the frame in the linkFrames section, " + link_frame_name + " will be used instead", c2uLogLevel::WARN);
            }
        }
        std::tie(ret, csysAsm_H_linkFrame) = getTransformFromOwnerToLinkFrame(comp_path, component_handle, link_frame_name, scale);

        if (!ret) return false;
        parentAsm_H_linkFrame = parentAsm_H_csysAsm * csysAsm_H_linkFrame;

        auto mass_prop = pfcSolid::cast(component_handle)->GetMassProperty();

        std::tie(ret, csysPart_H_link_frame) = getTransformFromPart(component_handle, link_frame_name, scale);
        if (!ret && warningsAreFatal)
        {
            return false;
        }

        iDynTree::Link link;
        link.setInertia(computeSpatialInertiafromCreo(mass_prop, csysPart_H_link_frame, urdf_link_name));

        if (!link.getInertia().isPhysicallyConsistent())
        {
            printToMessageWindow(link_name + " is NOT physically consistent!", c2uLogLevel::WARN);
            if (warningsAreFatal) {
                return false;
            }
        }

        LinkInfo l_info{ componentId, link_name, urdf_link_name, component_handle, parentAsm_H_linkFrame, csysAsm_H_linkFrame, link_frame_name };
        if (!link_info_map.emplace(componentId, l_info).second) return false;

        if (!idyn_model.addLink(urdf_link_name, link))
            throw std::runtime_error("Failed to add link " + urdf_link_name);
        if (!addMeshAndExport(l_info)) {
            printToMessageWindow("Failed to export mesh for " + link_name, c2uLogLevel::WARN);
            if (warningsAreFatal) {
                return false;
            }
        }
    }
    return true;
}

void Creo2Urdf::OnCommand() {
    try { runExport(); }
    catch (const std::exception& error) {
        printToMessageWindow(std::string("Export aborted: ") + error.what(), c2uLogLevel::WARN);
    }
}

void Creo2Urdf::runExport() {

    // Let's clear the map in case of multiple click
    {
        joint_info_map.clear();
        link_info_map.clear();
        exported_frame_info_map.clear();
        assigned_inertias_map.clear();
        assigned_collision_geometry_map.clear();
    }
    component_models.clear();
    component_names.clear();
    component_handles.clear();
    model_counts.clear();
    mesh_file_names.clear();
    m_need_to_move_link_frames_to_be_compatible_with_URDF = false;
    m_session_ptr = pfcGetProESession();
    if (!m_session_ptr) {
        printToMessageWindow("Failed to get the session", c2uLogLevel::WARN);
        return;
    }
    if (!m_root_asm_model_ptr) {
        m_root_asm_model_ptr = m_session_ptr->GetCurrentModel();
        if (!m_root_asm_model_ptr) {
            printToMessageWindow("Failed to get the current model", c2uLogLevel::WARN);
            return;
        }
    }


    // TODO Principal units probably to be changed from MM to M before getting the model properties
    // pfcSolid_ptr solid_ptr = pfcSolid::cast(m_session_ptr->GetCurrentModel());
    // auto length_unit = solid_ptr->GetPrincipalUnits()->GetUnit(pfcUnitType::pfcUNIT_LENGTH);
    // length_unit->Modify(pfcUnitConversionFactor::Create(0.001), length_unit->GetReferenceUnit()); // IT DOES NOT WORK

    idyn_model = iDynTree::Model::Model(); // no trivial way to clear the model
    auto yaml_file_open_option = pfcFileOpenOptions::Create("*.yml,*.yaml");
    yaml_file_open_option->SetDialogLabel("Select the yaml");

    // YAML file path
    if (m_yaml_path.empty()) {
        m_yaml_path = string(m_session_ptr->UIOpenFile(yaml_file_open_option));
    }
    if (!loadYamlConfig(m_yaml_path))
    {
        printToMessageWindow("Failed to run Creo2Urdf!", c2uLogLevel::WARN);
        return;
    }
    // CSV file path
    if (m_csv_path.empty()) {
        auto csv_file_open_option = pfcFileOpenOptions::Create("*.csv");
        csv_file_open_option->SetDialogLabel("Select the csv");
        m_csv_path = string(m_session_ptr->UIOpenFile(csv_file_open_option));
    }
    rapidcsv::Document joints_csv_table(m_csv_path, rapidcsv::LabelParams(0, 0));
    // Output folder path
    if (m_output_path.empty()) {
        auto output_folder_open_option = pfcDirectorySelectionOptions::Create();
        output_folder_open_option->SetDialogLabel("Select the output dir");
        m_output_path = string(m_session_ptr->UISelectDirectory(output_folder_open_option));
    }
    printToMessageWindow("Output path is: " + m_output_path);
    

    iDynRedirectErrors idyn_redirect;
    idyn_redirect.redirectBuffer(std::cerr.rdbuf(), "iDynTreeErrors.txt");

    bool ret;

    auto asm_component_list = m_root_asm_model_ptr->ListItems(pfcModelItemType::pfcITEM_FEATURE);
    if (asm_component_list->getarraysize() == 0) {
        printToMessageWindow("There are no FEATURES in the asm", c2uLogLevel::WARN);
        return;
    }

    if (config["warningsAreFatal"].IsDefined()) {
        warningsAreFatal = config["warningsAreFatal"].as<bool>();
    }

    if(config["scale"].IsDefined()) {
        if (!config["scale"].IsSequence() || config["scale"].size() < 3) {
            printToMessageWindow("scale must be a sequence of 3 values", c2uLogLevel::WARN);
            return;
        }
        const auto& scale_vec = config["scale"].as<std::vector<double>>();
        scale = { scale_vec[0],
                  scale_vec[1],
                  scale_vec[2] };
    }

    if (config["originXYZ"].IsDefined()) {
        if (!config["originXYZ"].IsSequence() || config["originXYZ"].size() < 3) {
            printToMessageWindow("originXYZ must be a sequence of 3 values", c2uLogLevel::WARN);
            return;
        }
        const auto& xyz_vec = config["originXYZ"].as<std::vector<double>>();
        originXYZ = { xyz_vec[0],
                      xyz_vec[1],
                      xyz_vec[2] };
    }

    if (config["originRPY"].IsDefined()) {
        if (!config["originRPY"].IsSequence() || config["originRPY"].size() < 3) {
            printToMessageWindow("originRPY must be a sequence of 3 values", c2uLogLevel::WARN);
            return;
        }
        const auto& rpy_vec = config["originRPY"].as<std::vector<double>>();
        originRPY = { rpy_vec[0],
                      rpy_vec[1],
                      rpy_vec[2] };
    }

    if (config["exportAllUseradded"].IsDefined()) {
        exportAllUseradded = config["exportAllUseradded"].as<bool>();
    }

    if (config["exportFirstBaseLinkAdditionalFrameAsFakeURDFBase"].IsDefined())
    {
        exportFirstBaseLinkAdditionalFrameAsFakeURDFBase = config["exportFirstBaseLinkAdditionalFrameAsFakeURDFBase"].as<bool>();
    }

    if (config["urdfNumericalPrecision"].IsDefined())
    {
        urdfNumericalPrecision = config["urdfNumericalPrecision"].as<int>();
        if (urdfNumericalPrecision < 1 || urdfNumericalPrecision > 15)
        {
            printToMessageWindow("urdfNumericalPrecision must be between 1 and 15. Using default maximum precision.", c2uLogLevel::WARN);
            urdfNumericalPrecision = 0;
        }
    }

    component_handles.emplace(ComponentId{}, m_root_asm_model_ptr);
    if (!processAsmItems(asm_component_list, m_root_asm_model_ptr, iDynTree::Transform::Identity(), {}, true)
        || !resolveOccurrenceNames()) return;
    readExportedFramesFromConfig();
    readAssignedInertiasFromConfig();
    readAssignedCollisionGeometryFromConfig();

    Sensorizer sensorizer;

    sensorizer.readFTSensorsFromConfig(config);
    sensorizer.readSensorsFromConfig(config);

    // Let's traverse the model tree and get all links and axis properties
    bool ok = processAsmItems(asm_component_list, m_root_asm_model_ptr);
    if (!ok) {
        printToMessageWindow("Failed to process the assembly", c2uLogLevel::WARN);
        return;
    }

    for (const auto& link : link_info_map)
        if (!populateExportedFrameInfoMap(link.second)) return;
    for (const auto& frame : exported_frame_info_map)
        if (!frame.second.resolved)
            throw std::runtime_error("Unresolved frame: " + frame.second.cad_frame_name + " on " + frame.second.frameReferenceLink);

    // Retain legacy joint names for unique CAD models; repeated occurrences
    // use the resolved URDF names. The CSV and sensors use these final names.
    std::map<std::string, JointInfo> namedJoints;
    for (const auto& entry : joint_info_map) {
        const auto& info = entry.second;
        auto parent = link_info_map.find(info.parent_link_id);
        auto child = link_info_map.find(info.child_link_id);
        if (parent == link_info_map.end() || child == link_info_map.end()) {
            printToMessageWindow("Skipping assembly-level or cut reference at " + entry.first, c2uLogLevel::WARN);
            continue;
        }
        const auto& p = parent->second;
        const auto& c = child->second;
        const auto legacyName = p.cad_model_name + "--" + c.cad_model_name;
        const bool unique = model_counts.at(p.cad_model_name) == 1 && model_counts.at(c.cad_model_name) == 1;
        if (!unique && config["rename"][legacyName].IsDefined())
            throw std::runtime_error("Ambiguous joint rename: " + legacyName + "; use URDF link names");
        const auto name = getRenameElementFromConfig(unique ? legacyName : p.name + "--" + c.name);
        if (!namedJoints.emplace(name, info).second)
            throw std::runtime_error("Duplicate joint name: " + name);
    }
    joint_info_map = std::move(namedJoints);
    // Now we have to add joints to the iDynTree model

    for (auto & joint_info : joint_info_map) {
        auto parent_link_id = joint_info.second.parent_link_id;
        auto child_link_id = joint_info.second.child_link_id;
        auto datum_name = joint_info.second.datum_name;
        auto joint_name = joint_info.first;

        const auto& parent_link_name = link_info_map.at(parent_link_id).name;
        const auto& child_link_name = link_info_map.at(child_link_id).name;
        auto asm_owner_H_parent_link = link_info_map.at(parent_link_id).rootAsm_H_linkFrame;
        auto asm_owner_H_child_link = link_info_map.at(child_link_id).rootAsm_H_linkFrame;
        auto parent_model = link_info_map.at(parent_link_id).modelhdl;
        auto parent_link_frame = link_info_map.at(parent_link_id).link_frame_name;

        //printToMessageWindow("Parent link H " + asm_owner_H_parent_link.toString());
        //printToMessageWindow("Child  link H " + asm_owner_H_child_link.toString());
        iDynTree::Transform parentLink_H_childLink = iDynTree::Transform::Identity();
        parentLink_H_childLink = asm_owner_H_parent_link.inverse() * asm_owner_H_child_link;

        if (joint_info.second.type == JointType::Revolute || joint_info.second.type == JointType::Linear) {

            iDynTree::Direction direction;
            iDynTree::Position axis_mid_point_pos_in_parent;
            std::tie(ret, direction, axis_mid_point_pos_in_parent) = getAxisFromPart(parent_model, datum_name, parent_link_frame, scale);

            if (!ret)
            {
                printToMessageWindow("Failed to get the axis from the part " + parent_link_name + ", skipping " + joint_name, c2uLogLevel::WARN);
                if (warningsAreFatal) {
                    return;
                }
                else {
                    continue;
                }
            }

            if (config["reverseRotationAxis"].IsDefined() && 
                config["reverseRotationAxis"].Scalar().find(joint_name) != std::string::npos)
            {
                direction = direction.reverse();
            }

            iDynTree::Axis idyn_axis{ direction, parentLink_H_childLink.getPosition() };

            // Check if the axis is aligned with the link frame
            if (parent_link_frame == "CSYS" && idyn_axis.getDistanceBetweenAxisAndPoint(axis_mid_point_pos_in_parent) > 1e-7 ) {
                idyn_axis.setOrigin(axis_mid_point_pos_in_parent);
                m_need_to_move_link_frames_to_be_compatible_with_URDF = true;
            }

            std::shared_ptr<iDynTree::IJoint> joint_sh_ptr;
            if (joint_info.second.type == JointType::Revolute) {
                joint_sh_ptr = std::make_shared<iDynTree::RevoluteJoint>();
                dynamic_cast<iDynTree::RevoluteJoint*>(joint_sh_ptr.get())->setAxis(idyn_axis);
            }
            else if (joint_info.second.type == JointType::Linear) {
                joint_sh_ptr = std::make_shared<iDynTree::PrismaticJoint>();
                dynamic_cast<iDynTree::PrismaticJoint*>(joint_sh_ptr.get())->setAxis(idyn_axis);
            }

            joint_sh_ptr->setRestTransform(parentLink_H_childLink);
            double conversion_factor = 1.0;
            if (joint_info.second.type == JointType::Revolute) {
                conversion_factor = deg2rad;
            }

            // Read limits from CSV data, until it is possible to do so from Creo directly
            setJointParametersFromCsv(joints_csv_table, joint_name, *joint_sh_ptr, conversion_factor);

            if (idyn_model.addJoint(parent_link_name,
                child_link_name, joint_name, joint_sh_ptr.get()) == iDynTree::JOINT_INVALID_INDEX) {
                throw std::runtime_error("Failed to add joint " + joint_name);
            }
        }
        else if (joint_info.second.type == JointType::Fixed) {
            iDynTree::FixedJoint joint(parentLink_H_childLink);
            if (idyn_model.addJoint(parent_link_name,
                child_link_name, joint_name, &joint) == iDynTree::JOINT_INVALID_INDEX) {
                throw std::runtime_error("Failed to add joint " + joint_name);
            }
        }
            else if (joint_info.second.type == JointType::Spherical) {
            iDynTree::SphericalJoint joint;
            joint.setAttachedLinks(
                idyn_model.getLinkIndex(parent_link_name),
                idyn_model.getLinkIndex(child_link_name)
            );
            joint.setRestTransform(parentLink_H_childLink);
            iDynTree::Position center_in_parent_link = iDynTree::Position::Zero();
            std::tie(ret, center_in_parent_link) = getPointCoordFromPart(parent_model, datum_name, scale);
            joint.setJointCenter(idyn_model.getLinkIndex(parent_link_name), center_in_parent_link);
            if (idyn_model.addJoint(joint_name, &joint) == iDynTree::JOINT_INVALID_INDEX) {
                throw std::runtime_error("Failed to add joint " + joint_name);
            }
        }
    }

    // Assign the transforms for the sensors
    sensorizer.assignTransformToSensors(exported_frame_info_map, link_info_map, scale);
    // Assign the transforms for the ft sensors
    sensorizer.assignTransformToFTSensor(exported_frame_info_map, link_info_map, joint_info_map, scale);

    // Let's add sensors and ft sensors frames

    for (auto& sensor : sensorizer.sensors) {
        if (sensor.exportFrameInURDF) {
            if (!idyn_model.addAdditionalFrameToLink(sensor.linkName, sensor.exportedFrameName,
                sensor.transform)) {
                printToMessageWindow("Failed to add additional frame  " + sensor.exportedFrameName, c2uLogLevel::WARN);
                continue;
            }
        }
    }

    for (auto& ftsensor : sensorizer.ft_sensors) {
        if (ftsensor.second.exportFrameInURDF) {
            auto joint_idx = idyn_model.getJointIndex(ftsensor.first);
            if (joint_idx == iDynTree::LINK_INVALID_INDEX) {
                // TODO FATAL?!
                printToMessageWindow("Failed to add additional frame, ftsensor: " + ftsensor.second.sensorName + " is not in the model", c2uLogLevel::WARN);
                continue;
            }

            auto joint = idyn_model.getJoint(joint_idx);
            auto link_name = idyn_model.getLinkName(joint->getFirstAttachedLink());

            if (!idyn_model.addAdditionalFrameToLink(link_name, ftsensor.second.exportedFrameName,
                ftsensor.second.parent_link_H_sensor)) {
                printToMessageWindow("Failed to add additional frame  " + ftsensor.second.exportedFrameName, c2uLogLevel::WARN);
                continue;
            }
        }
    }

    // Let's add all the exported frames
    for (auto & exported_frame_info : exported_frame_info_map) {
        std::string reference_link = exported_frame_info.second.frameReferenceLink;
        if (idyn_model.getLinkIndex(reference_link) == iDynTree::LINK_INVALID_INDEX) {
            // TODO FATAL?!
            printToMessageWindow("Failed to add additional frame, link " + reference_link + " is not in the model", c2uLogLevel::WARN);
            continue;
        }
        if (!idyn_model.addAdditionalFrameToLink(reference_link, exported_frame_info.second.exportedFrameName,
            exported_frame_info.second.linkFrame_H_additionalFrame * exported_frame_info.second.additionalTransformation)) {
            throw std::runtime_error("Failed to add frame " + exported_frame_info.second.exportedFrameName);
        }
    }


    std::ofstream idyn_model_out("iDynTreeModel.txt");
    idyn_model_out << idyn_model.toString();
    idyn_model_out.close();

    iDynTree::ModelExporterOptions export_options;
    export_options.exportSphericalJointsAsThreeRevoluteJoints = true;
    export_options.robotExportedName = config["robotName"].Scalar();
    export_options.exportFirstBaseLinkAdditionalFrameAsFakeURDFBase = exportFirstBaseLinkAdditionalFrameAsFakeURDFBase;

    // Set numerical precision if explicitly configured
    if (urdfNumericalPrecision > 0)
    {
        export_options.numericalPrecision = urdfNumericalPrecision;
    }

    if (config["root"].IsDefined())
        export_options.baseLink = config["root"].Scalar();
    else
        export_options.baseLink = "root_link";

    // Add FTs and other sensors as XML blobs for now

    std::vector<std::string> ft_xml_blobs = sensorizer.buildFTXMLBlobs();
    std::vector<std::string> sens_xml_blobs = sensorizer.buildSensorsXMLBlobs();

    export_options.xmlBlobs.insert(export_options.xmlBlobs.end(), ft_xml_blobs.begin(), ft_xml_blobs.end());
    export_options.xmlBlobs.insert(export_options.xmlBlobs.end(), sens_xml_blobs.begin(), sens_xml_blobs.end());
    
    if (config["XMLBlobs"].IsDefined()) {
        auto config_xml_blobs = config["XMLBlobs"].as<std::vector<std::string>>();
        export_options.xmlBlobs.insert(export_options.xmlBlobs.end(), config_xml_blobs.begin(), config_xml_blobs.end());
        // Adding gazebo pose as xml blob at the end of the urdf.
        std::string gazebo_pose_xml_str{""};
        gazebo_pose_xml_str = to_string(originXYZ[0]) + " " + to_string(originXYZ[1]) + " " + to_string(originXYZ[2]) + " " + to_string(originRPY[0]) + " " + to_string(originRPY[1]) + " " + to_string(originRPY[2]);
        gazebo_pose_xml_str = "<gazebo><pose>" + gazebo_pose_xml_str + "</pose></gazebo>";
        export_options.xmlBlobs.push_back(gazebo_pose_xml_str);
    }

    exportModelToUrdf(idyn_model, export_options);

    // Let's clear the map in case of multiple click TODO UNIFY
    m_yaml_path.clear();
    m_csv_path.clear();
    m_output_path.clear();
    config = YAML::Node();
    m_root_asm_model_ptr = nullptr;

    return;
}

bool Creo2Urdf::setJointParametersFromCsv(const rapidcsv::Document& csv, const std::string& joint_name, 
    iDynTree::IJoint& joint, double conversion_factor = 1.0)
{
    if (csv.GetRowIdx(joint_name) < 0) return false;

    double min = csv.GetCell<double>("lower_limit", joint_name) * conversion_factor;
    double max = csv.GetCell<double>("upper_limit", joint_name) * conversion_factor;

    if (!std::isinf(min) && !std::isinf(max))
    {
        joint.enablePosLimits(true);
        joint.setPosLimits(0, min, max);
    }
    // TODO we have to retrieve the rest transform from creo
    //joint.setRestTransform();

    joint.setJointDynamicsType(iDynTree::URDFJointDynamics);
    joint.setDamping(0, csv.GetCell<double>("damping", joint_name));
    joint.setStaticFriction(0, csv.GetCell<double>("friction", joint_name));

    return true;
}

bool Creo2Urdf::exportModelToUrdf(iDynTree::Model mdl, iDynTree::ModelExporterOptions options) {
    iDynTree::ModelExporter mdl_exporter;

    // Convert modelToExport in a URDF-compatible model (using the default base link)
    iDynTree::Model modelToExportURDFCompatible;

    if (m_need_to_move_link_frames_to_be_compatible_with_URDF) {
        bool ok = iDynTree::moveLinkFramesToBeCompatibleWithURDFWithGivenBaseLink(mdl, modelToExportURDFCompatible);
        if (!ok) {
            printToMessageWindow("Failed to move link frames to be URDF compatible", c2uLogLevel::WARN);
            return false;
        }
    }
    else {
        modelToExportURDFCompatible = mdl;
    }

    mdl_exporter.init(modelToExportURDFCompatible);
    mdl_exporter.setExportingOptions(options);

    if (!mdl_exporter.isValid())
    {
        printToMessageWindow("Model is not valid!", c2uLogLevel::WARN);
        return false;
    }

    if (!mdl_exporter.exportModelToFile(m_output_path+ "\\" + "model.urdf"))
    {
        printToMessageWindow("Error exporting the urdf. See iDynTreeErrors.txt for details", c2uLogLevel::WARN);
        return false;
    }

    printToMessageWindow("Urdf created successfully!");
    return true;
}

iDynTree::SpatialInertia Creo2Urdf::computeSpatialInertiafromCreo(pfcMassProperty_ptr mass_prop, iDynTree::Transform H, const std::string& link_name) {
    auto com = mass_prop->GetGravityCenter();
    auto inertia_tensor = mass_prop->GetCenterGravityInertiaTensor();

    iDynTree::RotationalInertiaRaw idyn_inertia_tensor_csysPart_orientation = iDynTree::RotationalInertiaRaw::Zero();
    iDynTree::RotationalInertiaRaw idyn_inertia_tensor_link_orientation = iDynTree::RotationalInertiaRaw::Zero();

    bool assigned_inertia_flag = assigned_inertias_map.find(link_name) != assigned_inertias_map.end();
    for (int i_row = 0; i_row < idyn_inertia_tensor_csysPart_orientation.rows(); i_row++) {
        for (int j_col = 0; j_col < idyn_inertia_tensor_csysPart_orientation.cols(); j_col++) {
            if ((assigned_inertia_flag) && (i_row == j_col)) {
                // The assigned inertia is already expressed in the link frame
                idyn_inertia_tensor_link_orientation.setVal(i_row, j_col, assigned_inertias_map.at(link_name)[i_row]);
            }
            else {
                idyn_inertia_tensor_csysPart_orientation.setVal(i_row, j_col, inertia_tensor->get(i_row, j_col) * scale[i_row] * scale[j_col]);
            }
        }
    }

    iDynTree::Position com_child({ com->get(0) * scale[0] , com->get(1) * scale[1], com->get(2) * scale[2] });

    // Account for csysPart_H_link_frame transformation
    // See https://github.com/mesh-iit/ergocub-software/issues/224#issuecomment-1985692598 for full contents

    // The COM returned by Creo's GetGravityCenter seems to be expressed in the root frame, so we need 
    // to transform it back to the link frame before passing it to iDynTree's fromRotationalInertiaWrtCenterOfMass
    com_child = H.inverse() * com_child;

    // The inertia returned by Creo's GetCenterGravityInertiaTensor seems to be expressed with the COM as the
    // point in which it is expressed, and with the orientation of the CSYS of the part, so we rotate it back with
    // the orientation of the link frame, unless an assignedInertia is used
    if (!assigned_inertia_flag) {
        // Note, this auto-defined methods are Eigen::Map, so they are reference to data that remains
        // stored in the original iDynTree object, see https://eigen.tuxfamily.org/dox/group__TutorialMapClass.html
        auto inertia_tensor_root = iDynTree::toEigen(idyn_inertia_tensor_csysPart_orientation);
        auto inertia_tensor_link = iDynTree::toEigen(idyn_inertia_tensor_link_orientation);
        auto csysPart_R_link = iDynTree::toEigen(H.getRotation());

        // See Equation 15 of https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/dd277ec654440f4c2b5b07d6c286c3fd_MIT16_07F09_Lec26.pdf
        inertia_tensor_link = csysPart_R_link.transpose()*inertia_tensor_root*csysPart_R_link;
    }

    double mass{ 0.0 };
    if (config["assignedMasses"][link_name].IsDefined()) {
        mass = config["assignedMasses"][link_name].as<double>();
    }
    else {
        mass = mass_prop->GetMass();
    }
    iDynTree::SpatialInertia sp_inertia(mass, com_child, idyn_inertia_tensor_link_orientation);
    sp_inertia.fromRotationalInertiaWrtCenterOfMass(mass, com_child, idyn_inertia_tensor_link_orientation);

    return sp_inertia;
}

bool Creo2Urdf::resolveOccurrenceNames() {
    std::map<ComponentId, std::string> aliases;
    std::map<std::string, std::string> renames;
    for (const auto& entry : config["rename"])
        renames.emplace(entry.first.as<std::string>(), entry.second.as<std::string>());
    if (config["componentNames"] && !config["componentNames"].IsSequence())
        throw std::runtime_error("componentNames must be a sequence");
    for (const auto& entry : config["componentNames"]) {
        const auto id = entry["path"].as<ComponentId>();
        if (id.empty() || std::any_of(id.begin(), id.end(), [](int v) { return v < 0; }) ||
            !aliases.emplace(id, entry["name"].as<std::string>()).second)
            throw std::runtime_error("Invalid or repeated componentNames path: " + componentIdString(id));
    }
    // Write the inventory even if aliases fail validation, so paths can be corrected.
    YAML::Emitter inventory;
    inventory << YAML::BeginSeq;
    for (const auto& model : component_models) {
        inventory << YAML::BeginMap << YAML::Key << "path" << YAML::Value << YAML::Flow << model.first
                  << YAML::Key << "model" << YAML::Value << model.second << YAML::EndMap;
        printToMessageWindow("Component [" + componentIdString(model.first) + "]: " + model.second);
    }
    inventory << YAML::EndSeq;
    const auto inventoryPath = m_output_path + "\\component-inventory.yaml";
    { std::ofstream file(inventoryPath); file << inventory.c_str();
      if (!file) throw std::runtime_error("Cannot write component inventory"); }
    component_names = resolveComponentNames(component_models, aliases, renames);
    const auto requireLink = [&](const std::string& name) {
        for (const auto& entry : component_names) if (entry.second == name) return;
        throw std::runtime_error("Unknown or ambiguous URDF link: " + name + "; see component-inventory.yaml and componentNames");
    };
    if (config["root"]) requireLink(config["root"].as<std::string>());
    for (const auto& frame : config["linkFrames"]) requireLink(frame["linkName"].as<std::string>());
    YAML::Emitter namedInventory;
    namedInventory << YAML::BeginSeq;
    for (const auto& model : component_models)
        namedInventory << YAML::BeginMap << YAML::Key << "path" << YAML::Value << YAML::Flow << model.first
                       << YAML::Key << "model" << YAML::Value << model.second
                       << YAML::Key << "name" << YAML::Value << component_names.at(model.first) << YAML::EndMap;
    namedInventory << YAML::EndSeq;
    std::ofstream file(inventoryPath);
    file << namedInventory.c_str();
    return static_cast<bool>(file);
}

bool Creo2Urdf::populateExportedFrameInfoMap(const LinkInfo& link) {
    if (exportAllUseradded) {
        auto csys = link.modelhdl->ListItems(pfcITEM_COORD_SYS);
        for (int i = 0; i < csys->getarraysize(); ++i) {
            const std::string name = csys->get(i)->GetName();
            if (name.find("SCSYS") == std::string::npos) continue;
            size_t count = 0;
            for (const auto& other : link_info_map) {
                auto frames = other.second.modelhdl->ListItems(pfcITEM_COORD_SYS);
                for (int j = 0; j < frames->getarraysize(); ++j)
                    if (std::string(frames->get(j)->GetName()) == name) ++count;
            }
            ExportedFrameInfo frame;
            frame.cad_frame_name = name;
            frame.frameReferenceLink = link.name;
            frame.exportedFrameName = count > 1 ? name + "__" + componentIdString(link.id) : name;
            if (idyn_model.getLinkIndex(frame.exportedFrameName) != iDynTree::LINK_INVALID_INDEX ||
                !exported_frame_info_map.emplace(frame.exportedFrameName, frame).second)
                throw std::runtime_error("Duplicate exported frame: " + frame.exportedFrameName);
        }
    }
    for (auto& entry : exported_frame_info_map) {
        auto& frame = entry.second;
        if (frame.frameReferenceLink != link.name) continue;
        bool ok;
        iDynTree::Transform part_H_frame, part_H_link;
        std::tie(ok, part_H_frame) = getTransformFromPart(link.modelhdl, frame.cad_frame_name, scale);
        if (!ok) return false;
        std::tie(ok, part_H_link) = getTransformFromPart(link.modelhdl, link.link_frame_name, scale);
        if (!ok) return false;
        frame.linkFrame_H_additionalFrame = part_H_link.inverse() * part_H_frame;
        frame.resolved = true;
    }
    return true;
}

void Creo2Urdf::readAssignedInertiasFromConfig() {
    if (!config["assignedInertias"].IsDefined()) {
        return;
    }
    for (const auto& ai : config["assignedInertias"]) {
        std::array<double, 3> assignedInertia { ai["xx"].as<double>(), ai["yy"].as<double>(), ai["zz"].as<double>()};
        assigned_inertias_map.insert(std::make_pair(ai["linkName"].Scalar(), assignedInertia));
    }
}

void Creo2Urdf::readAssignedCollisionGeometryFromConfig() {
    if (!config["assignedCollisionGeometry"].IsDefined()) {
        return;
    }
    for (const auto& cg : config["assignedCollisionGeometry"]) {
        CollisionGeometryInfo cgi;
        cgi.shape = stringToEnum<ShapeType>(shape_type_map, cg["geometricShape"]["shape"].Scalar());
        switch (cgi.shape)
        {
        case ShapeType::Box:
            cgi.size = cg["geometricShape"]["size"].as < array<double, 3>>();
            break;
        case ShapeType::Cylinder:
            cgi.radius = cg["geometricShape"]["radius"].as<double>();
            cgi.length = cg["geometricShape"]["length"].as<double>();
            break;
        case ShapeType::Sphere:
            cgi.radius = cg["geometricShape"]["radius"].as<double>();
            break;
        case ShapeType::None:
            break;
        default:
            break;
        }
        if (cg["geometricShape"]["origin"].IsDefined()) {
            // Origin is defined, so load it.
            auto origin = cg["geometricShape"]["origin"].as < std::array<double, 6>>();
            cgi.link_H_geometry.setPosition({ origin[0], origin[1], origin[2] });
            cgi.link_H_geometry.setRotation(iDynTree::Rotation::RPY(origin[3], origin[4], origin[5]));
        }
        assigned_collision_geometry_map.insert(std::make_pair(cg["linkName"].Scalar(), cgi));
    }
}

void Creo2Urdf::readExportedFramesFromConfig() {
    if (!config["exportedFrames"] || exportAllUseradded) return;
    for (const auto& ef : config["exportedFrames"]) {
        ExportedFrameInfo frame;
        frame.cad_frame_name = ef["frameName"].as<std::string>();
        frame.exportedFrameName = ef["exportedFrameName"].as<std::string>();
        const auto reference = ef["frameReferenceLink"].as<std::string>("");
        size_t matches = 0;
        for (const auto& component : component_names) {
            if (!reference.empty() && reference != component.second) continue;
            auto csys = component_handles.at(component.first)->ListItems(pfcITEM_COORD_SYS);
            for (int i = 0; i < csys->getarraysize(); ++i) {
                if (std::string(csys->get(i)->GetName()) != frame.cad_frame_name) continue;
                frame.frameReferenceLink = component.second;
                ++matches;
            }
        }
        if (matches != 1)
            throw std::runtime_error("Missing or ambiguous frame " + frame.cad_frame_name + "; specify frameReferenceLink using a URDF link name");
        if (ef["additionalTransformation"]) {
            const auto values = ef["additionalTransformation"].as<std::array<double, 6>>();
            frame.additionalTransformation.setPosition({values[0], values[1], values[2]});
            frame.additionalTransformation.setRotation(iDynTree::Rotation::RPY(values[3], values[4], values[5]));
        }
        for (const auto& component : component_names)
            if (component.second == frame.exportedFrameName)
                throw std::runtime_error("Frame name collides with link: " + frame.exportedFrameName);
        if (frame.exportedFrameName.empty() || !exported_frame_info_map.emplace(frame.exportedFrameName, frame).second)
            throw std::runtime_error("Duplicate or empty exported frame name: " + frame.exportedFrameName);
    }
}
bool Creo2Urdf::addMeshAndExport(const LinkInfo& link)
{
    auto component_handle = link.modelhdl;
    const auto& mesh_transform = link.link_frame_name;
    bool export_mesh = true;
    std::string file_extension = ".stl";
    std::string meshFormat = "stl_binary";
    std::string link_name = component_handle->GetFullName();
    const std::string& renamed_link_name = link.name;

    if (config["exportMeshes"].IsDefined())
    {
        export_mesh = config["exportMeshes"].as<bool>();
    }


    if (config["stringToRemoveFromMeshFileName"].IsDefined())
    {
        const auto remove = config["stringToRemoveFromMeshFileName"].Scalar();
        const auto pos = link_name.find(remove);
        if (pos != std::string::npos) link_name.erase(pos, remove.length());
    }

    // Make all alphabetic characters lowercase
    if (config["forcelowercase"].IsDefined() && config["forcelowercase"].as<bool>())
    {
        std::transform(link_name.begin(), link_name.end(), link_name.begin(),
            [](unsigned char c) { return std::tolower(c); });
    }

    if (config["meshFormat"].IsDefined()) {
        meshFormat = config["meshFormat"].Scalar();
        if (mesh_types_supported_extension_map.find(meshFormat) != mesh_types_supported_extension_map.end()) {
            file_extension = mesh_types_supported_extension_map.at(meshFormat);
        }
        else {
            printToMessageWindow("Mesh format " + meshFormat + " is not supported", c2uLogLevel::WARN);
            if (warningsAreFatal) {
                return false;
            }
        }
    }
    // Assign name
    string file_format = "%s";
    int mesh_quality = 3;
    if (config["filenameformat"].IsDefined()) {
        file_format = config["filenameformat"].Scalar();
    }
    else if (meshFormat != "step") {
        // We use ExportIntf3D for step format, applies the extension to the file name.
        file_format += file_extension;
    }

    if (config["meshQuality"].IsDefined()) {
        mesh_quality = config["meshQuality"].as<int>();
        if (mesh_quality < 1 || mesh_quality > 10) {
            printToMessageWindow("Mesh quality is too low, or too hight, the range is between 1 and 10", c2uLogLevel::WARN);
            if (warningsAreFatal) {
                printToMessageWindow("Aborting the mesh exportation", c2uLogLevel::WARN);
                return false;
            }
        }
    }

    if (model_counts.at(link.cad_model_name) > 1)
        link_name += "__" + componentIdString(link.id);
    if (file_format.find("%s") == std::string::npos)
        throw std::runtime_error("filenameformat must contain %s");
    // We assume there is only one of occurrence to replace
    file_format.replace(file_format.find("%s"), 2, link_name); // 2 is sizeof %s, in this way we keep the formatting extension

    const auto basename = file_format.substr(file_format.find_last_of("/\\") + 1);
    std::string collisionKey = basename;
    std::transform(collisionKey.begin(), collisionKey.end(), collisionKey.begin(), [](unsigned char c) { return std::tolower(c); });
    if (!mesh_file_names.insert(collisionKey).second)
        throw std::runtime_error("Duplicate mesh file name: " + basename);
    if (export_mesh)
    {
        const std::string mesh_file_name = m_output_path + "\\" + basename;

        try {
            if (meshFormat == "stl_binary") {
                auto stl_binary_export_instructions = pfcSTLBinaryExportInstructions().Create(mesh_transform.c_str());
                stl_binary_export_instructions->SetQuality(mesh_quality);
                component_handle->Export(mesh_file_name.c_str(), pfcExportInstructions::cast(stl_binary_export_instructions));
            }
            else if(meshFormat =="stl_ascii") {
                auto stl_ascii_export_instructions = pfcSTLASCIIExportInstructions().Create(mesh_transform.c_str());
                stl_ascii_export_instructions->SetQuality(mesh_quality);
                component_handle->Export(mesh_file_name.c_str(), pfcExportInstructions::cast(stl_ascii_export_instructions));
            }
            else if (meshFormat == "step") {
                component_handle->ExportIntf3D(mesh_file_name.c_str(), pfcExportType::pfcEXPORT_STEP);
            }
            else {
                return false;
            }

        }
        xcatchbegin
        xcatchcip(defaultEx)
        {
            printToMessageWindow(": exception caught: " + string(pfcXPFC::cast(defaultEx)->GetMessage()));
            return false;
        }
        xcatchend

        // Replace the first 5 bytes of the binary file with a string different than "solid"
        // to avoid issues with stl parsers.
        // For details see: https://github.com/mesh-iit/creo2urdf/issues/16
        if (meshFormat == "stl_binary") {
            sanitizeSTL(mesh_file_name);
        }
    }

    // Lets add the mesh to the link
    iDynTree::ExternalMesh visualMesh;
    // Meshes are in millimeters, while iDynTree models are in meters
    visualMesh.setScale({scale});

    iDynTree::Vector4 color;
    iDynTree::Material material;

    if(config["assignedColors"][renamed_link_name].IsDefined())
    {
        for (size_t i = 0; i < config["assignedColors"][renamed_link_name].size(); i++)
            color(i) = config["assignedColors"][renamed_link_name][i].as<double>();
    } 
    else
    {
        color(0) = color(1) = color(2) = 0.5;
        color(3) = 1;
    }

    material.setColor(color);
    visualMesh.setMaterial(material);
    // Assign transform
    // TODO Right now maybe it is not needed it ie exported respct the link csys
    // visualMesh.setLink_H_geometry(H_parent_to_child);

    visualMesh.setFilename(file_format);

    if (assigned_collision_geometry_map.find(renamed_link_name) != assigned_collision_geometry_map.end()) {
        auto geometry_info = assigned_collision_geometry_map.at(renamed_link_name);
        switch (geometry_info.shape)
        {
        case ShapeType::Box: {
            iDynTree::Box idyn_box;
            idyn_box.setX(geometry_info.size[0]); idyn_box.setY(geometry_info.size[1]); idyn_box.setZ(geometry_info.size[2]);
            idyn_box.setLink_H_geometry(geometry_info.link_H_geometry);
            idyn_model.collisionSolidShapes().getLinkSolidShapes()[idyn_model.getLinkIndex(renamed_link_name)].push_back(idyn_box.clone());
        }
            break;
        case ShapeType::Cylinder: {
            iDynTree::Cylinder idyn_cylinder;
            idyn_cylinder.setLength(geometry_info.length);
            idyn_cylinder.setRadius(geometry_info.radius);
            idyn_cylinder.setLink_H_geometry(geometry_info.link_H_geometry);
            idyn_model.collisionSolidShapes().getLinkSolidShapes()[idyn_model.getLinkIndex(renamed_link_name)].push_back(idyn_cylinder.clone());
        }
            break;
        case ShapeType::Sphere: {
            iDynTree::Sphere idyn_sphere;
            idyn_sphere.setRadius(geometry_info.radius);
            idyn_sphere.setLink_H_geometry(geometry_info.link_H_geometry);
            idyn_model.collisionSolidShapes().getLinkSolidShapes()[idyn_model.getLinkIndex(renamed_link_name)].push_back(idyn_sphere.clone());
        }
            break;
        case ShapeType::None:
            idyn_model.collisionSolidShapes().getLinkSolidShapes()[idyn_model.getLinkIndex(renamed_link_name)].clear();
            break;
        default:
            break;
        }

    }
    else {
        idyn_model.collisionSolidShapes().getLinkSolidShapes()[idyn_model.getLinkIndex(renamed_link_name)].push_back(visualMesh.clone());
    }
    idyn_model.visualSolidShapes().getLinkSolidShapes()[idyn_model.getLinkIndex(renamed_link_name)].push_back(visualMesh.clone());


    return true;
}

bool Creo2Urdf::loadYamlConfig(const std::string& filename)
{
    try 
    {
        config = YAML::LoadFile(filename);
        if (config["includes"].IsDefined() && config["includes"].IsSequence()) {
            auto folder_path = extractFolderPath(filename);
            for (const auto& include : config["includes"]) {
                auto include_filename = folder_path + include.as<std::string>();
                auto include_config = YAML::LoadFile(include_filename);
                mergeYAMLNodes(config, include_config);
            }
        }
    }
    catch (YAML::BadFile file_does_not_exist) 
    {
        printToMessageWindow("Configuration file " + filename + " does not exist!", c2uLogLevel::WARN);
        return false;
    }
    catch (YAML::ParserException badly_formed) 
    {
        printToMessageWindow(badly_formed.msg, c2uLogLevel::WARN);
        return false;
    }

    printToMessageWindow("Configuration file " + filename + " was loaded successfully");

    return true;
}

std::string Creo2Urdf::getRenameElementFromConfig(const std::string& elem_name)
{
    if (config["rename"][elem_name].IsDefined())
    {
        std::string new_name = config["rename"][elem_name].Scalar();
        return new_name;
    }
    else
    {
        printToMessageWindow("Element " + elem_name + " is not present in the configuration file!", c2uLogLevel::WARN);
        return elem_name;
    }
}

pfcCommandAccess Creo2UrdfAccess::OnCommandAccess(xbool AllowErrorMessages)
{
    auto model = pfcGetProESession()->GetCurrentModel();
    if (!model) {
        return pfcCommandAccess::pfcACCESS_AVAILABLE;
    }
    auto type = model->GetType();
    if (type != pfcMDL_PART && type != pfcMDL_ASSEMBLY) {
        return pfcCommandAccess::pfcACCESS_UNAVAILABLE;
    }
    return pfcCommandAccess::pfcACCESS_AVAILABLE;
}
