/*
/*
 * Copyright (C) 2006-2023 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <creo2urdf/Validator.h>


bool Validator::loadUrdfFromFile(const std::string& filename) {
    iDynTree::ModelLoader mdl_loader;
    if (!mdl_loader.loadModelFromFile(filename))
    {
        printToMessageWindow("Could not load urdf for validation!", c2uLogLevel::WARN);
        return false;
    }

    idyn_model = mdl_loader.model();

    return true;
}

bool Validator::assignCreoTransformToLink() {
    bool ret{ false };

    iDynTree::Transform H_child = iDynTree::Transform::Identity();

    auto components = creo_model_ptr->ListItems(pfcModelItemType::pfcITEM_FEATURE);
    if (components->getarraysize() == 0) {
        printToMessageWindow("There are no FEATURES in the asm", c2uLogLevel::WARN);
        return false;
    }
    // Let's first retrieve the trfs from root to all the links and store it in a map.
    for (int i = 0; i < components->getarraysize(); i++)
    {

        auto feat = pfcFeature::cast(components->get(i));

        if (feat->GetFeatType() != pfcFeatureType::pfcFEATTYPE_COMPONENT)
        {
            continue;
        }

        xintsequence_ptr seq = xintsequence::create();
        seq->append(feat->GetId());

        pfcComponentPath_ptr comp_path = pfcCreateComponentPath(pfcAssembly::cast(creo_model_ptr), seq);
        auto modelhdl = creo_session_ptr->RetrieveModel(pfcComponentFeat::cast(feat)->GetModelDescr());
        auto link_child_name = string(modelhdl->GetFullName());

        ret = true;

        if (!ret) {
            return ret;
        }

        link_name_to_creo_computed_trf_map[link_child_name] = H_child;
    }

    return true;
}

bool Validator::validatePositions(iDynTree::VectorDynSize positions) {

    // Setting the `world_T_base`
    iDynTree::Vector3 gravity; gravity.zero(); gravity(2) = gravity_z;
    auto H_base = link_name_to_creo_computed_trf_map.at("SIM_ECUB_HEAD_NECK_1");


    if (!computer.setRobotState(H_base, positions,
        iDynTree::Twist(),
        iDynTree::VectorDynSize(idyn_model.getNrOfDOFs()),
        gravity))
    {
        printToMessageWindow("Could not set the robot state!", c2uLogLevel::WARN);
        return false;
    }

    // Lets ask the same transform to idyntree and check the difference w/ the
    // creo computed one
    for (int i = 0; i < idyn_model.getNrOfLinks(); i++) {
        auto link_name = idyn_model.getLinkName(i);
        auto idyn_trf = computer.getWorldTransform(link_name);
        auto creo_trf = link_name_to_creo_computed_trf_map.at(link_name);

        iDynTree::Position position_error = idyn_trf.getPosition() - creo_trf.getPosition();
        double position_error_mag = sqrt(position_error[0] * position_error[0] + 
                                         position_error[1] * position_error[1] + 
                                         position_error[2] * position_error[2]);
   
        if (position_error_mag > epsilon)
        {
            printToMessageWindow("Position error for " + link_name + " is outside tolerance! : " + to_string(position_error_mag), c2uLogLevel::WARN);
        }

        double angle_error;

        angle_error = idyn_trf.getRotation().asRPY()[0] - creo_trf.getRotation().asRPY()[0];
        if (angle_error > epsilon * 1e3)
        {
            printToMessageWindow("Roll error for " + link_name + " is outside tolerance! " + to_string(angle_error), c2uLogLevel::WARN);
        }

        angle_error = idyn_trf.getRotation().asRPY()[1] - creo_trf.getRotation().asRPY()[1];
        if (angle_error > epsilon * 1e3)
        {
            printToMessageWindow("Pitch error for " + link_name + " is outside tolerance! " + to_string(angle_error), c2uLogLevel::WARN);
        }

        angle_error = idyn_trf.getRotation().asRPY()[2] - creo_trf.getRotation().asRPY()[2];
        if (angle_error > epsilon * 1e3)
        {
            printToMessageWindow("Yaw error for " + link_name + " is outside tolerance! " + to_string(angle_error), c2uLogLevel::WARN);
        }

        //printToMessageWindow("IDYN TRANSFORM: " + idyn_trf.toString());
        //printToMessageWindow("CREO TRANSFORM: " + creo_trf.toString());
    }

    return true;

}

void Validator::OnCommand() {

    printToMessageWindow("Model validation is not yet implemented", c2uLogLevel::INFO);

    /*
    iDynRedirectErrors idyn_redirect;
    idyn_redirect.redirectBuffer(std::cerr.rdbuf(), "iDynTreeErrors.txt");

    if (!loadUrdfFromFile("model.urdf")) {
        return;
    }

    creo_session_ptr = pfcGetProESession();
    creo_model_ptr = creo_session_ptr->GetCurrentModel();

    iDynTree::VectorDynSize positions(idyn_model.getNrOfDOFs());
    for (int i = 0; i < idyn_model.getNrOfJoints(); i++) {
        printToMessageWindow("Please insert value for joint " + idyn_model.getJointName(i), c2uLogLevel::PROMPT);
        positions[i] = creo_session_ptr->UIReadRealMessage(-360.0, 360.0) * deg2rad;
        printToMessageWindow("joint" + to_string(i) + " value " + to_string(positions[i]), c2uLogLevel::INFO);
    }

    if (!assignCreoTransformToLink()) {
        return;
    }

    printToMessageWindow("Running model validation");

    computer.loadRobotModel(idyn_model);

    if (!validatePositions(positions))
    {
        printToMessageWindow("Validation unsuccessful!", c2uLogLevel::WARN);
    }
    else
    {
        printToMessageWindow("Validation successful!");
    }
    */

    return;
}

pfcCommandAccess ValidatorAccess::OnCommandAccess(xbool AllowErrorMessages) {
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
