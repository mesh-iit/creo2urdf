
#include <iDynTree/Core/Axis.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/Indices.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/RevoluteJoint.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/SixAxisForceTorqueSensor.h>

#include <cmath>
#include <cstdlib>

inline bool checkDoubleAreEqual(const double& val1,
    const double& val2,
    const double tol)
{
    if (std::fabs(val1 - val2) > tol)
    {
        return false;
    }

    return true;
}


template<typename VectorType1, typename VectorType2>
bool checkVectorAreEqual(const VectorType1& dir1,
    const VectorType2& dir2,
    const double tol)
{
    if (dir1.size() != dir2.size())
    {
        return false;
    }

    for (int i = 0; i < dir1.size(); i++)
    {
        if (std::fabs(dir1(i) - dir2(i)) > tol)
        {
            return false;
        }
    }
    return true;
}

template<typename MatrixType1, typename MatrixType2>
bool checkMatrixAreEqual(const MatrixType1& mat1,
    const MatrixType2& mat2,
    const double tol)
{
    if (mat1.rows() != mat2.rows() ||
        mat1.cols() != mat2.cols())
    {
        return false;
    }

    for (int i = 0; i < mat1.rows(); i++)
    {
        for (int j = 0; j < mat1.cols(); j++)
        {
            if (std::fabs(mat1(i, j) - mat2(i, j)) > tol)
            {
                return false;
            }
        }
    }
    return true;
}

bool checkTransformAreEqual(const iDynTree::Transform& t1,
    const iDynTree::Transform& t2,
    const double tol)
{
    return checkMatrixAreEqual(t1.getRotation(), t2.getRotation(), tol) &&
        checkVectorAreEqual(t1.getPosition(), t2.getPosition(), tol);
}


bool getAxisInRootLink(iDynTree::KinDynComputations& comp,
    const std::string jointName,
    iDynTree::Axis& axisInRootLink)
{
    iDynTree::LinkIndex rootLinkIdx = comp.getFrameIndex("root_link");

    if (rootLinkIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "creo2urdf-model-test error: impossible to find root_link in model" << std::endl;
        return false;
    }

    iDynTree::JointIndex jntIdx = comp.getRobotModel().getJointIndex(jointName);

    if (jntIdx == iDynTree::JOINT_INVALID_INDEX)
    {
        std::cerr << "creo2urdf-model-test error: impossible to find " << jointName << " in model" << std::endl;
        return false;
    }

    iDynTree::LinkIndex childLinkIdx = comp.getRobotModel().getJoint(jntIdx)->getSecondAttachedLink();

    // Check that the joint are actually revolute as all the joints in iCub
    const iDynTree::RevoluteJoint* revJoint = dynamic_cast<const iDynTree::RevoluteJoint*>(comp.getRobotModel().getJoint(jntIdx));

    if (!revJoint)
    {
        std::cerr << "creo2urdf-model-test error: " << jointName << " is not revolute " << std::endl;
        return false;
    }

    if (!revJoint->hasPosLimits())
    {
        std::cerr << "creo2urdf-model-test error: " << jointName << " is a continous joint" << std::endl;
        return false;
    }

    axisInRootLink = comp.getRelativeTransform(rootLinkIdx, childLinkIdx) * (revJoint->getAxis(childLinkIdx));

    return true;
}

bool checkBaseLink(iDynTree::KinDynComputations& comp)
{
    iDynTree::LinkIndex rootLinkIdx = comp.getFrameIndex("root_link");

    if (rootLinkIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "creo2urdf-model-test error: impossible to find root_link in model" << std::endl;
        return false;
    }

    iDynTree::LinkIndex base_linkIdx = comp.getFrameIndex("base_link");

    if (rootLinkIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "creo2urdf-model-test error: impossible to find base_link in model" << std::endl;
        return false;
    }

    if (comp.getRobotModel().getFrameLink(base_linkIdx) != rootLinkIdx)
    {
        std::cerr << "creo2urdf-model-test error: base_link is not attached to root_link" << std::endl;
        return false;
    }

    if (!checkTransformAreEqual(comp.getRobotModel().getFrameTransform(base_linkIdx), iDynTree::Transform::Identity(), 1e-6))
    {
        std::cerr << "creo2urdf-model-test error: base_link <---> root_link transform is not an identity" << std::endl;
        return false;
    }

    std::cerr << "creo2urdf-model-test : base_link test performed correctly " << std::endl;

    return true;
}


bool checkAxisDirections(iDynTree::KinDynComputations& comp)
{

    std::vector<std::string> axisNames;
    std::vector<iDynTree::Direction> expectedDirectionInRootLink;
    axisNames.push_back("torso_roll");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-1, 0, 0));
    axisNames.push_back("l_hip_pitch");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0, -1, 0));


    for (int i = 0; i < axisNames.size(); i++)
    {
        std::string axisToCheck = axisNames[i];
        iDynTree::Axis axisInRootLink;
        iDynTree::Direction expectedDirection = expectedDirectionInRootLink[i];
        bool getAxisOk = getAxisInRootLink(comp, axisToCheck, axisInRootLink);

        if (!getAxisOk) {
            return false;
        }

        if (!checkVectorAreEqual(axisInRootLink.getDirection(), expectedDirection, 1e-5))
        {
            std::cerr << "creo2urdf-model-test error:" << axisToCheck << " got direction of " << axisInRootLink.getDirection().toString()
                << " instead of expected " << expectedDirection.toString() << std::endl;
            return false;
        }
    }

    return true;
}

/**
 * All the iCub have a odd and not null number of F/T sensors.
 */
bool checkFTSensorsAreOddAndNotNull(iDynTree::ModelLoader& mdlLoader)
{
    int nrOfFTSensors = mdlLoader.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);

    if (nrOfFTSensors == 0)
    {
        std::cerr << "creo2urdf-model-test error: no F/T sensor found in the model" << std::endl;
        return false;
    }

    if (nrOfFTSensors % 2 == 0)
    {
        std::cerr << "creo2urdf-model-test : even number of F/T sensor found in the model" << std::endl;
        return false;
    }


    return true;
}

/**
 * All the iCub have a even and not null number of F/T sensors.
 */
bool checkFTSensorsAreEvenAndNotNull(iDynTree::ModelLoader& mdlLoader)
{
    int nrOfFTSensors = mdlLoader.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);

    if (nrOfFTSensors == 0)
    {
        std::cerr << "creo2urdf-model-test error: no F/T sensor found in the model" << std::endl;
        return false;
    }

    if (nrOfFTSensors % 2 == 1)
    {
        std::cerr << "creo2urdf-model-test : odd number of F/T sensor found in the model" << std::endl;
        return false;
    }


    return true;
}


bool checkFrameIsCorrectlyOriented(iDynTree::KinDynComputations& comp,
    const iDynTree::Rotation& expected,
    const std::string& frameName)
{
    // Depending on the ergocub model, the sensor could be absent
    if (!comp.model().isFrameNameUsed(frameName))
    {
        return true;
    }

    iDynTree::Rotation actual = comp.getRelativeTransform("root_link", frameName).getRotation();

    if (!checkMatrixAreEqual(expected, actual, 1e-3))
    {
        std::cerr << "creo2urdf-model-test : transform between root_link and " << frameName << " is not the expected one, test failed." << std::endl;
        std::cerr << "creo2urdf-model-test : Expected transform : " << expected.toString() << std::endl;
        std::cerr << "creo2urdf-model-test : Actual transform : " << actual.toString() << std::endl;
        return false;
    }

    return true;
}



bool checkFTSensorsAreCorrectlyOriented(iDynTree::KinDynComputations& comp)
{

    iDynTree::Rotation rootLink_R_sensorFrameLeftArmExpected =
        iDynTree::Rotation(-0.250563, -0.961047, 0.116648,
            -0.935113, 0.271447, 0.227771,
            -0.250563, -0.0520081, -0.966702);
    iDynTree::Rotation rootLink_R_sensorFrameRightArmExpected =
        iDynTree::Rotation(-0.250563, 0.961047, 0.116648,
            0.935113, 0.271447, -0.227771,
            -0.250563, 0.0520081, -0.966702);

    iDynTree::Rotation rootLink_R_sensorFrameExpectedFoot =
        iDynTree::Rotation(-0.5, 0.866025, 0,
            -0.866025, -0.5, 0,
            0, 0, 1);

    iDynTree::Rotation rootLink_R_sensorFrameExpectedLeg =
        iDynTree::Rotation(-0.866025, -0.5, 0,
            -0.5, 0.866025, 0,
            0, 0, -1);

    iDynTree::Rotation rootLink_L_sensorFrameExpectedLeg =
        iDynTree::Rotation(-0.866025, 0.5, 0,
            0.5, 0.866025, 0,
            0, 0, -1);

    bool ok = checkFrameIsCorrectlyOriented(comp, rootLink_R_sensorFrameLeftArmExpected, "l_arm_ft");
    ok = checkFrameIsCorrectlyOriented(comp, rootLink_R_sensorFrameRightArmExpected, "r_arm_ft") && ok;
    ok = checkFrameIsCorrectlyOriented(comp, rootLink_L_sensorFrameExpectedLeg, "l_leg_ft") && ok;
    ok = checkFrameIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpectedLeg, "r_leg_ft") && ok;
    ok = checkFrameIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpectedFoot, "l_foot_rear_ft") && ok;
    ok = checkFrameIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpectedFoot, "r_foot_rear_ft") && ok;
    ok = checkFrameIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpectedFoot, "l_foot_front_ft") && ok;
    ok = checkFrameIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpectedFoot, "r_foot_front_ft") && ok;
    return ok;
}


bool Model_isFrameNameUsed(const iDynTree::Model& model, const std::string frameName)
{
    for (size_t i = 0; i < model.getNrOfLinks(); i++)
    {
        if (frameName == model.getLinkName(i))
        {
            return true;
        }
    }

    for (size_t i = model.getNrOfLinks(); i < model.getNrOfFrames(); i++)
    {

        if (frameName == model.getFrameName(i))
        {
            return true;
        }
    }

    return false;
}

bool checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(const std::string& modelPath,
    iDynTree::KinDynComputations& comp,
    const iDynTree::SensorsList& sensors,
    const std::string& sensorName)
{
    // As of mid 2023, for iCub 3 models the frame name is <prefix>_ft, while the sensor name is <prefix>_ft,
    // and the joint name is <prefix>_ft_sensor
    std::string frameName = sensorName;

    //std::cerr << comp.model().toString() << std::endl;

    // Check frame exist
    if (!comp.model().isFrameNameUsed(frameName))
    {
        std::cerr << "creo2urdf-model-test : model " << modelPath << " does not contain frame " << frameName << " as expected." << std::endl;
        return false;
    }

    // Check sensors exists
    std::ptrdiff_t sensorIdx;
    if (!sensors.getSensorIndex(iDynTree::SIX_AXIS_FORCE_TORQUE, sensorName, sensorIdx))
    {
        std::cerr << "creo2urdf-model-test : model " << modelPath << " does not contain FT sensor " << sensorName << " as expected." << std::endl;
        return false;
    }

    // Get root_H_link
    iDynTree::Transform root_H_frame = comp.getRelativeTransform("root_link", frameName);

    // Get root_H_sensor
    iDynTree::SixAxisForceTorqueSensor* sens
        = (::iDynTree::SixAxisForceTorqueSensor*)sensors.getSensor(::iDynTree::SIX_AXIS_FORCE_TORQUE, sensorIdx);
    if (!sens)
    {
        std::cerr << "creo2urdf-model-test : model " << modelPath << " error in reading sensor " << sensorName << "." << std::endl;
        return false;
    }

    std::string firstLinkName = sens->getFirstLinkName();
    iDynTree::Transform root_H_firstLink = comp.getRelativeTransform("root_link", firstLinkName);
    iDynTree::Transform firstLink_H_sensor;
    bool ok = sens->getLinkSensorTransform(sens->getFirstLinkIndex(), firstLink_H_sensor);

    if (!ok)
    {
        std::cerr << "creo2urdf-model-test : model " << modelPath << " error in reading transform of sensor " << sensorName << "." << std::endl;
        return false;
    }

    //std::cout<<firstLink_H_sensor.toString()<<std::endl;
    //std::cout<<firstLinkName<<" "<<sens->getSecondLinkName()<<std::endl;
    //std::cout<<root_H_firstLink.toString();

    iDynTree::Transform root_H_sensor = root_H_firstLink * firstLink_H_sensor;

    // Check that the two transfom are equal equal
    if (!checkTransformAreEqual(root_H_frame, root_H_sensor, 1e-5))
    {
        std::cerr << "creo2urdf-model-test : transform between root_H_frame and root_H_sensor for " << sensorName << " is not the expected one, test failed." << std::endl;
        std::cerr << "creo2urdf-model-test : root_H_frame :" << root_H_frame.toString() << std::endl;
        std::cerr << "creo2urdf-model-test : root_H_sensor :" << root_H_sensor.toString() << std::endl;
        return false;
    }

    // Beside checking that root_H_frame and root_H_sensor, we should also check that the translation
    // between the child link of FT joint and the FT frame is the zero vector, as as of mid 2023 the SDF
    // standard always assume that the 6D FT measured by the sensor is expressed in the origin of the child link frame
    // See https://github.com/gazebosim/sdformat/issues/130 for more details
    iDynTree::Traversal traversalWithURDFBase;
    comp.model().computeFullTreeTraversal(traversalWithURDFBase);

    iDynTree::LinkIndex childLinkIdx = traversalWithURDFBase.getChildLinkIndexFromJointIndex(comp.model(), sens->getParentJointIndex());
    std::string childLinkName = comp.model().getLinkName(childLinkIdx);

    iDynTree::Transform childLink_H_sensorFrame = comp.getRelativeTransform(childLinkName, frameName);

    iDynTree::Vector3 zeroVector;
    zeroVector.zero();

    if (!checkVectorAreEqual(childLink_H_sensorFrame.getPosition(), zeroVector, 1e-6))
    {
        std::cerr << "creo2urdf-model-test : translation between link " << childLinkName << " and sensor " << sensorName << " is non-zero, test failed, see  https://github.com/gazebosim/sdformat/issues/130  for more details." << std::endl;
        std::cerr << "creo2urdf-model-test : childLink_H_sensorFrame.getPosition(): " << childLink_H_sensorFrame.getPosition().toString() << std::endl;
        return false;
    }

    return true;
}

// Check FT sensors
// This is only possible with V3 as V3 models have FT frame exported models
// However, as of mid 2023 the V2 models do not need this check as the link explicitly
// are using the FT frames as frames of the corresponding link
bool checkAllFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(const std::string& modelPath)
{
    iDynTree::ModelLoader mdlLoader;

    // Open the model
    iDynTree::ModelParserOptions parserOptions;

    // By default iDynTree creates an additional frame with the same name of the sensor,
    // however in this case we have both the sensor and the urdf frame called <prefix>_ft,
    // and for this test we want to make sure that the <prefix>_ft additional frame is the
    // one in the URDF
    parserOptions.addSensorFramesAsAdditionalFrames = false;
    mdlLoader.setParsingOptions(parserOptions);

    mdlLoader.loadModelFromFile(modelPath);

    iDynTree::KinDynComputations comp;
    const bool modelLoaded = comp.loadRobotModel(mdlLoader.model());

    if (!modelLoaded)
    {
        std::cerr << "creo2urdf-model-test error: impossible to load model from " << modelLoaded << std::endl;
        return false;
    }

    iDynTree::Vector3 grav;
    grav.zero();
    iDynTree::JointPosDoubleArray qj(comp.getRobotModel());
    iDynTree::JointDOFsDoubleArray dqj(comp.getRobotModel());
    qj.zero();
    dqj.zero();

    comp.setRobotState(qj, dqj, grav);
    iDynTree::SensorsList sensors = mdlLoader.sensors();


    bool ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, "l_arm_ft");
    ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, "r_arm_ft") && ok;
    ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, "l_leg_ft") && ok;
    ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, "r_leg_ft") && ok;
    ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, "l_foot_rear_ft") && ok;
    ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, "r_foot_rear_ft") && ok;
    ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, "l_foot_front_ft") && ok;
    ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, "r_foot_front_ft") && ok;
    return ok;
}

int main(int argc, char** argv)
{



    const std::string modelPath = __argc > 1 ? __argv[1] : "model.urdf";

    iDynTree::ModelLoader mdlLoader;
    mdlLoader.loadModelFromFile(modelPath);

    // Open the model
    iDynTree::KinDynComputations comp;
    const bool modelLoaded = comp.loadRobotModel(mdlLoader.model());

    if (!modelLoaded)
    {
        std::cerr << "creo2urdf-model-test error: impossible to load model from " << modelLoaded << std::endl;
        return EXIT_FAILURE;
    }

    iDynTree::Vector3 grav;
    grav.zero();
    iDynTree::JointPosDoubleArray qj(comp.getRobotModel());
    iDynTree::JointDOFsDoubleArray dqj(comp.getRobotModel());
    qj.zero();
    dqj.zero();

    comp.setRobotState(qj, dqj, grav);

    // Check axis
    if (!checkAxisDirections(comp))
    {
        return EXIT_FAILURE;
    }
    // Check if base_link exist, and check that is a frame attached to root_link and if its
    // transform is the idyn
    if (!checkBaseLink(comp))
    {
        return EXIT_FAILURE;
    }

    // Now some test that test the sensors
    // The ft sensors orientation respect to the root_link are different to iCubV2 and they are under investigation.
    if (!checkFTSensorsAreEvenAndNotNull(mdlLoader))
    {
        return EXIT_FAILURE;
    }

    if (!checkFTSensorsAreCorrectlyOriented(comp))
    {
        return EXIT_FAILURE;
    }

    if (!checkAllFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath))
    {
        return EXIT_FAILURE;
    }


    std::cerr << "Check for model " << modelPath << " concluded correctly!" << std::endl;

    return EXIT_SUCCESS;
}