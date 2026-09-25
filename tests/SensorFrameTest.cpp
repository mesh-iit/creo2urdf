#include <creo2urdf/Sensorizer.h>
#include <cmath>
#include <iostream>

static int datumLookups = 0;
std::pair<bool, iDynTree::Transform> getTransformFromPart(
    pfcModel_ptr, const std::string&, const std::array<double, 3>&) {
    ++datumLookups;
    return {false, iDynTree::Transform::Identity()};
}
void printToMessageWindow(std::string, c2uLogLevel) {}

static void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

int main() {
    try {
        LinkInfo parent, child;
        parent.id = {40}; parent.name = "base";
        child.id = {75}; child.name = "arm";
        parent.rootAsm_H_linkFrame = iDynTree::Transform::Identity();
        child.rootAsm_H_linkFrame = iDynTree::Transform::Identity();
        parent.rootAsm_H_linkFrame.setPosition(iDynTree::Position(2, 0, 0));
        child.rootAsm_H_linkFrame.setPosition(iDynTree::Position(5, 0, 0));
        const std::map<ComponentId, LinkInfo> links{{parent.id, parent}, {child.id, child}};
        Sensorizer sensors;
        sensors.readSensorsFromConfig(YAML::Load(
            "sensors:\n  - linkName: arm\n    frameName: ''\n    sensorType: accelerometer\n"));
        sensors.assignTransformToSensors({}, links, {1, 1, 1});
        check(std::abs(sensors.sensors.at(0).transform.getPosition()(0)) < 1e-12,
              "Empty frame must coincide with the link, even away from world origin");
        sensors.sensors.at(0).frameReferenceLink = "base";
        sensors.assignTransformToSensors({}, links, {1, 1, 1});
        check(std::abs(sensors.sensors.at(0).transform.getPosition()(0) + 3) < 1e-12,
              "Empty frame must respect an explicit reference link");
        JointInfo joint;
        joint.parent_link_id = parent.id; joint.child_link_id = child.id;
        FTSensorInfo ft{};
        ft.frameName = ""; ft.frameReferenceLink = "arm";
        sensors.ft_sensors.emplace("hinge", ft);
        sensors.assignTransformToFTSensor({}, links, {{"hinge", joint}}, {1, 1, 1});
        check(std::abs(sensors.ft_sensors.at("hinge").child_link_H_sensor.getPosition()(0)) < 1e-12 &&
              std::abs(sensors.ft_sensors.at("hinge").parent_link_H_sensor.getPosition()(0) - 3) < 1e-12,
              "Empty FT frame must yield consistent parent and child transforms");
        check(datumLookups == 0, "Empty frame triggered a Creo datum lookup");
        sensors.sensors.at(0).frameName = "MISSING";
        bool rejected = false;
        try { sensors.assignTransformToSensors({}, links, {1, 1, 1}); }
        catch (const std::runtime_error&) { rejected = true; }
        check(rejected && datumLookups == 1, "Missing named frames must still fail");
        std::cout << "Sensor frame tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
