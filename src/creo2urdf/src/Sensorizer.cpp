/**
 * @file Sensorizer.cpp
 * @brief Contains definitions for the Sensorizer class.
 * @copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <creo2urdf/Sensorizer.h>

void Sensorizer::readSensorsFromConfig(const YAML::Node & config)
{
    m_config = config;
    if (!config["sensors"].IsDefined())
        return;

    for (const auto& s : config["sensors"]) {

        bool export_frame = false;

        // This is the only key that is not uniformly defined
        // in the config of sensors
        if (s["exportFrameInURDF"].IsDefined())
        {
            export_frame = s["exportFrameInURDF"].as<bool>();
        }

        double update_rate = 100;
        if (s["updateRate"].IsDefined())
        {
            update_rate = s["updateRate"].as<double>();
        }

        std::string sensor_name = s["linkName"].Scalar() + "_" + s["frameName"].Scalar();
        if (s["sensorName"].IsDefined()) {
            sensor_name = s["sensorName"].Scalar();
        }
        std::string exported_frame_name = sensor_name;
        if (s["exportedFrameName"].IsDefined()) {
            exported_frame_name = s["exportedFrameName"].Scalar();
        }
        std::vector<string> sensor_blobs;
        if (s["sensorBlobs"].IsDefined())
        {
            sensor_blobs = s["sensorBlobs"].as<std::vector<std::string>>();
        }

        try
        {
            sensors.push_back({ sensor_name,
                                s["frameName"].Scalar(),
                                s["linkName"].Scalar(),
                                exported_frame_name,
                                iDynTree::Transform::Identity(),
                                export_frame,
                                stringToEnum<SensorType>(sensor_type_map, s["sensorType"].Scalar()),
                                update_rate,
                                sensor_blobs, s["frameReferenceLink"].as<std::string>(s["linkName"].Scalar()) });
        }
        catch (YAML::Exception& e)
        {
            printToMessageWindow(e.msg, c2uLogLevel::WARN);
        }

    }
}

void Sensorizer::readFTSensorsFromConfig(const YAML::Node& config)
{
    if (config["forceTorqueSensors"].IsDefined())
    {
        for (const auto& s : config["forceTorqueSensors"])
        {
            bool export_frame = false;

            if (s["exportFrameInURDF"].IsDefined())
            {
                export_frame = s["exportFrameInURDF"].as<bool>();
            }
            std::string sensor_name = s["jointName"].Scalar();
            if (s["sensorName"].IsDefined()) {
                sensor_name = s["sensorName"].Scalar();
            }

            std::string exported_frame_name = sensor_name;
            if (s["exportedFrameName"].IsDefined()) {
                exported_frame_name = s["exportedFrameName"].Scalar();
            }

            std::vector<string> sensor_blobs;
            if (s["sensorBlobs"].IsDefined())
            {
                sensor_blobs = s["sensorBlobs"].as<std::vector<std::string>>();
            }

            ft_sensors.insert(
                {
                    s["jointName"].Scalar(),
                    {
                        s["directionChildToParent"].as<bool>(),
                        s["frame"].Scalar(),
                        sensor_name,
                        s["frameName"].Scalar(),
                        s["linkName"].Scalar(),
                        exported_frame_name,
                        iDynTree::Transform::Identity(),
                        iDynTree::Transform::Identity(),
                        export_frame,
                        sensor_blobs, s["frameReferenceLink"].as<std::string>(s["linkName"].as<std::string>(""))
                    }
                });
        }
    }

}

namespace {
const LinkInfo& sensorLink(const std::map<ComponentId, LinkInfo>& links, const std::string& name) {
    for (const auto& link : links) if (link.second.name == name) return link.second;
    throw std::runtime_error("Unknown sensor reference link: " + name);
}

iDynTree::Transform sensorWorldFrame(const std::map<std::string, ExportedFrameInfo>& frames,
    const std::map<ComponentId, LinkInfo>& links, const std::string& frameName,
    const std::string& referenceName, const std::array<double, 3>& scale) {
    // An empty frame selects the reference link frame, without a datum lookup.
    if (frameName.empty()) return sensorLink(links, referenceName).rootAsm_H_linkFrame;
    const ExportedFrameInfo* match = nullptr;
    auto named = frames.find(frameName);
    if (named != frames.end() && (referenceName.empty() || named->second.frameReferenceLink == referenceName))
        match = &named->second;
    else for (const auto& entry : frames) {
        const auto& frame = entry.second;
        if (frame.cad_frame_name != frameName || (!referenceName.empty() && frame.frameReferenceLink != referenceName)) continue;
        if (match) throw std::runtime_error("Ambiguous sensor frame: " + frameName + "; use its exported name");
        match = &frame;
    }
    if (match) {
        const auto& reference = sensorLink(links, match->frameReferenceLink);
        return reference.rootAsm_H_linkFrame * match->linkFrame_H_additionalFrame * match->additionalTransformation;
    }
    const auto& reference = sensorLink(links, referenceName);
    bool ok;
    iDynTree::Transform part_H_frame, part_H_link;
    std::tie(ok, part_H_frame) = getTransformFromPart(reference.modelhdl, frameName, scale);
    if (!ok) throw std::runtime_error("Missing sensor frame " + frameName + " on " + referenceName);
    std::tie(ok, part_H_link) = getTransformFromPart(reference.modelhdl, reference.link_frame_name, scale);
    if (!ok) throw std::runtime_error("Missing link frame on " + referenceName);
    return reference.rootAsm_H_linkFrame * part_H_link.inverse() * part_H_frame;
}
}

void Sensorizer::assignTransformToFTSensor(const std::map<std::string, ExportedFrameInfo>& frames,
    const std::map<ComponentId, LinkInfo>& links, const std::map<std::string, JointInfo>& joints,
    const std::array<double, 3> scale) {
    for (auto& entry : ft_sensors) {
        const auto joint = joints.find(entry.first);
        if (joint == joints.end()) throw std::runtime_error("Unknown FT sensor joint: " + entry.first);
        auto& sensor = entry.second;
        const auto& parent = links.at(joint->second.parent_link_id);
        const auto& child = links.at(joint->second.child_link_id);
        const auto reference = sensor.frameReferenceLink.empty() ? child.name : sensor.frameReferenceLink;
        const auto world_H_sensor = sensorWorldFrame(frames, links, sensor.frameName, reference, scale);
        sensor.parent_link_H_sensor = parent.rootAsm_H_linkFrame.inverse() * world_H_sensor;
        sensor.child_link_H_sensor = child.rootAsm_H_linkFrame.inverse() * world_H_sensor;
    }
}
std::vector<std::string> Sensorizer::buildFTXMLBlobs()
{
    std::vector<std::string> ft_xml_blobs;

    for (const auto& ft : ft_sensors)
    {
        xmlKeepBlanksDefault(0);
        xmlDocPtr doc = NULL;
        xmlNodePtr root_node = NULL, node = NULL;
        doc = xmlNewDoc(NULL);
        root_node = xmlNewNode(NULL, BAD_CAST "gazebo");

        xmlDocSetRootElement(doc, root_node);

        xmlNewProp(root_node, BAD_CAST "reference", BAD_CAST ft.first.c_str());

        node = xmlNewChild(root_node, NULL, BAD_CAST "sensor", NULL);
        xmlNewProp(node, BAD_CAST "name", BAD_CAST ft.second.sensorName.c_str());
        xmlNewProp(node, BAD_CAST "type", BAD_CAST "force_torque");

        auto node1 = xmlNewChild(node, NULL, BAD_CAST "always_on", BAD_CAST "1");
        node1 = xmlNewChild(node, NULL, BAD_CAST "update_rate", BAD_CAST "100");
        node1 = xmlNewChild(node, NULL, BAD_CAST "force_torque", NULL);

        auto node2 = xmlNewChild(node1, NULL, BAD_CAST "frame", BAD_CAST ft.second.frame.c_str());

        auto& trf = ft.second.child_link_H_sensor;

        if (ft.second.directionChildToParent)
        {
            node2 = xmlNewChild(node1, NULL, BAD_CAST "measure_direction", BAD_CAST "child_to_parent");
        }
        else
        {
            node2 = xmlNewChild(node1, NULL, BAD_CAST "measure_direction", BAD_CAST "parent_to_child");
        }

        std::string pose_xyz_rpy = trf.getPosition().toString() + " " + trf.getRotation().asRPY().toString();
        node1 = xmlNewChild(node, NULL, BAD_CAST "pose", BAD_CAST pose_xyz_rpy.c_str());

        for (auto & blob : ft.second.xmlBlobs)
        {
            xmlNodePtr node_xmlblob = nullptr;

            xmlParseInNodeContext(node, blob.c_str(), blob.size(), 0, &node_xmlblob);

            if (node_xmlblob)
                xmlAddChild(node, node_xmlblob);
        }

        xmlOutputBufferPtr gazebo_doc_buffer = xmlAllocOutputBuffer(NULL);
        xmlNodeDumpOutput(gazebo_doc_buffer, doc, root_node, 0, 1, NULL);
        ft_xml_blobs.push_back(string((char*)xmlBufContent(gazebo_doc_buffer->buffer)));

        xmlOutputBufferClose(gazebo_doc_buffer);

        xmlFreeDoc(doc);

        doc = xmlNewDoc(BAD_CAST "1.0");
        root_node = xmlNewNode(NULL, BAD_CAST "sensor");

        xmlDocSetRootElement(doc, root_node);

        xmlNewProp(root_node, BAD_CAST "name", BAD_CAST ft.second.sensorName.c_str());
        xmlNewProp(root_node, BAD_CAST "type", BAD_CAST "force_torque");
        node = xmlNewChild(root_node, NULL, BAD_CAST "parent", NULL);
        xmlNewProp(node, BAD_CAST "joint", BAD_CAST ft.first.c_str());

        node = xmlNewChild(root_node, NULL, BAD_CAST "force_torque", NULL);
        xmlNewChild(node, NULL, BAD_CAST "frame", BAD_CAST ft.second.frame.c_str());
        if (ft.second.directionChildToParent)
        {
            xmlNewChild(node, NULL, BAD_CAST "measure_direction", BAD_CAST "child_to_parent");
        }
        else
        {
            xmlNewChild(node, NULL, BAD_CAST "measure_direction", BAD_CAST "parent_to_child");
        }
        node = xmlNewChild(root_node, NULL, BAD_CAST "origin", NULL);
        xmlNewProp(node, BAD_CAST "rpy", BAD_CAST trf.getRotation().asRPY().toString().c_str());
        xmlNewProp(node, BAD_CAST "xyz", BAD_CAST trf.getPosition().toString().c_str());

        xmlOutputBufferPtr sensor_doc_buffer = xmlAllocOutputBuffer(NULL);
        xmlNodeDumpOutput(sensor_doc_buffer, doc, root_node, 0, 1, NULL);
        ft_xml_blobs.push_back(string((char*)xmlBufContent(sensor_doc_buffer->buffer)));

        xmlOutputBufferClose(sensor_doc_buffer);

        xmlFreeDoc(doc);
    }

    return ft_xml_blobs;
}

void Sensorizer::assignTransformToSensors(const std::map<std::string, ExportedFrameInfo>& frames,
    const std::map<ComponentId, LinkInfo>& links, const std::array<double, 3> scale) {
    for (auto& sensor : sensors) {
        const auto& target = sensorLink(links, sensor.linkName);
        sensor.transform = target.rootAsm_H_linkFrame.inverse() *
            sensorWorldFrame(frames, links, sensor.frameName, sensor.frameReferenceLink, scale);
    }
}
std::vector<std::string> Sensorizer::buildSensorsXMLBlobs()
{
    std::vector<std::string> xml_blobs;

    for (const auto& s : sensors)
    {
        xmlKeepBlanksDefault(0);
        xmlDocPtr doc = NULL;
        xmlNodePtr root_node = NULL, node = NULL;
        doc = xmlNewDoc(BAD_CAST "1.0");
        root_node = xmlNewNode(NULL, BAD_CAST "gazebo");

        xmlDocSetRootElement(doc, root_node);

        xmlNewProp(root_node, BAD_CAST "reference", BAD_CAST s.linkName.c_str());

        node = xmlNewChild(root_node, NULL, BAD_CAST "sensor", NULL);
        xmlNewProp(node, BAD_CAST "name", BAD_CAST s.sensorName.c_str());

        xmlNewProp(node, BAD_CAST "type", BAD_CAST gazebo_sensor_type_map.at(s.type).c_str());

        xmlNewChild(node, NULL, BAD_CAST "always_on", BAD_CAST "1");
        xmlNewChild(node, NULL, BAD_CAST "update_rate", BAD_CAST to_string(s.updateRate).c_str());

        iDynTree::Transform trf = s.transform;

        string pose = trf.getPosition().toString() + " " + trf.getRotation().asRPY().toString();

        xmlNewChild(node, NULL, BAD_CAST "pose", BAD_CAST pose.c_str());

        for (auto & blob : s.xmlBlobs)
        {
            xmlNodePtr node_xmlblob = nullptr;

            xmlParseInNodeContext(node, blob.c_str(), blob.size(), 0, &node_xmlblob);

            if (node_xmlblob)
                xmlAddChild(node, node_xmlblob);
        }

        xmlOutputBufferPtr doc_buffer = xmlAllocOutputBuffer(NULL);
        xmlNodeDumpOutput(doc_buffer, doc, root_node, 0, 1, NULL);
        xml_blobs.push_back(string((char*)xmlBufContent(doc_buffer->buffer)));

        xmlOutputBufferClose(doc_buffer);
        xmlFreeDoc(doc);

        doc = xmlNewDoc(BAD_CAST "1.0");
        root_node = xmlNewNode(NULL, BAD_CAST "sensor");
        xmlDocSetRootElement(doc, root_node);

        xmlNewProp(root_node, BAD_CAST "name", BAD_CAST s.sensorName.c_str());
        xmlNewProp(root_node, BAD_CAST "type", BAD_CAST sensor_type_map.at(s.type).c_str());
        node = xmlNewChild(root_node, NULL, BAD_CAST "parent", NULL);
        xmlNewProp(node, BAD_CAST "link", BAD_CAST s.linkName.c_str());
        node = xmlNewChild(root_node, NULL, BAD_CAST "origin", NULL);
        xmlNewProp(node, BAD_CAST "rpy", BAD_CAST trf.getRotation().asRPY().toString().c_str());
        xmlNewProp(node, BAD_CAST "xyz", BAD_CAST trf.getPosition().toString().c_str());
        doc_buffer = xmlAllocOutputBuffer(NULL);
        xmlNodeDumpOutput(doc_buffer, doc, root_node, 0, 1, NULL);
        xml_blobs.push_back(string((char*)xmlBufContent(doc_buffer->buffer)));

        xmlOutputBufferClose(doc_buffer);
        xmlFreeDoc(doc);
    }

    return xml_blobs;
}
