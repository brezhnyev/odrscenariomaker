#include "Serializer.h"
#include "Camera.h"

#include <eigen3/Eigen/Eigen>

#include <sstream>
#include <iostream>

using namespace std;

string Serializer::serialize_yaml(Selectable * object)
{
    YAML::Node root;
    serialize_yaml(root, object);

    stringstream ss;
    ss << root;

    return ss.str();
}

void Serializer::serialize_yaml(YAML::Node & parent, Selectable * object)
{
    if (object->getType() == "Scenario")
    {
        Scenario * scenario = dynamic_cast<Scenario*>(object);
        parent["type"] = "Scenario";
        parent["townname"] = scenario->getTownName();
        parent["rosbagfile"] = scenario->getRosbagFile();
        for (auto && rostopic : scenario->getRosbagTopics())
        {
            YAML::Node topic;
            topic["topic"] = rostopic;
            parent["rosbagtopics"].push_back(topic);
        }
        parent["rosbagoffset"] = scenario->getRosbagOffset();
        YAML::Node actors;
        parent["actors"] = actors;
        for (auto && child : object->children()) serialize_yaml(actors, child.second);
    }
    else
    {
        YAML::Node node;
        node["type"] = object->getType();
        if (object->getType() == "Vehicle")
        {
            Vehicle * vehicle = dynamic_cast<Vehicle*>(object);
            YAML::Node color;
            node["name"] = vehicle->getName();
            color["r"] = vehicle->m_color[0];
            color["g"] = vehicle->m_color[1];
            color["b"] = vehicle->m_color[2];
            node["color"] = color;
            YAML::Node waypaths;
            node["facilities"] = waypaths;
            for (auto && child : object->children()) serialize_yaml(waypaths, child.second);
            parent.push_back(node);
        }
        else if (object->getType() == "Waypath")
        {
            Waypath * waypath = dynamic_cast<Waypath*>(object);
            YAML::Node waypoints;
            node["waypoints"] = waypoints;
            for (auto && child : object->children()) serialize_yaml(waypoints, child.second);
            parent.push_back(node);
        }
        else if (object->getType() == "Camera")
        {
            Camera * camera = dynamic_cast<Camera*>(object);
            YAML::Node location;
            location["x"] = camera->getPos().x();
            location["y"] = camera->getPos().y();
            location["z"] = camera->getPos().z();
            node["location"] = location;
            YAML::Node orientation;
            orientation["roll"] = camera->getOri().x();
            orientation["pitch"] = camera->getOri().y();
            orientation["yaw"] = camera->getOri().z();
            node["orientation"] = orientation;
            parent.push_back(node);
        }
        else if (object->getType() == "Waypoint")
        {
            Waypoint * waypoint = dynamic_cast<Waypoint*>(object);
            YAML::Node location;
            location["x"] = waypoint->getPosition().x();
            location["y"] = waypoint->getPosition().y();
            location["z"] = waypoint->getPosition().z();
            node["location"] = location;
            node["speed"] = waypoint->getSpeed();
            parent.push_back(node);
        }
    }
}

Scenario Serializer::deserialize_yaml(const std::string & data)
{
    YAML::Node root = YAML::Load(data);
    Scenario scenario;

    if (root["type"].as<string>() == "Scenario")
    {
        scenario.setTownName(root["townname"].as<string>());
        scenario.setRosbagFile(root["rosbagfile"].as<string>());
        auto topics = root["rosbagtopics"];
        vector<string> rostopics;
        for (auto && topic : topics)
        {
            rostopics.push_back(topic["topic"].as<string>());
        }
        scenario.setRosbagTopics(rostopics);
        scenario.setRosbagOffset(root["rosbagoffset"].as<float>());
        deserialize_yaml(root["actors"], scenario);
    }

    return scenario;
}

void Serializer::deserialize_yaml(YAML::Node node, Selectable & object)
{
    for (auto it = node.begin(); it != node.end(); ++it)
    {
        auto child = *it;
        if (child["type"].as<string>() == "Vehicle")
        {
            Vehicle * vehicle = new Vehicle(&object);
            object.addChild(vehicle);
            vehicle->setName(child["name"].as<string>());
            auto color = child["color"];
            int r = color["r"].as<int>();
            int g = color["g"].as<int>();
            int b = color["b"].as<int>();
            vehicle->m_color = Eigen::Vector3i(r,g,b);
            if (!child["facilities"].IsNull())
                deserialize_yaml(child["facilities"], *vehicle);
        }
        if (child["type"].as<string>() == "Waypath")
        {
            Waypath * waypath = new Waypath(&object);
            object.addChild(waypath);
            if (!child["waypoints"].IsNull())
                deserialize_yaml(child["waypoints"], *waypath);
        }
        if (child["type"].as<string>() == "Camera")
        {
            Camera * camera = new Camera(&object);
            auto location = child["location"];
            float x = location["x"].as<float>();
            float y = location["y"].as<float>();
            float z = location["z"].as<float>();
            camera->setPos(Eigen::Vector3f(x,y,z));
            auto orientation = child["orientation"];
            float roll = orientation["roll"].as<float>();
            float pitch = orientation["pitch"].as<float>();
            float yaw = orientation["yaw"].as<float>();
            camera->setOri(Eigen::Vector3f(roll,pitch,yaw));
            object.addChild(camera);
        }
        if (child["type"].as<string>() == "Waypoint")
        {
            auto location = child["location"];
            float x = location["x"].as<float>();
            float y = location["y"].as<float>();
            float z = location["z"].as<float>();
            auto speed = child["speed"];
            Waypoint * waypoint = new Waypoint(Eigen::Vector3f(x,y,z), speed.as<float>(), &object);
            object.addChild(waypoint);
        }
    }
}