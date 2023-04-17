#include "Serializer.h"
#include "Camera.h"
#include "Waypoint.h"
#include "Waypath.h"
#include "Vehicle.h"
#include "Walker.h"

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
        Actor * actor = dynamic_cast<Actor*>(object);
        if (actor)
        {
            YAML::Node color;
            node["name"] = actor->get_name();
            color["r"] = actor->get_color()[0];
            color["g"] = actor->get_color()[1];
            color["b"] = actor->get_color()[2];
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
            location["x"] = camera->get_pos().x();
            location["y"] = camera->get_pos().y();
            location["z"] = camera->get_pos().z();
            node["location"] = location;
            YAML::Node orientation;
            orientation["roll"] = camera->get_ori().x();
            orientation["pitch"] = camera->get_ori().y();
            orientation["yaw"] = camera->get_ori().z();
            node["orientation"] = orientation;
            parent.push_back(node);
        }
        else if (object->getType() == "Waypoint")
        {
            Waypoint * waypoint = dynamic_cast<Waypoint*>(object);
            YAML::Node location;
            location["x"] = waypoint->get_pos().x();
            location["y"] = waypoint->get_pos().y();
            location["z"] = waypoint->get_pos().z();
            node["location"] = location;
            node["speed"] = waypoint->get_speed();
            parent.push_back(node);
        }
    }
}

void Serializer::deserialize_yaml(Scenario & scenario, const std::string & data)
{
    YAML::Node root = YAML::Load(data);

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
}

void Serializer::deserialize_yaml(YAML::Node node, Selectable & object)
{
    for (auto it = node.begin(); it != node.end(); ++it)
    {
        auto child = *it;
        Actor * actor = nullptr;
        if (child["type"].as<string>() == "Vehicle") actor = new Vehicle(&object);
        if (child["type"].as<string>() == "Walker")  actor = new Walker(&object);
        if (actor)
        {
            actor->set_name(child["name"].as<string>());
            auto color = child["color"];
            int r = color["r"].as<int>();
            int g = color["g"].as<int>();
            int b = color["b"].as<int>();
            actor->set_color(Eigen::Vector3i(r,g,b));
            if (!child["facilities"].IsNull())
                deserialize_yaml(child["facilities"], *actor);
            actor->updatePose();
        }
        if (child["type"].as<string>() == "Waypath")
        {
            Waypath * waypath = new Waypath(&object);
            if (!child["waypoints"].IsNull())
                deserialize_yaml(child["waypoints"], *waypath);
            waypath->updateSmoothPath();
        }
        if (child["type"].as<string>() == "Camera")
        {
            Camera * camera = new Camera(&object);
            auto location = child["location"];
            float x = location["x"].as<float>();
            float y = location["y"].as<float>();
            float z = location["z"].as<float>();
            camera->set_pos(Eigen::Vector3f(x,y,z));
            auto orientation = child["orientation"];
            float roll = orientation["roll"].as<float>();
            float pitch = orientation["pitch"].as<float>();
            float yaw = orientation["yaw"].as<float>();
            camera->set_ori(Eigen::Vector3f(roll,pitch,yaw));
        }
        if (child["type"].as<string>() == "Waypoint")
        {
            auto location = child["location"];
            float x = location["x"].as<float>();
            float y = location["y"].as<float>();
            float z = location["z"].as<float>();
            auto speed = child["speed"];
            Waypoint * waypoint = new Waypoint(Eigen::Vector3f(x,y,z), speed.as<float>(), &object);
        }
    }
}