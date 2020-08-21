#include "Serializer.h"

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
        for (auto && child : object->children()) serialize_yaml(parent, child.second);
    }
    else if (object->getType() == "Vehicle")
    {
        Vehicle * vehicle = dynamic_cast<Vehicle*>(object);
        YAML::Node node;
        node["type"] = "Vehicle";
        node["id"] = vehicle->getID();
        node["name"] = vehicle->getName();
        YAML::Node color;
        color["r"] = vehicle->m_color[0];
        color["g"] = vehicle->m_color[1];
        color["b"] = vehicle->m_color[2];
        node["color"] = color;
        YAML::Node waypaths;
        node["waypaths"] = waypaths;
        for (auto && child : object->children()) serialize_yaml(waypaths, child.second);
        parent.push_back(node);
    }
    else if (object->getType() == "Waypath")
    {
        Waypath * waypath = dynamic_cast<Waypath*>(object);
        YAML::Node node;
        node["type"] = "Waypath";
        YAML::Node waypoints;
        node["waypoints"] = waypoints;
        for (auto && child : object->children()) serialize_yaml(waypoints, child.second);
        parent.push_back(node);
    }
    else if (object->getType() == "Waypoint")
    {
        Waypoint * waypoint = dynamic_cast<Waypoint*>(object);
        YAML::Node node;
        node["type"] = "Waypoint";
        YAML::Node location;
        location["x"] = waypoint->getPosition().x();
        location["y"] = waypoint->getPosition().y();
        location["z"] = waypoint->getPosition().z();
        node["location"] = location;
        parent.push_back(node);
    }
}

Scenario Serializer::deserialize_yaml(const std::string & data)
{
    YAML::Node root = YAML::Load(data);
    Scenario object;

    deserialize_yaml(root, object);

    return object;
}

void Serializer::deserialize_yaml(YAML::Node node, Selectable & object)
{
    for (auto it = node.begin(); it != node.end(); ++it)
    {
        auto child = *it;
        if (child["type"].as<string>() == "Vehicle")
        {
            Vehicle * vehicle = new Vehicle();
            object.addChild(vehicle);
            vehicle->m_name = child["name"].as<string>();
            vehicle->setID(child["id"].as<int>());
            auto color = child["color"];
            int r = color["r"].as<int>();
            int g = color["g"].as<int>();
            int b = color["b"].as<int>();
            vehicle->m_color = Eigen::Vector3i(r,g,b);
            if (!child["waypaths"].IsNull())
                deserialize_yaml(child["waypaths"], *vehicle);
        }
        if (child["type"].as<string>() == "Waypath")
        {
            Waypath * waypath = new Waypath();
            object.addChild(waypath);
            if (!child["waypoints"].IsNull())
                deserialize_yaml(child["waypoints"], *waypath);
        }
        if (child["type"].as<string>() == "Waypoint")
        {
            auto location = child["location"];
            float x = location["x"].as<float>();
            float y = location["y"].as<float>();
            float z = location["z"].as<float>();
            Waypoint * waypoint = new Waypoint(Eigen::Vector3f(x,y,z), 0);
            object.addChild(waypoint);
        }
    }
}