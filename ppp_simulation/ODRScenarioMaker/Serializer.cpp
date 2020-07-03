#include "Serializer.h"
#include "scenario.h"

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
        node["name"] = vehicle->getName();
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
        YAML::Node x, y, z;
        x["x"] = waypoint->getPosition().x();
        y["y"] = waypoint->getPosition().y();
        z["z"] = waypoint->getPosition().z();
        location.push_back(x);
        location.push_back(y);
        location.push_back(z);
        node["location"] = location;
        parent.push_back(node);
    }
}