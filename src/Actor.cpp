#include "Actor.h"
#include "scenario.h"
#include "Waypath.h"
#include "Camera.h"

#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace Eigen;


void Actor::updatePose()
{
    Waypath * firstWaypath = getFirstWaypath();
    if (!firstWaypath)
        return;
    set_pos(firstWaypath->getStartingPosition());
    Vector3f dir = firstWaypath->getStartingDirection();
    float pitch = asin(dir[2]/dir.norm())*RAD2DEG;
    float yaw = atan2(dir[1], dir[0])*RAD2DEG;
    set_ori(Vector3f(0,pitch,yaw));
}

std::string Actor::colorToString()
{
    return to_string(m_color[0]) + "," + to_string(m_color[1]) + "," + to_string(m_color[2]);
}

Eigen::Vector3i Actor::stringToColor()
{
    return Eigen::Vector3i(0,0,0);
}

Waypath * Actor::getFirstWaypath()
{
    for (auto && c : children())
        if (c.second->getType() == "Waypath")
            return dynamic_cast<Waypath*>(c.second);
    return nullptr;
}

Waypoint * Actor::getFirstWaypoint()
{
    Waypath * wpath = getFirstWaypath();
    if (wpath && !wpath->children().empty())
        return dynamic_cast<Waypoint*>(wpath->children().begin()->second);
    return nullptr;
}

void Actor::to_yaml(YAML::Node & parent)
{
    YAML::Node node;
    node["type"] = getType();
    YAML::Node color;
    node["name"] = m_name;
    color["r"] = m_color[0];
    color["g"] = m_color[1];
    color["b"] = m_color[2];
    node["color"] = color;
    YAML::Node waypaths;
    node["components"] = waypaths;
    Selectable::to_yaml(waypaths);
    parent.push_back(node);
}

void Actor::from_yaml(const YAML::Node & node)
{
    m_name = node["name"].as<string>();
    auto color = node["color"];
    int r = color["r"].as<int>();
    int g = color["g"].as<int>();
    int b = color["b"].as<int>();
    m_color = Eigen::Vector3i(r,g,b);
    auto components = node["components"];
    for (auto it = components.begin(); it != components.end(); ++it)
    {
        auto child = *it;
        if (child["type"].as<string>() == "Waypath")
        {
            Waypath * waypath = new Waypath(this);
            waypath->from_yaml(child);
        }
        else if (child["type"].as<string>() == "Camera")
        {
            Camera * camera = new Camera(this);
            camera->from_yaml(child);
        }
    }
    updatePose();
}