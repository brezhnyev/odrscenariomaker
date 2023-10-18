#include "Actor.h"
#include "scenario.h"
#include "Waypath.h"

#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace Eigen;

Actor::~Actor() {} // pure virtual destructor is not allowed to be inline, so place it here

void Actor::updatePose()
{
    if (m_children.empty())
        return;
    Waypath * firstWaypath = static_cast<Waypath*>(m_children.begin()->second);
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