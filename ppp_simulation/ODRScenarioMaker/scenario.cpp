#include "scenario.h"
#include <vector>

using namespace std;
using namespace Eigen;

Scenario::Scenario(const Scenario & other) : Selectable(other)
{
    *this = other;
}

Scenario & Scenario::operator=(const Scenario & other)
{
    Selectable::operator=(other);
    m_rosbagFile = other.m_rosbagFile;
    m_rosbagTopics = other.m_rosbagTopics;
    m_townName = other.m_townName;
    m_rosbagOffset = other.m_rosbagOffset;

    parse([](Selectable * object)
    {
        Waypath * path = dynamic_cast<Waypath*>(object);
        if (path)
        {
            path->updateSmoothPath();
        }
        Vehicle * vehicle = dynamic_cast<Vehicle*>(object);
        if (vehicle)
        {
            vehicle->updatePose();
        }
    });
    return *this;
}

// get active elements -------------
Actor * Scenario::getActiveActor()
{
    return dynamic_cast<Actor*>(getActiveChild(1));
}

Waypath * Scenario::getActiveWaypath()
{
    return dynamic_cast<Waypath*>(getActiveChild(2));
}

Waypoint * Scenario::getActiveWaypoint()
{
    return dynamic_cast<Waypoint*>(getActiveChild(3));
}

int Scenario::getActiveActorID()
{
    return getActiveChild(1) ? getActiveChild(1)->getID() : -1;
}

int Scenario::getActiveWaypathID()
{
    return getActiveChild(2) ? getActiveChild(2)->getID() : -1;
}

int Scenario::getActiveWaypointID()
{
    return getActiveChild(3) ? getActiveChild(3)->getID() : -1;
}