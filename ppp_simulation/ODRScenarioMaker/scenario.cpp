#include "scenario.h"
#include <vector>

using namespace std;
using namespace Eigen;


int Scenario::addWaypoint(Vector3f p)
{
    return getActiveChild(1) ? getActiveChild(1)->addChild(new Waypoint(p, 0)) : -1;
}

// get active elements -------------
Actor * Scenario::getActiveActor()
{
    return getActiveChild(0) ? dynamic_cast<Actor*>(getActiveChild(0)) : nullptr;
}

Waypath * Scenario::getActiveWaypath()
{
    return getActiveChild(1) ? dynamic_cast<Waypath*>(getActiveChild(1)) : nullptr;
}

Waypoint * Scenario::getActiveWaypoint()
{
    return getActiveChild(2) ? dynamic_cast<Waypoint*>(getActiveChild(2)) : nullptr;
}

int Scenario::getActiveActorID()
{
    return getActiveChild(0) ? getActiveChild(0)->getID() : -1;
}

int Scenario::getActiveWaypathID()
{
    return getActiveChild(1) ? getActiveChild(1)->getID() : -1;
}

int Scenario::getActiveWaypointID()
{
    return getActiveChild(2) ? getActiveChild(2)->getID() : -1;
}