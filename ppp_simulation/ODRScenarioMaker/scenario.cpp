#include "scenario.h"

using namespace std;
using namespace Eigen;

// adding ---------------
int Scenario::addVehicle()
{
    Vehicle * vehicle = new Vehicle();
    return addChild(vehicle);
}

int Scenario::addWaypath()
{
    Waypath * path = new Waypath();
    return m_children[m_activeChild]->addChild(path);
}

int Scenario::addWaypoint(Vector3f p)
{
    Waypoint * point = new Waypoint(p, 0);
    return getActiveChild()->getActiveChild()->addChild(point);
}

// deleting -------------
void Scenario::delActor(int id)
{
    delChild(id);
}

void Scenario::delWaypath(int id)
{
    getActiveChild()->delChild(id);
}

void Scenario::delWaypoint(int id)
{
    getActiveChild()->getActiveChild()->delChild(id);
}


// get active elements -------------

Actor * Scenario::getActiveActor()
{
    return dynamic_cast<Actor*>(getActiveChild());
}

Waypath * Scenario::getActiveWaypath()
{
    return dynamic_cast<Waypath*>(getActiveChild()->getActiveChild());
}

Waypoint * Scenario::getActiveWaypoint()
{
    return dynamic_cast<Waypoint*>(getActiveChild()->getActiveChild()->getActiveChild());
}

int Scenario::getActiveActorID()
{
    return getActiveChild()->getID();
}

int Scenario::getActiveWaypathID()
{
    return getActiveChild()->getActiveChild()->getID();
}

int Scenario::getActiveWaypointID()
{
    return getActiveChild()->getActiveChild()->getActiveChild()->getID();
}