#include "scenario.h"

using namespace std;
using namespace Eigen;

Scenario::Scenario() : m_canvas("../data/Town02.jpg", QRect(-27, 92, 239, 237)) {}

void Scenario::init()
{
    m_canvas.init();
}

void Scenario::draw()
{
    m_canvas.draw();
    for (auto & child : m_children) child.second->draw();
}

void Scenario::drawWithNames()
{
    m_canvas.drawWithNames();
    for (auto & child : m_children) child.second->drawWithNames();
}

// adding ---------------
int Scenario::addVehicle()
{
    return addChild(new Vehicle);
}

int Scenario::addWaypath()
{
    return getActiveChild(0) ? getActiveChild(0)->addChild(new Waypath) : -1;
}

int Scenario::addWaypoint(Vector3f p)
{
    return getActiveChild(1) ? getActiveChild(1)->addChild(new Waypoint(p, 0)) : -1;
}

// deleting -------------
int Scenario::delActor(int id)
{
    return delChild(id);
}

int Scenario::delWaypath(int id)
{
    return getActiveChild(0) ? getActiveChild(0)->delChild(id) : -1;
}

int Scenario::delWaypoint(int id)
{
    return getActiveChild(1) ? getActiveChild(1)->delChild(id) : -1;
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