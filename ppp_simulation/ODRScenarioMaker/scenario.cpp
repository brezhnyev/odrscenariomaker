#include "scenario.h"

using namespace std;
using namespace Eigen;

Scenario::Scenario() : m_activeActor(0) {}

void Scenario::draw()
{
    for (auto && wp : m_actors) wp.second->draw();
}

void Scenario::drawWithNames()
{
    for (auto && wp : m_actors) wp.second->drawWithNames();
}
// adding ---------------
int Scenario::addVehicle()
{
    Vehicle *  vehicle = new Vehicle();
    m_actors[vehicle->getID()] = vehicle;
    m_activeActor = vehicle->getID();
    return m_activeActor;
}

int Scenario::addWaypath()
{
    return m_actors[m_activeActor]->addWaypath();
}

int Scenario::addWaypoint(Vector3f p)
{
    return m_actors[m_activeActor]->addWaypoint(p);
}

// deleting -------------
void Scenario::delActor(int id)
{
    if (m_actors.empty()) return;
    delete m_actors[id];
    m_actors.erase(id);
    m_activeActor = 0;
}

void Scenario::delWaypath(int id)
{
    m_actors[m_activeActor]->delWaypath(id);
}

void Scenario::delWaypoint()
{
    m_actors[m_activeActor]->delWaypoint();
}

void Scenario::select(int id)
{
    m_activeActor = 0;
    // then select the id:
    for (auto && actor : m_actors)
    {
        if (actor.second->select(id))
        {
            m_activeActor = actor.second->getID();
        }
    }
}

Selectable * Scenario::getSelectable(int id)
{
    Selectable * selection = nullptr;
    for (auto actor : m_actors)
    {
        if (actor.second->getID() == id) return actor.second;
        selection = actor.second->getChild(id);
        if (selection) return selection;
    }
    return selection;
}

Waypath * Scenario::getActiveWaypath()
{
    return m_actors[m_activeActor]->getActiveWaypath();
}

int Scenario::getActiveWaypathID()
{
    return m_actors[m_activeActor]->getActiveWaypath()->getID();
}

int Scenario::getActiveActorID()
{
    return m_actors[m_activeActor]->getID();
}