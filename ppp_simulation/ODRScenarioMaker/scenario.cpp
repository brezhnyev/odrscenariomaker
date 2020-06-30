#include "scenario.h"

using namespace std;
using namespace Eigen;

Scenario::Scenario() : m_activeWaypath(0) {}

void Scenario::draw()
{
    for (auto && wp : m_waypaths) wp.second.draw();
}

void Scenario::drawWithNames()
{
    for (auto && wp : m_waypaths) wp.second.drawWithNames();
}

int Scenario::addWaypath()
{
    Waypath wp;
    m_waypaths[wp.getID()] = wp;
    m_activeWaypath = wp.getID();
    return m_activeWaypath;
}

void Scenario::delWaypath(int id)
{
    if (m_waypaths.empty()) return;

    m_waypaths.erase(id);
    m_activeWaypath = 0;
}

int Scenario::addWaypoint(Vector3f p)
{
    int id = m_waypaths[m_activeWaypath].pushWaypoint(p);
    return id;
}

void Scenario::delWaypoint()
{
    m_waypaths[m_activeWaypath].popWaypoint();
}

void Scenario::select(int id)
{
    m_activeWaypath = 0;
    // then select the id:
    for (auto && wpath : m_waypaths)
    {
        if (wpath.second.select(id))
        {
            m_activeWaypath = wpath.second.getID();
        }
    }
}

Selectable * Scenario::getSelectable(int id)
{
    if (!m_activeWaypath) return nullptr;

    auto point = m_waypaths[m_activeWaypath].getChild(id);
    if (point) return point;

    return &m_waypaths[m_activeWaypath];
}

Waypath * Scenario::getActiveWaypath()
{
    return &m_waypaths[m_activeWaypath];
}