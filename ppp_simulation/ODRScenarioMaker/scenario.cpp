#include "scenario.h"

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
    Waypath w;
    m_waypaths[w.getID()] = w; // KB: disadvantage: copy assignment we skip one id
    m_activeWaypath = w.getID();
    return m_activeWaypath;
}

void Scenario::delWaypath(int id)
{
    if (m_waypaths.empty()) return;

    m_waypaths.erase(id);
    m_activeWaypath = 0;
}

int Scenario::addWaypoint(Eigen::Vector3f p)
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
    // first deselect
    for (auto && wpath : m_waypaths) wpath.second.select(-1);
    // then select the id:
    for (auto && wpath : m_waypaths)
    {
        if (wpath.second.select(id))
        {
            m_activeWaypath = wpath.second.getID();
            if (id != m_activeWaypath) m_activeWaypoint = id;
        }
    }
}

Waypath * Scenario::getActiveWaypath()
{
    return &m_waypaths[m_activeWaypath];
}

Waypoint * Scenario::getActiveWaypoint()
{
    return m_waypaths[m_activeWaypath].getWaypoint(m_activeWaypoint);
}