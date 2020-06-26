#include "scenario.h"

void Scenario::draw()
{
    for (auto && wp : m_waypaths) wp.draw();
}

void Scenario::drawWithNames()
{
    for (auto && wp : m_waypaths) wp.drawWithNames();
}

int Scenario::addWaypath()
{
    m_waypaths.push_back(Waypath());
    m_activeWaypath = m_waypaths.size() - 1;
    return m_waypaths.back().getID();
}

void Scenario::delWaypath(int id)
{
    if (id < m_waypaths.size()) return;
    if (m_waypaths.empty()) return;
    auto it = m_waypaths.begin() + id;
    m_waypaths.erase(it);
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

void Scenario::setActiveWaypath(int id)
{
    if (id < m_waypaths.size()) m_activeWaypath = id;
}

void Scenario::selectWaypoint(int id)
{
    // first deselect
    for (auto && wpath : m_waypaths) wpath.selectWaypoint(-1);
    // then select the id:
    m_waypaths[m_activeWaypath].selectWaypoint(id);
    m_activeWaypoint = id;
}

Waypath * Scenario::getActiveWaypath()
{
    return &m_waypaths[m_activeWaypath];
}

Waypoint * Scenario::getActiveWaypoint()
{
    return m_waypaths[m_activeWaypath].getWaypoint(m_activeWaypoint);
}