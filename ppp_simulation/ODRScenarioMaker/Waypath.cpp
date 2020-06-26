#include "Waypath.h"

using namespace Eigen;

void Waypath::draw()
{
    for (auto && wp : m_wpoints) wp.draw();
}

void Waypath::drawWithNames()
{
    for (auto && wp : m_wpoints) wp.drawWithNames();
}

void Waypath::popWaypoint()
{
    if (m_wpoints.empty()) return;

    m_wpoints.pop_back();
}

int Waypath::pushWaypoint(Vector3f p)
{
    m_wpoints.emplace_back(p, 0);
    return m_wpoints.back().getID();
}

void Waypath::selectWaypoint(int id)
{
    // deselect all first:
    for (auto && w : m_wpoints) w.select(false);

    if (id != -1)
    {
        auto it = find_if(m_wpoints.begin(), m_wpoints.end(), [&](Waypoint & wp){ return (wp.getID() == id); });
        if (it != m_wpoints.end())
            it->select(true);
    }
}