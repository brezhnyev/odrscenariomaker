#include "Waypath.h"

using namespace Eigen;

int Waypath::s_counter = 0;

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

bool Waypath::pushWaypoint(Vector3f p)
{
    // TODO: find out if the waypoint is already there
    Waypoint wp(p, 0, m_wpoints.size());
    m_wpoints.push_back(wp);
}

void Waypath::selectWaypoint(int id)
{
    // deselect all first:
    for (auto && w : m_wpoints) w.select(false);

    if (id != -1)
        m_wpoints[id].select(true);
}