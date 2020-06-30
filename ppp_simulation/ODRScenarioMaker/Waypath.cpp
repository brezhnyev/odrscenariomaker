#include "Waypath.h"

#include <sstream>

using namespace Eigen;
using namespace std;

Waypath::Waypath() {}

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
    auto s = m_wpoints.size();
    m_wpoints.emplace_back(p, 0);
    return m_wpoints.back().getID();
}

bool Waypath::select(int id)
{
    // otherwise selet the waypoint only:
    m_activeWaypoint = 0;
    m_selected = false;

    for (auto && wp : m_wpoints)
    {
        if (wp.select(id))
        {
            m_activeWaypoint = wp.getID();
        }
    }

    // if path is selected then select all points on the path:
    if (id == m_id)
    {
        for (auto && p : m_wpoints) p.select(true);
        m_selected = true;
    }

    return m_selected || m_activeWaypoint;
}

bool Waypath::getNext(Vector3f & pos)
{
    const int R = 5;
    // find the nearest next point:
    float dist = 100000;
    int index = 0;
    for (int i = 0; i < m_wpoints.size(); ++i)
    {
        auto d = (pos - m_wpoints[i].getPosition()).norm();
        if (d < dist)
        {
            index = i;
            dist = d;
        }
    }
    if (index != m_wpoints.size() - 1)
    {
        auto v1 = (m_wpoints[index+1].getPosition() - m_wpoints[index].getPosition()).normalized();
        auto v2 = (m_wpoints[index].getPosition() - pos).normalized();
        if (v1.dot(v2) < 0) ++index;
    }

    dist = 0;
    for (int i=index; i < m_wpoints.size(); ++i)
    {
        Vector3f v = m_wpoints[i].getPosition() - pos;
        float N = v.norm();
        if (dist + N > R)
        {
            pos = pos + v*((R - dist)/N);
            return true;
        }
        else
        {
            pos = pos + v;
            dist += v.norm();
        }
    }
    return false;
}

string Waypath::serialize()
{
    Eigen::Vector3f v; float val; int counter = 0;
    stringstream ss;
    for (auto && point : m_wpoints)
    {
        auto p = point.getPosition();
        ss << p.x() << " " << p.y() << " " << p.z() << " ";
    }
    return ss.str();
}