#include "Waypath.h"

#include <sstream>

using namespace Eigen;
using namespace std;

int Waypath::delChild(int id) // id is dummy parameter for Waypath
{
    if (m_children.empty()) return -1;

    id = m_children.rbegin()->first;
    m_children.erase(id);
    if (m_children.empty()) m_activeChild = -1;
    else m_activeChild = 0;

    return id;
}

bool Waypath::getNext(Vector3f & pos)
{
    const int R = 5;
    // find the nearest next point:
    float dist = 100000;
    int index = 0;
    vector<Waypoint*> wpoints;
    for (auto && child : m_children) wpoints.push_back(dynamic_cast<Waypoint*>(child.second));

    for (int i = 0; i < wpoints.size(); ++i)
    {
        auto d = (pos - wpoints[i]->getPosition()).norm();
        if (d < dist)
        {
            index = i;
            dist = d;
        }
    }
    if (index != wpoints.size() - 1)
    {
        auto v1 = (wpoints[index+1]->getPosition() - wpoints[index]->getPosition()).normalized();
        auto v2 = (wpoints[index]->getPosition() - pos).normalized();
        if (v1.dot(v2) < 0) ++index;
    }

    dist = 0;
    for (int i=index; i < wpoints.size(); ++i)
    {
        Vector3f v = wpoints[i]->getPosition() - pos;
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
    for (auto && child : m_children)
    {
        auto p = dynamic_cast<Waypoint*>(child.second)->getPosition();
        ss << p.x() << " " << p.y() << " " << p.z() << " ";
    }
    return ss.str();
}