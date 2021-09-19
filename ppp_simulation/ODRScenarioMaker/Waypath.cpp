#include "Waypath.h"

#include <sstream>
#include <iostream>

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

bool Waypath::getNext(Vector3f & pos, float & targetSpeed)
{
    // find the nearest next point:
    float dist = 100000;
    int index = 0;
    vector<Waypoint*> wpoints;
    for (auto && child : m_children) wpoints.push_back(dynamic_cast<Waypoint*>(child.second));

    if ((pos - wpoints.back()->getPosition()).norm() < 1)
        return false;

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
        Vector3f v1 = (wpoints[index+1]->getPosition() - wpoints[index]->getPosition()).normalized(); // motion direction
        Vector3f v2 = (wpoints[index]->getPosition() - pos).normalized(); // direction to the next
        if (v1.dot(v2) < 0) ++index;
        // otherwise next is the nearest
    }
    if (0 == index) ++index;
    // now index is guaranteed the NEXT waypoint
    Vector3f v1 = wpoints[index]->getPosition() - pos; // distance to next
    Vector3f v2 = pos - wpoints[index-1]->getPosition(); // distance from previous
    float f = v2.norm()/(v1.norm() + v2.norm());
    targetSpeed = (1 - f)*wpoints[index-1]->getSpeed() + f*wpoints[index]->getSpeed();

    pos = wpoints[index]->getPosition();
    return true;
}

string Waypath::serialize() const
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