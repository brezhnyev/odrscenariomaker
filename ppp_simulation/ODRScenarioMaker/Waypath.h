#pragma once

#include "Waypoint.h"

#include <vector>
#include <deque>

class Waypath : public BaseObject
{
public:
    int pushWaypoint(Eigen::Vector3f p);
    void popWaypoint();
    void draw();
    void drawWithNames();
    bool select(int id);
    size_t size() { return m_wpoints.size(); }
    Waypoint * getWaypoint(int id)
    { 
        auto it = find_if(m_wpoints.begin(), m_wpoints.end(), [id](Waypoint & p){ return (id == p.getID()); });
        if (it != m_wpoints.end()) return &(*it);
        return nullptr;
    }
private:
    std::vector<Waypoint> m_wpoints;
};