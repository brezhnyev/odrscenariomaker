#pragma once

#include "Waypoint.h"

#include <vector>
#include <deque>

class Waypath : public Selectable
{
public:
    Waypath();
    int pushWaypoint(Eigen::Vector3f p);
    void popWaypoint();
    void draw() override;
    void drawWithNames() override;
    bool select(int id) override;
    Waypoint * getChild(int id) override
    { 
        auto it = find_if(m_wpoints.begin(), m_wpoints.end(), [id](Waypoint & p){ return (id == p.getID()); });
        if (it != m_wpoints.end()) return &(*it);
        return nullptr;
    }
    Waypoint * getActive() override { return &m_wpoints[m_activeWaypoint]; }

    size_t size() { return m_wpoints.size(); }
    bool getNext(Eigen::Vector3f & pos);
    std::vector<Waypoint> getWaypoints() { return m_wpoints; }
    std::string serialize();

private:
    int m_activeWaypoint;
    std::vector<Waypoint> m_wpoints;
};