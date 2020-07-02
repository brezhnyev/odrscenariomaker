#pragma once

#include "Waypoint.h"

#include <vector>
#include <deque>

class Waypath : public Selectable
{
public:
    Waypath();
    int addWaypoint(Eigen::Vector3f p);
    int delWaypoint();
    void draw() override;
    void drawWithNames() override;
    bool select(int id) override;
    Selectable * getChild(int id) override;

    size_t size() { return m_wpoints.size(); }
    bool getNext(Eigen::Vector3f & pos);
    std::vector<Waypoint> getWaypoints() { return m_wpoints; }
    std::string serialize();

private:
    int m_activeWaypoint;
    std::vector<Waypoint> m_wpoints;
};