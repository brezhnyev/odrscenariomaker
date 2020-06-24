#pragma once

#include "Waypoint.h"

#include <deque>

class Waypath
{
public:
    bool pushWaypoint(Eigen::Vector3f p);
    void popWaypoint();
    void draw();
    void drawWithNames();
    void selectWaypoint(int id);
private:
    std::deque<Waypoint> m_wpoints;
};