#pragma once

#include "Waypoint.h"

#include <deque>

class Waypath
{
public:
    Waypath() { ++s_counter; }
    bool pushWaypoint(Eigen::Vector3f p);
    void popWaypoint();
    void draw();
    void drawWithNames();
    void selectWaypoint(int id);
    size_t size() { return m_wpoints.size(); }
    static int counter() { return s_counter; }
private:
    std::deque<Waypoint> m_wpoints;
    static int s_counter;
};