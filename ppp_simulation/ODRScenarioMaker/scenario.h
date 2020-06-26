#pragma once

#include "Waypath.h"

#include <eigen3/Eigen/Eigen>

#include <vector>

class Scenario
{
public:
    Scenario() : m_activeWaypath(0), m_activeWaypoint(0) {}
    void draw();
    void drawWithNames();

    int addWaypath();
    int addWaypoint(Eigen::Vector3f);
    void delWaypath(int id);
    void delWaypoint();
    void select(int id);
    Waypoint * getActiveWaypoint();
    Waypath  * getActiveWaypath();

public:
    int m_activeWaypath;
    int m_activeWaypoint;

private:
    std::map<int, Waypath>    m_waypaths;
};  