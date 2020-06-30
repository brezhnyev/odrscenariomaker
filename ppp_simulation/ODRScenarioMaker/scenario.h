#pragma once

#include "Waypath.h"

#pragma once

#include "Actor.h"

#include <eigen3/Eigen/Eigen>

#include <vector>

class Scenario
{
public:
    Scenario();
    void draw();
    void drawWithNames();

    int addWaypath();
    int addWaypoint(Eigen::Vector3f);
    void delWaypath(int id);
    void delWaypoint();
    void select(int id);

    Selectable * getSelectable(int id);
    Waypath *    getActiveWaypath();

public:
    int m_activeWaypath;

private:
    std::map<int, Waypath>    m_waypaths;
};  