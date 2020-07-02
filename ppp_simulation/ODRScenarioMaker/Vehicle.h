#pragma once

#include "Actor.h"
#include "Waypath.h"

#include <eigen3/Eigen/Eigen>

class Vehicle : public Actor
{
public:
    Vehicle() : m_activeWaypath(-1) {}
    void setTrf(Eigen::Vector3f pos, float yaw) override
    {
        m_pos = pos;
        m_yaw = yaw;
    }
    void draw() override;
    void drawWithNames() override;
    bool select(int id) override;
    Selectable * getChild(int id) override;
    int addWaypath() override;
    int addWaypoint(Eigen::Vector3f) override;
    void delWaypath(int id) override;
    void delWaypoint() override;
    Waypath * getActiveWaypath() override;

private:
    int                       m_activeWaypath;
    std::map<int, Waypath>    m_waypaths;
};