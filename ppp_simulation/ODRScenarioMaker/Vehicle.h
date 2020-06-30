#pragma once

#include "Actor.h"

#include <eigen3/Eigen/Eigen>

class Vehicle : public Actor
{
public:
    void setTrf(Eigen::Vector3f pos, float yaw) override
    {
        m_pos = pos;
        m_yaw = yaw;
    }
    void draw() override;
    void drawWithNames() override {};
    bool select(int id) override {};
    Selectable * getChild(int id) override {} // TODO
    Selectable * getActive() override {} // TODO
};