#pragma once

#include "Actor.h"
#include "Waypath.h"

#include <eigen3/Eigen/Eigen>

class Vehicle : public Actor
{
public:
    Vehicle(std::string name = "vehicle.audi.a2") : Actor(name) {}
    void setTrf(Eigen::Vector3f pos, float yaw) override
    {
        m_pos = pos;
        m_yaw = yaw;
    }
    void setTrf(float x, float y, float z, float yaw) override
    {
        m_pos = Eigen::Vector3f(x,y,z);
        m_yaw = yaw;
    }
    void draw() override;
    std::string getType() const override { return "Vehicle"; }
    std::string getName() const override { return m_name; }
};