#pragma once

#include <eigen3/Eigen/Eigen>

class Vehicle
{
public:
    void setPosYaw(Eigen::Vector3f pos, float yaw)
    {
        m_pos = pos;
        m_yaw = yaw;
    }
    void draw();

private:
    Eigen::Vector3f m_pos;
    float m_yaw;
};