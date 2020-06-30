#pragma once

#include "Selectable.h"

#include <eigen3/Eigen/Eigen>

class Actor : public Selectable
{
    virtual void setTrf(Eigen::Vector3f pos, float yaw) = 0;

protected:
    Eigen::Vector3f m_pos;
    float m_yaw;
};