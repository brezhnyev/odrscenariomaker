#pragma once

#include "globals.h"
#include "Selectable.h"

#include <eigen3/Eigen/Eigen>

class Actor : public Selectable
{
public:
    Actor(const std::string & name, Selectable * parent) : Selectable(parent), m_name(name) {}
    virtual ~Actor() = 0;
    void setTrf(Eigen::Vector3f pos, Eigen::Vector3f ori)
    {
        m_pos = pos;
        m_ori = ori;
    }
    void setTrf(float x, float y, float z, float roll, float pitch, float yaw)
    {
        m_pos = Eigen::Vector3f(x,y,z);
        m_ori = Eigen::Vector3f(roll, pitch, yaw);
    }
    void updatePose();

    ADDVAR(protected, std::string, name, "");
    ADDVAR(protected, Eigen::Vector3f, pos, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    ADDVAR(protected, Eigen::Vector3f, ori, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    ADDVAR(protected, Eigen::Vector3f, bbox, Eigen::Vector3f(2.0f, 1.0f, 1.0f));
    ADDVAR(protected, Eigen::Vector3i, color, Eigen::Vector3i(50, 100, 150));

    std::string colorToString();
    Eigen::Vector3i stringToColor();

};