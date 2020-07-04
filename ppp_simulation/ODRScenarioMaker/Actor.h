#pragma once

#include "Selectable.h"

#include <eigen3/Eigen/Eigen>

class Waypath;

class Actor : public Selectable
{
public:
    Actor(std::string name) : m_name(name) {}
    virtual ~Actor() = 0;
    virtual void setTrf(Eigen::Vector3f pos, float yaw) {};
    virtual void setTrf(float x, float y, float z, float yaw) {}
    std::string getName() { return m_name; }
    std::string setName(std::string name) { m_name = name; }

    std::string m_name; // Due to crash in the ActorProps this made public
protected:
    Eigen::Vector3f m_pos;
    float m_yaw;
};