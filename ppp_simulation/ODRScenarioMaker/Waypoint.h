#pragma once

#include "Selectable.h"

#include <atomic>

#include <eigen3/Eigen/Eigen>


class Waypoint : public Selectable
{
public:
    Waypoint(Eigen::Vector3f pos, float speed);
    void draw() override;
    void drawWithNames() override;
    void drawGeometry() override;
    std::string getType() const override { return "Waypoint"; }

    Eigen::Vector3f getPosition() { return m_pos; }
    void  setPosition(Eigen::Vector3f val) { m_pos = val; }
    void flipY() { m_pos[1] = -m_pos[1]; }
    float getSpeed() { return m_speed; }
    float setSpeed(float speed) { m_speed = speed; }

private:
    Eigen::Vector3f     m_pos;
    float               m_speed;
};
