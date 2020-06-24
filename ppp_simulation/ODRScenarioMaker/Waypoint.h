#pragma once

#include <eigen3/Eigen/Eigen>

class Waypoint
{
public:
    Waypoint(Eigen::Vector3f pos, float speed, int id) : m_pos(pos), m_speed(speed), m_id(id), m_selected(false) {}
    void draw();
    void drawWithNames();
    void select(bool flag) { m_selected = flag; }

private:
    Eigen::Vector3f     m_pos;
    float               m_speed;
    int                 m_id;
    bool                m_selected;
};
