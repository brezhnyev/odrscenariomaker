#pragma once

#include <atomic>

#include <eigen3/Eigen/Eigen>

class BaseObject
{
public:
    BaseObject() : m_selected(false) { ++s_ID; m_id = s_ID; }
    int getID() { return m_id; }
protected:
    static int          s_ID;
    int                 m_id;
    bool                m_selected;
};

class Waypoint : public BaseObject
{
public:
    Waypoint(Eigen::Vector3f pos, float speed) : BaseObject(), m_pos(pos), m_speed(speed) {}
    void draw();
    void drawWithNames();
    void select(bool flag) { m_selected = flag; }
    Eigen::Vector3f getPosition() { return m_pos; }
    void  setPosition(Eigen::Vector3f val) { m_pos = val; }
    float getSpeed() { return m_speed; }
    float setSpeed(float speed) { m_speed = speed; }

private:
    Eigen::Vector3f     m_pos;
    float               m_speed;
};
