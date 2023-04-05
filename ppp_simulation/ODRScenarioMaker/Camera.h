#pragma once

#include "Actor.h"
#include <QtWidgets/QLabel> // need to think about Qt here

#define ADDVAR(type, var, initVal)\
    private: type m_##var{initVal};\
    public:\
    void set_##var(type val) { m_##var = val; }\
    type get_##var() { return m_##var; }

class QLabel;

class Camera : public Actor
{
public:
    Camera(Selectable * parent) : Actor("", parent) { m_camWidget = new QLabel(); m_bbox = Eigen::Vector3f(1,1,1); }
    Camera(const std::string & name, Selectable * parent) : Actor(name, parent) {}
    void draw() const override;
    void drawWithNames() const override;
    void drawGeometry() const override;
    std::string getType() const override { return "Camera"; }

    ADDVAR(double, FOV, 60.0f);
    ADDVAR(double, height, 300.0f);
    ADDVAR(double, width, 400.0f);
    ADDVAR(bool, enabled, true);
    ADDVAR(QLabel *, camWidget, nullptr);
};
