#pragma once

#include "globals.h"
#include "Actor.h"
#include <QtWidgets/QLabel> // need to think about Qt here

class QLabel;

class Camera : public Actor
{
public:
    Camera(Selectable * parent) : Actor("", parent) { m_camWidget = new QLabel(); set_bbox(Eigen::Vector3f(1,1,1)); }
    Camera(const std::string & name, Selectable * parent) : Actor(name, parent) {}
    void draw() const override;
    void drawWithNames() const override;
    void drawGeometry() const override;
    std::string getType() const override { return "Camera"; }

    ADDVAR(protected, double, FOV, 60.0f);
    ADDVAR(protected, double, height, 300.0f);
    ADDVAR(protected, double, width, 400.0f);
    ADDVAR(protected, bool, enabled, true);
    ADDVAR(protected, QLabel *, camWidget, nullptr);
};
