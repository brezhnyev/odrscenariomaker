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
    void to_yaml(YAML::Node & parent) override;
    void from_yaml(const YAML::Node & node) override;

    ADDVAR(protected, float, FOV, 60.0f);
    ADDVAR(protected, float, height, 300.0f);
    ADDVAR(protected, float, width, 400.0f);
    ADDVAR(protected, bool, enabled, true);
    ADDVAR(protected, QLabel *, camWidget, nullptr);
};
