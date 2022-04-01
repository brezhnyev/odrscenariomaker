#pragma once

#include "Actor.h"
#include <QtWidgets/QLabel> // need to think about Qt here

class QLabel;

class Camera : public Actor
{
public:
    Camera() : Actor("") { m_cameraWidget = new QLabel(); }
    Camera(const std::string & name) : Actor(name) {}
    void draw() override {};
    void drawWithNames() override {};
    void drawGeometry() {};
    std::string getType() const override { return "Camera"; }
    QLabel * getCamWidget() { return m_cameraWidget; }

private:
    QLabel * m_cameraWidget;
};
