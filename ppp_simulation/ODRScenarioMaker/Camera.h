#pragma once

#include "Actor.h"
#include <QtWidgets/QLabel> // need to think about Qt here

class QLabel;

class Camera : public Actor
{
public:
    Camera(Selectable * parent) : Actor("", parent) { m_cameraWidget = new QLabel(); m_bbox = Eigen::Vector3f(1,1,1); }
    Camera(const std::string & name, Selectable * parent) : Actor(name, parent) {}
    void draw() const override;
    void drawWithNames() const override;
    void drawGeometry() const override;
    std::string getType() const override { return "Camera"; }
    QLabel * getCamWidget() { return m_cameraWidget; }

    float getFOV() { return m_FOV; }
    float getWidth() { return m_w; }
    float getHeight() { return m_h; }
    void setFOV(double val) { m_FOV = (float)val; }
    void setWidth(double val) { m_w = (float)val; }
    void setHeight(double val) { m_h = (float)val; }
 
private:
    QLabel * m_cameraWidget{nullptr};
    float m_FOV{60.0f};
    float m_w{400};
    float m_h{300};
};
