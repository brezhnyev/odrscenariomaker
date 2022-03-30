#pragma once

#include "Actor.h"

class Camera : public Actor
{
public:
    Camera() : Actor("") {}
    Camera(const std::string & name) : Actor(name) {}
    void draw() override {};
    void drawWithNames() override {};
    void drawGeometry() {};
    std::string getType() const override { return "Camera"; }
};
