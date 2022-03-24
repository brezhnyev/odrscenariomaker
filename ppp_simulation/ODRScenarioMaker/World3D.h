#pragma once

#include "Selectable.h"

#include <string>

class World3D : public Selectable
{
public:
    World3D();
    World3D(const std::string & objpath);
    void draw() override;
    std::string getType() const override { return "World3D"; }
    void init();

protected:
    uint m_objPointsList;
};