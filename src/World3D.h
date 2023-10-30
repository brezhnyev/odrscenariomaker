#pragma once

#include "Selectable.h"

#include <string>
#include <vector>

class World3D : public Selectable
{
public:
    World3D() : Selectable(nullptr) {};
    ~World3D();
    World3D(const std::string & objfile);
    void draw() const override;
    std::string getType() const override { return "World3D"; }
    void init();

protected:
    unsigned int VAO;
    uint m_objPointsList;
    std::vector<float> m_vertices;
    uint64_t m_vnum{0};
};