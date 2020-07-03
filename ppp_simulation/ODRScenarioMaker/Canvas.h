#pragma once

#include "Selectable.h"

#include <qrect.h>
#include <qimage.h>

#include <eigen3/Eigen/Eigen>

#include <string>
#include <vector>

class Canvas : public Selectable
{
public:
    Canvas();
    Canvas(std::string texname, QRect rect);
    void init();
    void draw() override;
    void drawWithNames() override;

private:
    QRect m_rect;
    std::vector<Eigen::Vector3f> m_vertices;
    std::vector<Eigen::Vector2f> m_texcoord;
    unsigned int m_texture;
    QImage m_image;
};