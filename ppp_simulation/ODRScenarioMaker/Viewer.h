#pragma once

#include "Canvas.h"
#include "Waypath.h"

#include <QGLViewer/qglviewer.h>

class Viewer : public QGLViewer
{
    Q_OBJECT
public:
    Viewer();
    ~Viewer() override {};
    void draw() override;
    void drawWithNames() override;
    void init() override;
    void postSelection(const QPoint &point) override;
    void addWaypath();
    void addWaypoint(Eigen::Vector3f);
    void delWaypath(int id);
    void delWaypoint();
    void setActivePath(int id) { if (id < m_waypaths.size()) m_activeWaypath = id;  }
    void selectWaypoint(int id);

private:
    Canvas m_canvas;
    std::vector<Waypath>    m_waypaths;
    int                     m_activeWaypath;
    int                     m_activeWaypoint;
};
