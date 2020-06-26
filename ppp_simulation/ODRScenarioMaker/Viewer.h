#pragma once

#include "Canvas.h"
#include "Waypath.h"
#include "scenario.h"

#include <QGLViewer/qglviewer.h>

class Viewer : public QGLViewer
{
    Q_OBJECT
public:
    Viewer(Scenario & scenario);
    ~Viewer() override {};
    void draw() override;
    void drawWithNames() override;
    void init() override;
    void postSelection(const QPoint &point) override;

signals:
    void signal_addWaypath(int);
    void signal_addWaypoint(int);
    void signal_delWaypath(int);
    void signal_delWaypoint(int);
    void signal_select(int);
    

private:
    Canvas m_canvas;
    Scenario & m_scenario;
};
