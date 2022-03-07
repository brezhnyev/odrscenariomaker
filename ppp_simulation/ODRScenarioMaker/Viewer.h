#pragma once

#include "Waypath.h"
#include "scenario.h"
#include "Vehicle.h"
#include "CanvasXODR.h"

#include <QGLViewer/qglviewer.h>

#include <thread>

class Viewer : public QGLViewer
{
    Q_OBJECT
public:
    Viewer(Scenario & scenario, const std::string & xodrfile);
    ~Viewer() override {};
    void draw() override;
    void drawWithNames() override;
    void init() override;
    void postSelection(const QPoint &point) override;
    void listenForResponse();

signals:
    void signal_addWaypoint(int);
    void signal_select(int);
    void signal_setVehicle(Eigen::Vector3f, float yaw);

public slots:
    void slot_select(int);

private:
    Scenario &      m_scenario;
    CanvasXODR      m_canvas;
};
