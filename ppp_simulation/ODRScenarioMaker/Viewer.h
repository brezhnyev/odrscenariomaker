#pragma once

#include "Canvas.h"
#include "Waypath.h"
#include "scenario.h"
#include "Vehicle.h"

#include <QGLViewer/qglviewer.h>

#include <thread>

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
    void listenForResponse();

signals:
    void signal_addActor(int);
    void signal_addWaypath(int);
    void signal_addWaypoint(int);
    void signal_delWaypath(int);
    void signal_delWaypoint(int);
    void signal_select(int);
    void signal_setVehicle(Eigen::Vector3f, float yaw);

public slots:
    void slot_select(int);
    void slot_play();
    void slot_stop();

private:
    Canvas m_canvas;
    Scenario & m_scenario;
    bool m_isconnected;
    int m_sock;
    std::thread * m_commThread;
};
