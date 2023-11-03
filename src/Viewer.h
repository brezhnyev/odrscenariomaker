#pragma once

#include "Waypath.h"
#include "scenario.h"
#include "Vehicle.h"
#include "CanvasXODR.h"
#include "World3D.h"

#include <QGLViewer/qglviewer.h>

#include <thread>

class Viewer : public QGLViewer
{
    Q_OBJECT
public:
    Viewer(Scenario & scenario, Eigen::Matrix4f & spectatorMtx, std::string xodrfile, std::string objfile = "");
    ~Viewer() override {};
    void draw() override;
    void drawWithNames() override;
    void init() override;
    void postSelection(const QPoint &point) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;
    void keyPressEvent(QKeyEvent*) override;
    void listenForResponse();

signals:
    void signal_addWaypoint(int);
    void signal_select(int);
    void signal_setVehicle(Eigen::Vector3f, float yaw);
    void signal_moveSelectedTo(float x, float y, float z);

public slots:
    void slot_select(int);

private:
    void renderAxis();

private:
    Scenario &      m_scenario;
    CanvasXODR      m_canvas;
    World3D         m_world3d;
    bool            m_leftMousePressed{false};
    int x{-1};
    int y{-1};
    bool            m_renderAxis{false};
    Eigen::Matrix4f & m_spectatorMtrx;
};
