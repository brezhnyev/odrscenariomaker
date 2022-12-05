#include "Viewer.h"
#include "Vehicle.h"
#include "Camera.h"

#include <QGLViewer/vec.h>
#include <QtWidgets/QMessageBox>
#include <QtGui/QtEvents>

#include <iostream>
#include <sstream>


#include <stdio.h> 

#include <unistd.h> 
#include <string.h>

using namespace std;
//using namespace qglviewer; // otherwise names clash with Camera
using namespace Eigen;

Matrix4f camTrf;
extern int playStatus;

Viewer::Viewer(Scenario & scenario, const string & xodrfile, string objfile) : m_scenario(scenario), m_canvas(xodrfile), m_world3d(objfile)
{
    using namespace qglviewer;
    camera()->setSceneRadius(400);
    camera()->fitSphere(Vec(0, 0, 0), 200);
    camera()->setZNearCoefficient(0.01);
    camera()->setFieldOfView(M_PI_2);
}

void Viewer::init()
{
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    m_canvas.init();
    m_world3d.init();
}

void Viewer::draw()
{
    m_canvas.draw();
    m_scenario.draw();
    m_world3d.draw();
    camera()->getModelViewMatrix(camTrf.data());
}

void Viewer::drawWithNames()
{
    m_canvas.drawWithNames();
    m_scenario.drawWithNames();
}

void Viewer::postSelection(const QPoint &point)
{
    // Find the selectedPoint coordinates, using camera()->pointUnderPixel().
    bool found;
    qglviewer::Vec sp = QGLViewer::camera()->pointUnderPixel(point, found);

    if (selectedName() == -1) return;

    if (selectedName() == m_canvas.getID()) // Canvas
    {
        Camera * cam = dynamic_cast<Camera*>(m_scenario.getActiveActor());
        if (cam)
        {
            cam->setPos(Vector3f(0,0,0));
            signal_activeSelectableMovedBy(sp.x, sp.y, sp.z);
        }
        else
        {
            Waypath * waypath = m_scenario.getActiveWaypath();
            if (nullptr == waypath)
            {
                QMessageBox::warning(this, "Error adding Element", "Add/activate waypath in Actor!");
                return;
            }
            int id = waypath->addChild(new Waypoint(Vector3f(sp.x, sp.y, sp.z), 0, waypath));
            emit signal_addWaypoint(id);
        }
    }
    else
    {
        int id = selectedName();
        m_scenario.select(id); // In graphics we can only select waypoint
        emit signal_select(id);
        // KB: no need to call update, since this function is a virtual and the update is called after it from OGLViewer
    }
}

void Viewer::mousePressEvent(QMouseEvent * e)
{
    if (e->button() == Qt::LeftButton)
        m_leftMousePressed = true;

    QGLViewer::mousePressEvent(e);
}

void Viewer::mouseReleaseEvent(QMouseEvent * e)
{
    m_leftMousePressed = false;
    x = y = -1;

    QGLViewer::mouseReleaseEvent(e);
}

void Viewer::mouseMoveEvent(QMouseEvent * e)
{
    if (m_leftMousePressed)
    {
        if (x != -1)
        {
            // bool found;
            // Vec sp = camera()->pointUnderPixel(QPoint(-e->pos().x(), -e->pos().y()), found);
            // if (found)
            //     cout << sp.x << " " << sp.y << " " << sp.z << endl;
            // cout << selectedName() << endl;
            int dx = int(e->pos().x()) - x;
            int dy = int(e->pos().y()) - y;
            if (e->modifiers() == Qt::ShiftModifier)
                signal_activeSelectableMovedBy(0.01f*dx, -0.01f*dy, 0);
            else
            {
                // lets implement the orbiting navigation since not present in QGLViewer:
                // orientation:
                double R [3][3];
                camera()->orientation().getRotationMatrix(R);
                double betta = atan2(R[1][0], R[0][0]);
                Eigen::Matrix3d M;
                memcpy(M.data(), R, sizeof(double)*9);
                M.transposeInPlace();
                Eigen::Matrix3d rot = AngleAxisd(betta-0.25*dx*DEG2RAD, Vector3d::UnitZ())*AngleAxisd(-0.25*dy*DEG2RAD, Vector3d::UnitX())*AngleAxisd(-betta, Vector3d::UnitZ()).toRotationMatrix();
                M = rot*M;
                M.transposeInPlace();
                memcpy(R, M.data(), sizeof(double)*9);
                qglviewer::Quaternion d; d.setFromRotationMatrix(R);
                camera()->setOrientation(d);
                // position:
                qglviewer::Vec p = camera()->position();
                qglviewer::Vec pivot = camera()->pivotPoint();
                Eigen::Vector3d pos(p.x, p.y, p.z);
                Eigen::Vector3d piv(pivot.x, pivot.y, pivot.z);
                pos = piv + rot*(pos-piv);
                p.x = pos[0]; p.y = pos[1]; p.z = pos[2];
                camera()->setPosition(p);
                update();
            }
        }
        x = e->pos().x();
        y = e->pos().y();
    }
    else
        QGLViewer::mouseMoveEvent(e);
}

void Viewer::slot_select(int id)
{
    m_scenario.select(id); // in the tree we can select both waypoint and waypath
    emit signal_select(id);
    update();
}