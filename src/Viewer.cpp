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


Viewer::Viewer(Scenario & scenario, Eigen::Matrix4f & spectatorMtx, string xodrfile, string objfile) : 
    m_scenario(scenario), m_canvas(xodrfile), m_world3d(objfile), m_spectatorMtrx(spectatorMtx)
{
}

void Viewer::init()
{
    using namespace qglviewer;
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    m_canvas.init();
    m_world3d.init();

    auto bbox = m_canvas.getSceneBbox();
    Vec minp(bbox.first[0], bbox.first[1], bbox.first[2]);
    Vec maxp(bbox.second[0], bbox.second[1], bbox.second[2]);
    camera()->setSceneBoundingBox(minp, maxp);
    camera()->setSceneRadius((bbox.second - bbox.first).norm()); // we reset the radius from setSceneBoundingBox
    //camera()->showEntireScene();
    camera()->fitSphere(sceneCenter(), 0.1*sceneRadius()); // we make our own showEntireScene()
    camera()->setZNearCoefficient(0.001);
    camera()->setFieldOfView(M_PI*100.f/180.f);
}

void Viewer::draw()
{
    m_canvas.draw();
    m_scenario.draw();
    m_world3d.draw();
    camera()->getModelViewMatrix(m_spectatorMtrx.data());
    if (m_renderAxis)
        renderAxis();
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
            signal_moveSelectedTo(sp.x, sp.y, sp.z);
        }
        else
        {
            Waypath * waypath = m_scenario.getActiveWaypath();
            if (nullptr == waypath)
            {
                QMessageBox::warning(this, "Error adding Element", "Add/activate waypath in Actor!");
                return;
            }
            int id = (new Waypoint(Vector3f(sp.x, sp.y, sp.z), waypath))->getID();
            emit signal_addWaypoint(id);
        }
    }
    else
    {
        int id = selectedName();
        m_scenario.select(id);
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
            // Important: the pointUnderPixel only has effect in postSelection
            // evntl. when all names are filled with drawWithNames for glReadPixels !!
            int dx = int(e->pos().x()) - x;
            int dy = int(e->pos().y()) - y;
            if (e->modifiers() == Qt::ShiftModifier)
            {
                // Lets try to fetch the selected object as close to mouse pointer:
                Selectable * object = m_scenario.getSelected();
                Vector3f pos;
                if (object->getType() == "Waypoint")
                {
                    Waypoint * waypoint = dynamic_cast<Waypoint*>(object);
                    pos = waypoint->get_pos();
                }
                if (object->getType() == "Camera")
                {
                    Camera * camera = dynamic_cast<Camera*>(object);
                    pos = camera->get_pos();
                    Actor * parent = dynamic_cast<Actor*>(camera->getParent());
                    if (parent)
                    {
                        Matrix4f parentTrf; parentTrf.setIdentity();
                        Vector3f ori = parent->get_ori();
                        parentTrf.block(0,0,3,3) = AngleAxisf(ori[2]*DEG2RAD, Vector3f::UnitZ())*AngleAxisf(ori[1]*DEG2RAD, Vector3f::UnitY())*AngleAxisf(ori[0]*DEG2RAD, Vector3f::UnitX()).toRotationMatrix();
                        parentTrf.block(0,3,3,1) = parent->get_pos();
                        pos = (parentTrf*Vector4f(pos[0], pos[1], pos[2], 1.0f)).block(0,0,3,1);
                    }
                }
                qglviewer::Vec orig, dir;
                camera()->convertClickToLine(QPoint(e->pos().x(), e->pos().y()), orig, dir); // in World CS
                Vector3f V = pos - Vector3f(orig.x, orig.y, orig.z);
                Vector3f P = Vector3f(orig.x, orig.y, orig.z) + Vector3f(dir.x, dir.y, dir.z)*V.norm();
                // Since we cannot sample the underlying geometry, lets keep z same as original:
                P.z() = pos.z();
                // Potentially its possible to sample XODR's z similar how we do it in RAIV
                signal_moveSelectedTo(P.x(), P.y(), P.z());
            }
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

void Viewer::keyPressEvent(QKeyEvent * e)
{
    if (e->key() == Qt::Key_A)
    {
        m_renderAxis = !m_renderAxis;
        update();
    }
    else if (e->key() == Qt::Key_Escape)
    {
        // we do not allow exit program
    }
    else
        QGLViewer::keyPressEvent(e);
}

void Viewer::renderAxis()
{
    const float length = 0.25f;
    const float charWidth = length / 40.0f;
    const float charHeight = length / 30.0f;
    const float charShift = 1.04 * length;

    glPushMatrix();
    float sf = camera()->sceneRadius();
    glScalef(sf, sf, sf);
    glBegin(GL_LINES);
    // The X
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(charShift, charWidth, -charHeight);
    glVertex3f(charShift, -charWidth, charHeight);
    glVertex3f(charShift, -charWidth, -charHeight);
    glVertex3f(charShift, charWidth, charHeight);
    // The X axis:
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(length, 0.0f, 0.0f);
    // The Y
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3d(charWidth, charShift, charHeight);
    glVertex3d(0.0, charShift, 0.0);
    glVertex3d(-charWidth, charShift, charHeight);
    glVertex3d(0.0, charShift, 0.0);
    glVertex3d(0.0, charShift, 0.0);
    glVertex3d(0.0, charShift, -charHeight);
    // The Y axis:
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, length, 0.0f);
    // The Z
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3d(-charWidth, charHeight, charShift);
    glVertex3d(charWidth, charHeight, charShift);
    glVertex3d(charWidth, charHeight, charShift);
    glVertex3d(-charWidth, -charHeight, charShift);
    glVertex3d(-charWidth, -charHeight, charShift);
    glVertex3d(charWidth, -charHeight, charShift);
    // The Z axis:
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, length);
    glEnd();
    glPopMatrix();

    update();
}