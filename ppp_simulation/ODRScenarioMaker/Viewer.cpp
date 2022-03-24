#include "Viewer.h"
#include "Vehicle.h"

#include <QGLViewer/vec.h>
#include <QtWidgets/QMessageBox>

#include <iostream>
#include <sstream>


#include <stdio.h> 

namespace net
{
    #include <sys/socket.h>
    #include <arpa/inet.h> 
} 
#include <unistd.h> 
#include <string.h>
#define PORT 12345 

using namespace std;
using namespace qglviewer;
using namespace Eigen;

extern Matrix4f camTrf;


Viewer::Viewer(const string & xodrfile) : m_canvas(xodrfile)
{
    using namespace net;

    // cout << "Version: " << glGetString(GL_VERSION) << endl; // KB: causes cout stop!
    // cout << "GLSL: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
    // cout.flush();
    camera()->setSceneRadius(200);
    camera()->fitSphere(Vec(0, 0, 0), 200);
    camera()->setZNearCoefficient(0.01);
    camera()->setFieldOfView(M_PI_2/2);
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
    //camera()->getModelViewMatrix(camTrf.data());
}

void Viewer::drawWithNames()
{
    m_canvas.drawWithNames();
    m_scenario.drawWithNames();
}

void Viewer::postSelection(const QPoint &point)
{
    qglviewer::Vec orig, dir;
    // Compute orig and dir, used to draw a representation of the intersecting
    // line
    camera()->convertClickToLine(point, orig, dir); // orig == camera position

    // Find the selectedPoint coordinates, using camera()->pointUnderPixel().
    bool found;
    Vec sp = camera()->pointUnderPixel(point, found);

    if (selectedName() == -1) return;

    if (selectedName() == 1) // Canvas
    {
        int id = m_scenario.addWaypoint(Vector3f(sp.x, sp.y, sp.z));
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Add/activate waypath in Actor!");
            return;
        }
        emit signal_addWaypoint(id);
    }
    else
    {
        int id = selectedName();
        m_scenario.select(id); // In graphics we can only select waypoint
        emit signal_select(id);
        // KB: no need to call update, since this function is a virtual and the update is called after it from OGLViewer
    }
}

void Viewer::slot_select(int id)
{
    m_scenario.select(id); // in the tree we can select both waypoint and waypath
    emit signal_select(id);
    update();
}