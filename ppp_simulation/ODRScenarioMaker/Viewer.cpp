#include "Viewer.h"
#include <QGLViewer/vec.h>
#include <QtWidgets/QMessageBox>

#include <iostream>

using namespace std;
using namespace qglviewer;
using namespace Eigen;


Viewer::Viewer(Scenario & scenario) : m_canvas("../data/Town02.jpg", QRect(-27, 92, 239, 237)), m_scenario(scenario)
{
    cout << "Version: " << glGetString(GL_VERSION) << endl;
    cout << "GLSL: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
    cout.flush();
    camera()->setSceneRadius(200);
    camera()->fitSphere(Vec(0, 0, 0), 200);
    camera()->setZNearCoefficient(0.01);
}

void Viewer::init()
{
    m_canvas.init();
    int id = m_scenario.addWaypath();
    emit signal_addWaypath(id); // must be done over gui later
}

void Viewer::draw()
{
    m_canvas.draw();
    m_scenario.draw();
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

    if (selectedName() == 0) // put a new waypoint
    {
        int id = m_scenario.addWaypoint(Vector3f(sp.x, sp.y, sp.z));
        emit signal_addWaypoint(id);
    }
    else
    {
        int id = selectedName();
        m_scenario.select(id); // In graphics we can only select waypoint
        emit signal_select(id);
    }
}

void Viewer::slot_select(int id)
{
    m_scenario.select(id); // in the tree we can select both waypoint and waypath
    emit signal_select(id);
    update();
}

void Viewer::slot_play()
{
    Waypath & wp = *m_scenario.getActiveWaypath();
    string command = "./client/client \"" + wp.serialize() + "\" &";
    system(command.c_str());
}