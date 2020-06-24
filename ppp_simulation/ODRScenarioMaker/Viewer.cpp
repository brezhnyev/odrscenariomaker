#include "Viewer.h"
#include <QGLViewer/vec.h>
#include <QtWidgets/QMessageBox>

#include <iostream>

using namespace std;
using namespace qglviewer;
using namespace Eigen;

Viewer::Viewer() : m_canvas("../data/Town02.jpg", QRect(-30, 90, 230, 230)),
m_activeWaypath(-1)
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
}

void Viewer::draw()
{
    m_canvas.draw();
    for (auto && wp : m_waypaths) wp.draw();
}

void Viewer::drawWithNames()
{
    m_canvas.drawWithNames();
    for (auto && wp : m_waypaths) wp.drawWithNames();
}

void Viewer::addWaypath()
{
    m_waypaths.push_back(Waypath());
    m_activeWaypath = m_waypaths.size() - 1;
}

void Viewer::delWaypath(int id)
{
    if (id < m_waypaths.size()) return;
    if (m_waypaths.empty()) return;
    auto it = m_waypaths.begin();
    std::advance(it, id);
    m_waypaths.erase(it);
    m_activeWaypath = -1;
}

void Viewer::addWaypoint(Eigen::Vector3f p)
{
    m_waypaths[m_activeWaypath].pushWaypoint(p);
}

void Viewer::delWaypoint()
{
    m_waypaths[m_activeWaypath].popWaypoint();
}

void Viewer::selectWaypoint(int id)
{
    // first deselect
    for (auto && wpath : m_waypaths) wpath.selectWaypoint(-1);
    // then select the id:
    m_waypaths[m_activeWaypath].selectWaypoint(id);
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

    if (m_waypaths.empty()) addWaypath();

    if (selectedName() == 255) // put a new waypoint
        addWaypoint(Vector3f(sp.x, sp.y, sp.z));
    else
        selectWaypoint(selectedName());
}