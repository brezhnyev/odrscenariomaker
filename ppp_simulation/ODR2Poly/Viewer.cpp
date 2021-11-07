#include <Viewer.h>
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


Viewer::Viewer(string xodrfile, float radius, float xodrResolution) : m_canvas(xodrfile, radius, xodrResolution)
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

Viewer::~Viewer() {}

void Viewer::init()
{
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    m_canvas.init();
}

void Viewer::draw()
{
    m_canvas.draw();
}

void Viewer::drawWithNames()
{
    m_canvas.drawWithNames();
}

void Viewer::mousePressEvent(QMouseEvent * event)
{
    gettimeofday(&mMClTime, nullptr);
    QGLViewer::mousePressEvent(event);
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
        timeval tv1;
        gettimeofday(&tv1, nullptr);
        cout << "Selected Point: " << sp.x << " " << sp.y << " " << sp.z << endl;
        cout << "OpenGL select time: " << 0.000001*((tv1.tv_sec*1000000+tv1.tv_usec) - (mMClTime.tv_sec*1000000+mMClTime.tv_usec)) << " s" << endl;
        //m_canvas.computePolys(Vector3d(sp.x, sp.y, sp.z), Vector2f(1.0f, 0.0f)); // for more proper time measurement
        m_canvas.computePolys(Vector3d(sp.x, sp.y, sp.z));
        timeval tv2;
        gettimeofday(&tv2, nullptr);
        cout << "Compute Poly time: " << 0.000001*((tv2.tv_sec*1000000+tv2.tv_usec) - (tv1.tv_sec*1000000+tv1.tv_usec)) << " s" << endl;
        cout << "------------------------" << endl;
    }
    else
    {
        int id = selectedName();
        // KB: no need to call update, since this function is a virtual and the update is called after it from OGLViewer
    }
}