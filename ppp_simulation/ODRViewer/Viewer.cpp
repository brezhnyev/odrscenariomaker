#include <Viewer.h>
#include <QtWidgets/QMessageBox>
#include <QKeyEvent>

#include <iostream>
#include <sstream>
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
#define FONTSIZE 20

using namespace std;
using namespace qglviewer;
using namespace Eigen;

extern Matrix4f camTrf;

using namespace std;


Viewer::Viewer(string xodrfiles, float xodrResolution) : mFr(xodrfiles), mXodrRes(xodrResolution), mRibbonStart(0,0,0)
{
    using namespace net;

    // cout << "Version: " << glGetString(GL_VERSION) << endl; // KB: causes cout stop!
    // cout << "GLSL: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
    // cout.flush();
    camera()->setZNearCoefficient(0.01);
    camera()->setFieldOfView(M_PI_2/2);

    string next = mFr.getNext("xodr");
    if (!next.empty())
    {
        cout << "Loading " << next << "  "; cout.flush();
        mCurrentFile = next;
        m_canvas = new CanvasXODR(next, xodrResolution);
    }
    else
    {
        cout << "Loading " << xodrfiles << "  "; cout.flush();
        mCurrentFile = xodrfiles;
        m_canvas = new CanvasXODR(xodrfiles, xodrResolution);
    }
    cout << "Loading completed" << endl;

    setWindowTitle(next.c_str());
}

Viewer::~Viewer() {}

void Viewer::init()
{
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    m_canvas->init();
    double R = m_canvas->getSceneRadius();
    camera()->setSceneRadius(R);
    camera()->fitSphere(Vec(0, 0, 0), R);
}

void Viewer::draw()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);

    m_canvas->draw();
    if (!mInfo.empty())
    {
        // Printing text leads to bug in the mSelPoint
        // glColor3f(1.0f, 1.0f, 0.0f);
        // drawText(10, 20, QString(mInfo.c_str()));
        // drawText(10, 50, "Global position: " + QString::number(mSelPoint.x) + "   " + QString::number(mSelPoint.y) + "   " + QString::number(mSelPoint.z));

        glColor3f(0.0f, 0.0f, 1.0f);
        glPointSize(20);
        glBegin(GL_POINTS);
        glVertex3f(mSelPoint.x, mSelPoint.y, mSelPoint.z + 0.1f);
        glEnd();
    }
}

void Viewer::drawWithNames()
{
    m_canvas->drawWithNames();
}

void Viewer::mousePressEvent(QMouseEvent * event)
{
    //gettimeofday(&mMClTime, nullptr);
    QGLViewer::mousePressEvent(event);
}

void Viewer::keyPressEvent(QKeyEvent * event)
{
    string next = mFr.getNext("xodr");
    auto doLoad = [this](){
        CanvasXODR * canvas = m_canvas;
        m_canvas = new CanvasXODR(mCurrentFile, mXodrRes);
        cout << "Loading completed" << endl;
        setWindowTitle(mCurrentFile.c_str());
        delete canvas;
        update();
    };
    if (event->key() == Qt::Key_N && !next.empty())
    {
        cout << "Loading " << next << "  "; cout.flush();
        mCurrentFile = next;
        doLoad();
    }
    if (event->key() == Qt::Key_R && !mCurrentFile.empty())
    {
        cout << "Reloading " << mCurrentFile << "  "; cout.flush();
        doLoad();
    }
    QGLViewer::keyPressEvent(event);
}

void Viewer::postSelection(const QPoint &point)
{
    qglviewer::Vec orig, dir;
    // Compute orig and dir, used to draw a representation of the intersecting
    // line
    camera()->convertClickToLine(point, orig, dir); // orig == camera position

    // Find the selectedPoint coordinates, using camera()->pointUnderPixel().
    bool found;

    if (selectedName() == -1)
    {
        mInfo.clear();
        float f = orig.z/dir.z;
        if (mRibbonStart.squaredNorm()) // some value as flag of initialization
        {
             mRibbonEnd = orig - dir*f;
             if (mRibbonStart.x > mRibbonEnd.x) swap(mRibbonStart.x, mRibbonEnd.x);
             if (mRibbonStart.y > mRibbonEnd.y) swap(mRibbonStart.y, mRibbonEnd.y);
             m_canvas->highlightSelection(
                 Eigen::Vector3d(mRibbonStart.x, mRibbonStart.y, mRibbonStart.z),
                 Eigen::Vector3d(mRibbonEnd.x, mRibbonEnd.y, mRibbonEnd.z));
            mRibbonStart = qglviewer::Vec(0,0,0);
            mRibbonEnd = qglviewer::Vec(0,0,0);
        }
        else
        {
            mRibbonStart = orig - dir*f;
        }
        return;
    }

    mSelPoint = camera()->pointUnderPixel(point, found);
    int id = selectedName();
    glaneid_t laneID = m_canvas->printLaneInfo(id, Vector3d(mSelPoint.x, mSelPoint.y, mSelPoint.z));
    // display the lane info:
    stringstream ss;
    ss << laneID;
    mInfo = ss.str();
    cout << "Point: (" << mSelPoint.x << "/" << mSelPoint.y << "/" << mSelPoint.z << ")      " << ss.str();
}