#include "Viewer.h"

#include <QKeyEvent>

#include <thread>

bool doRecording = false;

using namespace std;
using namespace qglviewer;
using namespace Eigen;

Viewer::Viewer(
    vector<vector<Vector2f>> &&baseLines,
    vector<Vector2f> &&baseLinesZ,
    vector<vector<Vector3f>> &&centerlines,
    vector<vector<Vector3f>> &&boundaries) : baseLines_(baseLines),
                                             baseLinesZ_(baseLinesZ),
                                             centerlines_(centerlines),
                                             boundaries_(boundaries),
                                             enableLegend_(true)
{
    colors_.emplace_back(0, 1, 1);
    colors_.emplace_back(0, 0, 1);
    colors_.emplace_back(1, 0, 0);
    colors_.emplace_back(1, 1, 0);
    colors_.emplace_back(0, 1, 0);
    colors_.emplace_back(1, 0, 0);
    colors_.emplace_back(1, 0.5, 0.5);
    colors_.emplace_back(1, 0, 0.5);
    colors_.emplace_back(0.5, 0.5, 0.5);
}

void Viewer::init()
{
    camera()->setSceneRadius(200);
    camera()->fitSphere(qglviewer::Vec(0, 0, 0), 200);
    resize(800,600);
}

void Viewer::updateMovingObjects(std::vector<Matrix4f> && v)
{
    {
        lock_guard<mutex> lk(mtx_);
        actors_ = v;
    }
    update();
}

void Viewer::drawWireBox(const Eigen::Vector3f & bbox, int)
{
    glBegin(GL_LINE_LOOP);
    glVertex3f(-bbox.x(),-bbox.y(),-bbox.z());
    glVertex3f( bbox.x(),-bbox.y(),-bbox.z());
    glVertex3f( bbox.x(), bbox.y(),-bbox.z());
    glVertex3f(-bbox.x(), bbox.y(),-bbox.z());
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(-bbox.x(),-bbox.y(), bbox.z());
    glVertex3f( bbox.x(),-bbox.y(), bbox.z());
    glVertex3f( bbox.x(), bbox.y(), bbox.z());
    glVertex3f(-bbox.x(), bbox.y(), bbox.z());
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(-bbox.x(),-bbox.y(),-bbox.z());
    glVertex3f(-bbox.x(),-bbox.y(), bbox.z());
    glVertex3f( bbox.x(),-bbox.y(),-bbox.z());
    glVertex3f( bbox.x(),-bbox.y(), bbox.z());
    glVertex3f( bbox.x(), bbox.y(),-bbox.z());
    glVertex3f( bbox.x(), bbox.y(), bbox.z());
    glVertex3f(-bbox.x(), bbox.y(),-bbox.z());
    glVertex3f(-bbox.x(), bbox.y(), bbox.z());
    glEnd();
}


void Viewer::drawSolidBox(const Eigen::Vector3f & bbox, int id)
{
    glBegin(GL_QUADS);
    glVertex3f(-bbox.x(),-bbox.y(), bbox.z());
    glVertex3f( bbox.x(),-bbox.y(), bbox.z());
    glVertex3f( bbox.x(), bbox.y(), bbox.z());
    glVertex3f(-bbox.x(), bbox.y(), bbox.z());
    glEnd();
    glBegin(GL_QUADS);
    glVertex3f(-bbox.x(),-bbox.y(),-bbox.z());
    glVertex3f( bbox.x(),-bbox.y(),-bbox.z());
    glVertex3f( bbox.x(), bbox.y(),-bbox.z());
    glVertex3f(-bbox.x(), bbox.y(),-bbox.z());
    glEnd();
    // make the sides a bit darker for contrast:
    float col[4];
    glGetFloatv(GL_CURRENT_COLOR, col);
    if (id <= 1) // only for vehicles and walkers (not for traffic lights)
        glColor3f(col[0]*0.5f, col[1]*0.5f, col[2]*0.5f);
    glBegin(GL_TRIANGLE_STRIP);
    glVertex3f(-bbox.x(),-bbox.y(),-bbox.z());
    glVertex3f(-bbox.x(),-bbox.y(), bbox.z());
    glVertex3f( bbox.x(),-bbox.y(),-bbox.z());
    glVertex3f( bbox.x(),-bbox.y(), bbox.z());
    glVertex3f( bbox.x(), bbox.y(),-bbox.z());
    glVertex3f( bbox.x(), bbox.y(), bbox.z());
    glVertex3f(-bbox.x(), bbox.y(),-bbox.z());
    glVertex3f(-bbox.x(), bbox.y(), bbox.z());
    glVertex3f(-bbox.x(),-bbox.y(),-bbox.z());
    glVertex3f(-bbox.x(),-bbox.y(), bbox.z());
    glEnd();
}


void Viewer::drawInfo()
{
    glDisable(GL_DEPTH_TEST);
    if (enableLegend_)
    {
        glColor3f(1,1,1); drawText(10, 20, "Enable/Disable legend: L, start/stop recording: R");
        glColor3f(colors_[0][0], colors_[0][1], colors_[0][2]); drawText(10,40, "■"); glColor3f(1,1,1); drawText(30, 40, "Vehicles");
        glColor3f(colors_[1][0], colors_[1][1], colors_[1][2]); drawText(10,60, "■"); glColor3f(1,1,1); drawText(30, 60, "Walker");
        glColor3f(colors_[2][0], colors_[2][1], colors_[2][2]); drawText(10,80, "■");
        glColor3f(colors_[3][0], colors_[3][1], colors_[3][2]); drawText(30,80, "■");
        glColor3f(colors_[4][0], colors_[4][1], colors_[4][2]); drawText(50,80, "■");
        glColor3f(1,1,1); drawText(70,80, "Traffic lights");
        glColor3f(colors_[5][0], colors_[5][1], colors_[5][2]); drawText(10,100, "□"); glColor3f(1,1,1); drawText(30, 100, "Speed limit");
        glColor3f(colors_[6][0], colors_[6][1], colors_[6][2]); drawText(10,120, "□"); glColor3f(1,1,1); drawText(30, 120, "Give way");
        glColor3f(colors_[7][0], colors_[7][1], colors_[7][2]); drawText(10,140, "□"); glColor3f(1,1,1); drawText(30, 140, "Stop");
        glColor3f(colors_[8][0], colors_[8][1], colors_[8][2]); drawText(10,160, "□"); glColor3f(1,1,1); drawText(30, 160, "Unknown traffic sign");
    }
    if (doRecording)
    {
        glColor3f(1,0,0); drawText(500, 40, "RECORDING");
    }
}


void Viewer::draw()
{
    drawInfo();
    // static objects:
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glColor3f(0.5,0.5,0.5);
    glLineWidth(3);

    // for simple flat shading:
    Vector3f prevV(0,0,0);
    Vector3f lightDir(1,0,0);

    for (size_t i = 0; i < baseLines_.size(); ++i)
    {
        auto && v = baseLines_[i];
        auto && z = baseLinesZ_[i];
        glBegin(GL_TRIANGLE_STRIP);
        for (auto && p : v)
        {
            if (!prevV.isZero())
            {
                Vector3f tangent = (Vector3f(p.x(), p.y(), z[0]) - prevV).normalized();
                float f = lightDir.dot(tangent)*0.25;
                glColor3f(0.5-f, 0.5-f, 0.5-f);
            }
            prevV = Vector3f(p.x(), p.y(), z[0]);
            glVertex3f(p.x(), p.y(), z[0]);
            glVertex3f(p.x(), p.y(), z[1]);
        }
        glVertex3f(v[0].x(), v[0].y(), z[0]);
        glVertex3f(v[0].x(), v[0].y(), z[1]);
        glEnd();
    }
    // road centerlines:
    glLineWidth(1);
    glColor3f(0.75,0.75,1);
    for (auto && v : centerlines_)
    {
        glBegin(GL_POINTS);
        for (auto && p : v)
        {
            glVertex3f(p.x(), p.y(), p.z());
        }
        glEnd();
    }

    // road boundaries
    glLineWidth(3);
    glColor3f(1,1,1);
    for (auto && v : boundaries_)
    {
        glBegin(GL_LINE_STRIP);
        for (auto && p : v)
        {
            glVertex3f(p.x(), p.y(), p.z());
        }
        glEnd();
    }

    // moving objects / traffic lights:
    {
        lock_guard<mutex> lk(mtx_);
        for (auto a : actors_) // auto && a : actors_   causes flickering
        {
            bool drawSolid = true;
            int cid = int(a(3,3));
            glColor3f(colors_[cid][0], colors_[cid][1], colors_[cid][2]);
            if (cid >= 5) drawSolid = false;
            a(3,3) = 1.0f;
            Vector3f bbox = a.block(3,0,1,3).transpose();
            a.block(3,0,1,3).setZero();

            glPushMatrix();
            glMultMatrixf(a.data());
            if (drawSolid)
                drawSolidBox(bbox, cid);
            else
                drawWireBox(bbox, cid);
            glPopMatrix();
        }
    }
}


void Viewer::keyPressEvent(QKeyEvent * event)
{
    if(event->key() == Qt::Key_L)
    {
        enableLegend_ = !enableLegend_;
        update();
    }
    if(event->key() == Qt::Key_R)
    {
        doRecording = !doRecording;
        update();
    }
    QGLViewer::keyPressEvent(event);
}