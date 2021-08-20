#include <Viewer.h>

#include <thread>

using namespace std;
using namespace qglviewer;
using namespace Eigen;

Viewer::Viewer(
    vector<vector<Vector2f>> && baseLines,
    vector<Vector2f> && baseLinesZ,
    vector<vector<Vector3f>> && centerlines,
    vector<vector<Vector3f>> && boundaries)
{
    baseLines_ = baseLines;
    baseLinesZ_ = baseLinesZ;
    centerlines_ = centerlines;
    boundaries_ = boundaries;
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

void Viewer::draw()
{
    // static objects:
    glDisable(GL_LIGHTING);
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
            switch(int(a(3,3)))
            {
                case 0: // car
                    glColor3f(0,1,1);
                    break;
                case 1: // walker
                    glColor3f(0,0,1);
                    break;
                case 2: // traffic light red
                    glColor3f(1,0,0);
                    break;
                case 3: // traffic light yellow
                    glColor3f(1,1,0);
                    break;
                case 4: // traffic light green
                    glColor3f(0,1,0);
                    break;
            }
            a(3,3) = 1.0f;
            Vector3f bbox = a.block(3,0,1,3).transpose();
            a.block(3,0,1,3).setZero();

            glPushMatrix();

            glMultMatrixf(a.data());

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

            glPopMatrix();
        }
    }
}
