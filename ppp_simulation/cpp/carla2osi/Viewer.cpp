#include <Viewer.h>

#include <thread>

using namespace std;

Viewer::Viewer(
    vector<vector<Eigen::Vector2f>> && staticObjects,
    vector<vector<Eigen::Vector2f>> && centerlines,
    vector<vector<Eigen::Vector2f>> && boundaries)
{
    dataStatic_ = staticObjects;
    centerlines_ = centerlines;
    boundaries_ = boundaries;
}

void Viewer::init()
{
    camera()->setSceneRadius(100);
    camera()->fitSphere(qglviewer::Vec(0, 0, 0), 100);
    resize(800,600);
}

void Viewer::updateMovingObjects(std::vector<Eigen::Matrix4f> && v)
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
    glColor3f(0.75,0.75,0.75);
    glLineWidth(3);
    for (auto && v : dataStatic_)
    {
        glBegin(GL_LINE_STRIP);
        for (auto && p : v)
        {
            glVertex2f(p.x(), p.y());
        }
        glVertex2f(v[0].x(), v[0].y());
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
            glVertex2f(p.x(), p.y());
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
            glVertex2f(p.x(), p.y());
        }
        glEnd();
    }

    // moving objects:
    {
        lock_guard<mutex> lk(mtx_);
        for (auto a : actors_) // auto && a : actors_   causes flickering
        {
            if (0 == a(3,3)) // car
            {
                glColor3f(0,1,0);
            }
            else if (1 == a(3,3)) // walker
            {
                glColor3f(0,0,1);
            }
            a(3,3) = 1.0f;
            Eigen::Vector3f bbox = a.block(3,0,1,3).transpose();
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
