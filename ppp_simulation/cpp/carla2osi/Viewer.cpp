#include <Viewer.h>

void Viewer::init()
{
    camera()->setSceneRadius(100);
    camera()->fitSphere(qglviewer::Vec(0, 0, 0), 100);
    resize(800,600);
}

void Viewer::addDataStatic(std::vector<Eigen::Vector2f> && v)
{
    dataStatic_.push_back(v);
}

void Viewer::updateDataRoads(std::vector<std::vector<Eigen::Vector2f>> && centerlines, std::vector<std::vector<Eigen::Vector2f>> && boundaries)
{
    centerlines_ = centerlines;
    boundaries_ = boundaries;
}

void Viewer::updateMovingObjects(std::vector<Eigen::Matrix4f> v)
{
    actors_ = v;
    update();
}

void Viewer::draw()
{
    // static
    glDisable(GL_LIGHTING);
    glColor3f(1,1,1);
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
    // centerlines
    glLineWidth(1);
    glColor3f(1,1,0.5);
    for (auto && v : centerlines_)
    {
        glBegin(GL_LINE_STRIP);
        for (auto && p : v)
        {
            glVertex2f(p.x(), p.y());
        }
        glEnd();
    }

    // boundaries
    // centerlines
    glLineWidth(3);
    glColor3f(1,1,0);
    for (auto && v : boundaries_)
    {
        glBegin(GL_LINE_STRIP);
        for (auto && p : v)
        {
            glVertex2f(p.x(), p.y());
        }
        glEnd();
    }

    glLineWidth(2);
    glColor3f(0,1,0);
    for (auto && a : actors_)
    {
        if (0 == a(3,3)) // car
        {
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

            glPopMatrix();
        }
    }
}
