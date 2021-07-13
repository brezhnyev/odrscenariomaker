#include <Viewer.h>

void Viewer::init()
{
    camera()->setSceneRadius(100);
    camera()->fitSphere(qglviewer::Vec(0, 0, 0), 100);
    resize(1920,1080);
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
    glColor3f(0,1,1);
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
}

void Viewer::updateMovingObjects(std::vector<Eigen::Matrix4f> && v)
{
    glPointSize(5);
    for (auto && o : v)
    {
        if (0 == o(3,3)) // car
        {
            glBegin(GL_POINTS);
            glVertex3f(o(0,3), o(1,3), o(2,3));
            glEnd();
        }
    }
}