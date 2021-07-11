#include <Viewer.h>


void Viewer::addDataStatic(std::vector<Eigen::Vector2f> && v)
{
    dataStatic.push_back(v);
}

void Viewer::updateDataRoads(std::vector<std::vector<Eigen::Vector2f>> && v)
{
    dataRoads = v;
}

void Viewer::draw()
{
    glDisable(GL_LIGHTING);
    glColor3f(1,1,1);
    glLineWidth(2);
    for (auto && v : dataStatic)
    {
        glBegin(GL_LINE_STRIP);
        for (auto && p : v)
        {
            glVertex2f(p.x(), p.y());
        }
        glVertex2f(v[0].x(), v[0].y());
        glEnd();
    }
    glColor3f(1,1,0);
    for (auto && v : dataRoads)
    {
        glBegin(GL_LINE_STRIP);
        for (auto && p : v)
        {
            glVertex2f(p.x()-520, p.y()-90);
        }
        glEnd();
    }
    glPointSize(4);
    for (auto && v : dataRoads)
    {
        glBegin(GL_POINTS);
        for (auto && p : v)
        {
            glVertex2f(p.x()-520, p.y()-90);
        }
        glEnd();
    }
}
void Viewer::init()
{
    camera()->setSceneRadius(100);
    camera()->fitSphere(qglviewer::Vec(0, 0, 0), 100);
    resize(1920,1080);
}