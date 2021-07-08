#include <Viewer.h>


void Viewer::addData(std::vector<Eigen::Vector2f> && v)
{
    data.push_back(v);
}

void Viewer::draw()
{
    glDisable(GL_LIGHTING);
    for (auto && v : data)
    {
        glBegin(GL_LINE_STRIP);
        for (auto && p : v)
        {
            glVertex2f(p.x(), p.y());
        }
        glVertex2f(v[0].x(), v[0].y());
        glEnd();
    }
}
void Viewer::init()
{
    camera()->setSceneRadius(100);
    camera()->fitSphere(qglviewer::Vec(0, 0, 0), 100);
    resize(1920,1080);
}