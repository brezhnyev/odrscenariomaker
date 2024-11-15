#include "CanvasXODR.h"

#include <eigen3/Eigen/Eigen>
#include <GL/gl.h>

#include <set>
#include <iostream>
#include <deque>

using namespace std;
using namespace Eigen;

CanvasXODR::CanvasXODR(string xodrfile) : Selectable(nullptr), m_xodrBuilder(xodrfile, 1.0f)
{
}

CanvasXODR::~CanvasXODR()
{
    glDeleteLists(listRoad, 1);
    glDeleteLists(listBounadries, 1);
    glDeleteLists(listCenterlines, 1);
}

void CanvasXODR::init()
{
    m_sceneBbox.first[0] = __DBL_MAX__;
    m_sceneBbox.first[1] = __DBL_MAX__;
    m_sceneBbox.first[2] = __DBL_MAX__;
    m_sceneBbox.second[0] = -__DBL_MAX__;
    m_sceneBbox.second[1] = -__DBL_MAX__;
    m_sceneBbox.second[2] = -__DBL_MAX__;

    // prepare boundaries for graphic card:
    listBounadries = glGenLists(1);
    glNewList(listBounadries, GL_COMPILE);
    glPushMatrix();
    glTranslatef(0,0,0.1f);
    for (auto && r : m_xodrBuilder.getBoundaries())
    {
        for (auto && g : r.second)
        {
            for (auto && s : g.second)
            {
                for (auto && l : s.second)
                {
                    glBegin(GL_LINE_STRIP);
                    for (auto && p : l.second)
                    {
                        glVertex3f(p.x(), p.y(), p.z());
                        addToSceneBbox(p);
                    }
                    glEnd();
                }
            }
        }
    }
    glPopMatrix();
    glEndList();

    // prepare centerlines for graphic card:
    listCenterlines = glGenLists(1);
    glNewList(listCenterlines, GL_COMPILE);
    glPushMatrix();
    glTranslatef(0,0,0.1f);
    for (auto && r : m_xodrBuilder.getCenters())
    {
        for (auto && g : r.second)
        {
            for (auto && s : g.second)
            {
                for (auto && l : s.second)
                {
                    glBegin(GL_POINTS);
                    for (auto && p : l.second)
                    {
                        glVertex3f(p.x(), p.y(), p.z());
                    }
                    glEnd();
                }
            }
        }
    }
    glPopMatrix();
    glEndList();

    // roads, come as last otherwise may affect visualization of boundaries (was the case on intel card):
    listRoad = glGenLists(1);
    glNewList(listRoad, GL_COMPILE);
    for (auto && r : m_xodrBuilder.getBoundaries())
    {
        for (auto && g : r.second)
        {
            for (auto && s : g.second)
            {
                for (auto it1 = s.second.begin(); it1 != s.second.end(); ++it1)
                {
                    auto it2 = it1;
                    advance(it2, 1);
                    if (it2 == s.second.end()) break;

                    glBegin(GL_TRIANGLE_STRIP);
                    for (size_t i = 0; i < min(it1->second.size(), it2->second.size()); ++i)
                    {
                        glVertex3f(it1->second[i].x(), it1->second[i].y(), it1->second[i].z());
                        glVertex3f(it2->second[i].x(), it2->second[i].y(), it2->second[i].z());
                    }
                    glEnd();
                }
            }
        }
    }
    glEndList();
}

void CanvasXODR::draw() const
{
    glColor3f(0.5f, 0.5f, 0.5f);
    glCallList(listRoad);
    glColor3f(1.0f, 1.0f, 1.0f);
    glCallList(listBounadries);
    glColor3f(0.75,0.75,1);
    glCallList(listCenterlines);
}

void CanvasXODR::drawWithNames() const
{
    glPushName(m_id);
    glCallList(listRoad);
    glPopName();
}