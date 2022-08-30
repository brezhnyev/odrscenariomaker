#include "CanvasXODR.h"

#include <eigen3/Eigen/Eigen>

#include <set>
#include <iostream>
#include <deque>
#include <omp.h>

using namespace odr_1_5;
using namespace odr;
using namespace std;
using namespace Eigen;
using namespace tinyxml2;


CanvasXODR::CanvasXODR(const string & xodrfile, float xodrResolution) : mXodrResolution(xodrResolution), m_xodrBuilder(xodrfile, xodrResolution), mXodrFile(xodrfile)
{
}

CanvasXODR::~CanvasXODR()
{
    for (auto && e : mListID2LaneMap)
    {
        glDeleteLists(e.first, 1);
    }
    glDeleteLists(listBounadries, 1);
    glDeleteLists(listCenterlines, 1);
}

void CanvasXODR::init()
{
    // roads:
    for (auto && r : m_xodrBuilder.getBoundaries())
    {
        for (auto && geom : r.second)
        {
            for (auto && s : geom.second)
            {
                for (auto it1 = s.second.begin(); it1 != s.second.end(); ++it1)
                {
                    auto it2 = it1;
                    advance(it2, 1);
                    if (it2 == s.second.end()) break;

                    LaneElementBBox<Vector3d> bbox;
                    uint listid = glGenLists(1);
                    glNewList(listid, GL_COMPILE);
                    glBegin(GL_TRIANGLE_STRIP);
                    for (size_t i = 0; i < min(it1->second.size(), it2->second.size()); ++i)
                    {
                        glVertex3f(it1->second[i].x(), it1->second[i].y(), it1->second[i].z());
                        glVertex3f(it2->second[i].x(), it2->second[i].y(), it2->second[i].z());
                        bbox.addPoint(it1->second[i].block(0,0,3,1));
                        bbox.addPoint(it2->second[i].block(0,0,3,1));
                    }
                    glEnd();
                    glEndList();
                    bbox.xodrid.roadID = r.first;
                    bbox.xodrid.geoID= geom.first;
                    bbox.xodrid.sectID = s.first;
                    bbox.xodrid.laneID = it1->first >= 0 ? it1->first + 1 : it1->first;
                    bbox.minZ -= 0.1;
                    bbox.maxZ += 0.1;
                    mListID2LaneMap[listid] = bbox;
                }
            }
        }
    }

    // get the BBox of the whole scene:
    for (auto && it : mListID2LaneMap)
    {
        auto lanebb = it.second;
        mSceneBB.addPoint(Vector3d(lanebb.minX,lanebb.minY,lanebb.minZ));
        mSceneBB.addPoint(Vector3d(lanebb.minX,lanebb.minY,lanebb.maxZ));
        mSceneBB.addPoint(Vector3d(lanebb.minX,lanebb.maxY,lanebb.minZ));
        mSceneBB.addPoint(Vector3d(lanebb.minX,lanebb.maxY,lanebb.maxZ));
        mSceneBB.addPoint(Vector3d(lanebb.maxX,lanebb.minY,lanebb.minZ));
        mSceneBB.addPoint(Vector3d(lanebb.maxX,lanebb.minY,lanebb.maxZ));
        mSceneBB.addPoint(Vector3d(lanebb.maxX,lanebb.maxY,lanebb.minZ));
        mSceneBB.addPoint(Vector3d(lanebb.maxX,lanebb.maxY,lanebb.maxZ));
    }

    // draw boundaries:
    listBounadries = glGenLists(1);
    glNewList(listBounadries, GL_COMPILE);
    glPushMatrix();
    glTranslatef(0,0,0.1f);

    for (auto && r : m_xodrBuilder.getBoundaries())
    {
        for (auto && geom : r.second)
        {
            for (auto && s : geom.second)
            {
                for (auto && l : s.second)
                {
                    glBegin(GL_LINE_STRIP);
                    for (auto && p : l.second)
                    {
                        glVertex3f(p.x(), p.y(), p.z());
                    }
                    glEnd();
                }
            }
        }
    }
    glEndList();

    // draw centerlines:
    listCenterlines = glGenLists(1);
    glNewList(listCenterlines, GL_COMPILE);
    for (auto && r : m_xodrBuilder.getCenters())
    {
        for (auto && geom : r.second)
        {
            for (auto && s : geom.second)
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
}

double CanvasXODR::getSceneRadius()
{
    return 0.5*(Eigen::Vector3d(mSceneBB.minX, mSceneBB.minY, mSceneBB.minZ) - Eigen::Vector3d(mSceneBB.maxX, mSceneBB.maxY, mSceneBB.maxZ)).norm();
}

void CanvasXODR::drawSelectable(uint32_t listid)
{
    glCallList(listid);
}

void CanvasXODR::drawUnSelectable()
{
    glLineWidth(2);
    glPointSize(1);
    glColor3f(1.0f, 1.0f, 1.0f);
    glCallList(listBounadries);
    glColor3f(0.75,0.75,1);
    glCallList(listCenterlines);
    // draw the highlighted lane:
    glColor3f(1.0f, 0.0f, 0.0f);
    glCallList(mSelectedLane);
    // draw selected lanes:
    glColor3f(1.0f,1.0f,0.0f);
    for (auto && indx : mSelectedBoxes)
        glCallList(indx);
}

void CanvasXODR::draw()
{
    drawUnSelectable();

    // glColor3f(0.5f, 0.5f, 0.5f);
    // for (auto && e : mListID2LaneMap)
    // {
    //     drawSelectable(e.first);
    // }
}

void CanvasXODR::drawWithNames()
{
    for (auto && e : mListID2LaneMap)
    {
        glPushName(e.first);
        drawSelectable(e.first);
        glPopName();
    }
}

xodrid_t CanvasXODR::highlightSelected(uint32_t glid, Vector3d p)
{
    mSelectedLane = glid;
    return mListID2LaneMap[glid].xodrid;
}

void CanvasXODR::highlightSelected(uint32_t xodrid)
{
    mSelectedBoxes.clear();
    for (auto && bbox : mListID2LaneMap)
    {
        if (bbox.second.xodrid.roadID == xodrid)
            mSelectedBoxes.push_back(bbox.first);
    }
}

void CanvasXODR::highlightRibbonSelection(const Vector3d & start, const Vector3d & end)
{
    BBox<Vector3d> selBox(start.x(), end.x(), start.y(), end.y(), mSceneBB.minZ, mSceneBB.maxZ);
    mSelectedBoxes.clear();
    set<uint32_t> selectedRoads;
    for (auto && it : mListID2LaneMap)
    {
        if (it.second.isCrossingOther(selBox))
        {
            mSelectedBoxes.push_back(it.first);
            selectedRoads.insert(it.second.xodrid.roadID);
        }
    }

    XMLDocument doc;
    doc.LoadFile(mXodrFile.c_str());
    XMLElement * pRootElement = doc.RootElement();
    if (mXodrFile.empty() || !pRootElement)
    {
        cerr << "Could not load XODR file" << endl;
        return;
    }
    XMLElement * pElement = pRootElement->FirstChildElement("road");
    // first make sure that the selected connecting roads will always have the in/out pair:
    while (pElement)
    {
        if (string(pElement->Name()) == "road")
        {
            int32_t id = atoi(pElement->Attribute("id"));
            int32_t jid = atoi(pElement->Attribute("junction"));
            if (selectedRoads.find(id) != selectedRoads.end() && jid != -1) // i.e. we deal with connecting road
            {
                XMLElement * pLinkElement = pElement->FirstChildElement("link");
                if (pLinkElement)
                {
                    XMLElement * pSuccessor = pLinkElement->FirstChildElement("successor");
                    if (pSuccessor && string(pSuccessor->Attribute("elementType")) == "road")
                    {
                        uint32_t iid = atoi(pSuccessor->Attribute("elementId"));
                        if (selectedRoads.find(iid) == selectedRoads.end())
                        {
                            selectedRoads.insert(iid);
                            cout << "Road " << iid << " added to exported roads, since it is incoming road for a connecting road "  << id << endl;
                        }
                    }
                    XMLElement * pPredecessor = pLinkElement->FirstChildElement("predecessor");
                    if (pPredecessor && string(pPredecessor->Attribute("elementType")) == "road")
                    {
                        uint32_t iid = atoi(pPredecessor->Attribute("elementId"));
                        if (selectedRoads.find(iid) == selectedRoads.end())
                        {
                            selectedRoads.insert(iid);
                            cout << "Road " << iid << " added to exported roads, since it is incoming road for a connecting road " << id << endl;
                        }
                    }
                }
            }
        }
        pElement = pElement->NextSiblingElement();
    }
    // remove unselected roads:
    pElement = pRootElement->FirstChildElement("road");
    while (pElement)
    {
        XMLElement * pToRemove = pElement;
        pElement = pElement->NextSiblingElement();
        if (string(pToRemove->Name()) == "road")
        {
            uint32_t id = atoi(pToRemove->Attribute("id"));
            if (selectedRoads.find(id) == selectedRoads.end())
            {
                pToRemove->Parent()->DeleteChild(pToRemove);
            }
        }
    }
    // find all referenced junctions:
    set<uint32_t> junctions;
    pElement = pRootElement->FirstChildElement("road");
    while (pElement)
    {
        XMLElement * pToRemove = pElement;
        pElement = pElement->NextSiblingElement();
        if (string(pToRemove->Name()) == "road")
        {
            int32_t jid = atoi(pToRemove->Attribute("junction"));
            if (jid != -1)
            {// connecting road
                junctions.insert(jid);
                continue;
            }
            // else
            XMLElement * pLinkElement = pToRemove->FirstChildElement("link");
            if (pLinkElement)
            {
                XMLElement * pSuccessor = pLinkElement->FirstChildElement("successor");
                if (pSuccessor && string(pSuccessor->Attribute("elementType")) == "junction")
                    junctions.insert(atoi(pSuccessor->Attribute("elementId")));
                XMLElement * pPredecessor = pLinkElement->FirstChildElement("predecessor");
                if (pPredecessor && string(pPredecessor->Attribute("elementType")) == "junction")
                    junctions.insert(atoi(pPredecessor->Attribute("elementId")));
            }
        }
    }
    // remove all unreferenced junctions:
    pElement = pRootElement->FirstChildElement("junction");
    while (pElement)
    {
        XMLElement * pToRemove = pElement;
        pElement = pElement->NextSiblingElement();
        if (string(pToRemove->Name()) == "junction")
        {
            uint32_t jid = atoi(pToRemove->Attribute("id"));
            if (junctions.find(jid) == junctions.end())
                pToRemove->Parent()->DeleteChild(pToRemove);
        }
    }
    // for those remaining junctions remove unreferneced connections inside the junctions:
    pElement = pRootElement->FirstChildElement("junction");
    while (pElement)
    {
        if (string(pElement->Name()) == "junction")
        {
            XMLElement * pToRemove = pElement;
            pElement = pElement->NextSiblingElement();
            XMLElement *pConnection = pToRemove->FirstChildElement("connection");
            while (pConnection)
            {
                XMLElement *pToRemove = pConnection;
                pConnection = pConnection->NextSiblingElement();
                if (string(pToRemove->Name()) == "connection")
                {
                    uint32_t cid = atoi(pToRemove->Attribute("connectingRoad"));
                    if (selectedRoads.find(cid) == selectedRoads.end())
                        pToRemove->Parent()->DeleteChild(pToRemove);
                }
            }
            if (!pToRemove->FirstChildElement("connection"))
            {
                junctions.erase(atoi(pToRemove->Attribute("id")));
                pToRemove->Parent()->DeleteChild(pToRemove);
            }
        }
    }

    // for the remaining roads remove the unreferenced predecessor/successor junctions:
    pElement = pRootElement->FirstChildElement("road");
    while (pElement)
    {
        if (string(pElement->Name()) == "road")
        {
            XMLElement *pLinkElement = pElement->FirstChildElement("link");
            if (pLinkElement)
            {
                auto removeLink = [&](XMLElement *pLink, pair<string, set<uint32_t>&> where, string link)
                {
                    if (pLink && string(pLink->Attribute("elementType")) == where.first)
                    {
                        uint32_t id = atoi(pLink->Attribute("elementId"));
                        if (where.second.find(id) == where.second.end())
                        {
                            pLink->Parent()->DeleteChild(pLink);
                            // for all lanes remove the successor or predecessor:
                            XMLElement *pLane = pElement->FirstChildElement("lanes");
                            if (pLane)
                            {
                                XMLNode *pSection = pLane->FirstChildElement("laneSection");
                                removeLinks(pSection, link);
                            }
                        }
                    }
                };
                vector<string> links = {"successor", "predecessor"};
                for (auto &&link : links)
                {
                    removeLink(pLinkElement->FirstChildElement(link.c_str()), pair<string, set<uint32_t>&>("road", selectedRoads), link);
                    removeLink(pLinkElement->FirstChildElement(link.c_str()), pair<string, set<uint32_t>&>("junction", junctions), link);
                }
            }
        }
        pElement = pElement->NextSiblingElement();
    }
    doc.SaveFile((mXodrFile + "_cropped.xodr").c_str());
    cout << "Exported cropped file " << mXodrFile << "_cropped.xodr" << endl;
}

void CanvasXODR::removeLinks(XMLNode * node, const string &name)
{
    if (!node)
        return;
    XMLElement *element = node->ToElement();
    if (string(element->Name()) == name)
        element->Parent()->DeleteChild(element);
    else
    {
        removeLinks(element->FirstChild(), name);
        removeLinks(element->NextSiblingElement(), name);
    }
}