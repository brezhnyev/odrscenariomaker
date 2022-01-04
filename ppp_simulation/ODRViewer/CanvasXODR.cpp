#include "CanvasXODR.h"

#include "odr_1_5.hpp"
#include "odrparser/odrparser.h"

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


CanvasXODR::CanvasXODR(const string & xodrfile, float xodrResolution) : mXodrResolution(xodrResolution)
{
    if (parseXodr(xodrfile))
        init();
}

//                     sub-road
// |-------------------------------------------------------------|
//
//                   geometry 1                      geometry 2          sub_planView.sub_geometry
// | --------------------------------------------|---------------|


//
//                                              /               /
//                                             /               /
//                                            /               /
// ........................................../               /
//                                                          /
//                                                .        /
//                                                        /
//                                                       /
// ...................................................../
//                          .
//                    .
//                .          <----- one of two lanes ends (its width will change according to poly function)
//             .  
// .......


// |------------------------|------------------------------------|
//  lane secion 1 (two lanes)    lane section 2 (one lane)

bool CanvasXODR::parseXodr(const string & xodrfile)
{
    mXodrFile = xodrfile;
    OpenDRIVEFile ODR;
    if (!loadFile(xodrfile, ODR))
        return false;

    for (auto && odr_road : ODR.OpenDRIVE1_5->sub_road)
    {
        double S = 0; // total length of road
        uint32_t sindex = 0; // sections index (for storing for rendering)

        // skip of no lanes available
        if (!odr_road.sub_lanes)
            continue;

        // skip if lanes empty
        if (odr_road.sub_lanes->sub_laneSection.empty())
            continue;

        auto sit = odr_road.sub_lanes->sub_laneSection.begin(); // sections iterator
        int geom = 0;
        for (auto && odr_subroad : odr_road.sub_planView->sub_geometry)
        {
            // starting matrix for this segment:
            Eigen::Matrix4d M; M.setIdentity();
            M.block(0,0,3,3) = Eigen::AngleAxisd(*odr_subroad._hdg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            M.block(0,3,3,1) = Eigen::Vector3d(*odr_subroad._x, *odr_subroad._y, 0.0);
            Eigen::Matrix3d RM; // remember the rotational part, cause will be changed by superelevation:
            RM = M.block(0,0,3,3);
            Eigen::Vector3d velocity(1.0,0.0,0.0);
            Eigen::Vector4d normal(0.0,1.0,0.0,0.0);
            Eigen::Vector4d P(0, 0, 0, 1);

            auto buildSubroad = [&](double s, double S)
            {

                if (odr_subroad.sub_arc)
                {
                    double R = 1.0/(*odr_subroad.sub_arc->_curvature);
                    double radians = s / R;
                    M.block(0,0,3,3) = Eigen::AngleAxisd(*odr_subroad._hdg - M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    RM = M.block(0,0,3,3);
                    P = Eigen::Vector4d(R*cos(radians) - R, R*sin(radians), 0.0, 1.0); // found from trial and error
                    // velocity as first derivative of position:
                    velocity.x() =-R*sin(radians);
                    velocity.y() = R*cos(radians);
                    if (R < 0) velocity = -velocity; // found from trial and error
                }
                else if (odr_subroad.sub_line)
                {
                    P = Eigen::Vector4d(s, 0.0, 0.0, 1.0);
                    // velocity as first derivative of position:
                    velocity.x() = 1;
                    velocity.y() = 0;
                }
                else if (odr_subroad.sub_paramPoly3 && odr_subroad._length)
                {
                    auto poly3 = odr_subroad.sub_paramPoly3;
                    double t = s;
                    if (!odr_subroad.sub_paramPoly3->_pRange || *odr_subroad.sub_paramPoly3->_pRange == "normalized") t /= (*odr_subroad._length);
                    // else "arcLength" and t = s
                    P = Eigen::Vector4d(
                        *poly3->_aU + *poly3->_bU * t + *poly3->_cU * t * t + *poly3->_dU * t * t * t,
                        *poly3->_aV + *poly3->_bV * t + *poly3->_cV * t * t + *poly3->_dV * t * t * t,
                        0.0,
                        1.0);
                    // velocity as first derivative of position:
                    velocity.x() = *poly3->_bU + *poly3->_cU * 2 * t + *poly3->_dU * 3 * t * t;
                    velocity.y() = *poly3->_bV + *poly3->_cV * 2 * t + *poly3->_dV * 3 * t * t;
                }
                else if (odr_subroad.sub_poly3 && odr_subroad._length)
                {
                    auto poly3 = odr_subroad.sub_paramPoly3;
                    P = Eigen::Vector4d(
                        s,
                        *poly3->_aV + *poly3->_bV * s + *poly3->_cV * s * s + *poly3->_dV * s * s * s,
                        0.0,
                        1.0);
                    // velocity as first derivative of position:
                    velocity.x() = 1;
                    velocity.y() = *poly3->_bV + *poly3->_cV * 2 * s + *poly3->_dV * 3 * s * s;
                }
                else if (odr_subroad.sub_spiral)
                {
                    double t = s/(*odr_subroad._length);
                    double curvs = *odr_subroad.sub_spiral->_curvStart;
                    double curve = *odr_subroad.sub_spiral->_curvEnd;
                    double curvature = (1.0 - t)*curvs + t*curve;
                    if (abs(curvature) < 1e-10) curvature = curvature < 0 ? -1e-10 : 1e-10;
                    double R = 1.0/curvature;
                    Vector3d P2center = R*normal.block(0,0,3,1);
                    Vector3d center = P.block(0,0,3,1) + P2center;
                    auto Rot = AngleAxisd(curvature*mXodrResolution, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    Vector3d center2pnext = Rot*(-P2center);
                    Vector3d nextP = center + center2pnext;
                    velocity.block(0,0,3,1) = (AngleAxisd(curvature < 0 ? -M_PI_2 : M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix()*center2pnext).normalized();
                    P.block(0,0,3,1) = nextP;
                }
                else
                    return;


                auto polyInter = [&](double T, auto & polis, double (*getS)(void * itt) ) -> double
                {
                    auto pit = polis.begin(); // polis iterator
                    while (pit != polis.end() && getS(&(*pit)) < T) ++pit;
                    if (pit != polis.begin()) --pit;
                    double t = T - getS(&(*pit));
                    return *(pit->_a) + *(pit->_b)*t + *(pit->_c)*t*t + *(pit->_d)*t*t*t;
                };

                // Super-elevation (not tested):
                // if (odr_road.sub_lateralProfile && !odr_road.sub_lateralProfile->sub_superelevation.empty())
                // {
                //     double roll = polyInter(S, odr_road.sub_lateralProfile->sub_superelevation, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_lateralProfile->sub_superelevation[0])>(it)->_s; });
                //     M.block(0,0,3,3) = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix()*RM;
                // }
                // Elevation:
                if (odr_road.sub_elevationProfile && !odr_road.sub_elevationProfile->sub_elevation.empty())
                    P.z() = polyInter(S, odr_road.sub_elevationProfile->sub_elevation, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_elevationProfile->sub_elevation[0])>(it)->_s; });

                // left normal:
                normal.x() =-velocity.y();
                normal.y() = velocity.x();
                normal.normalize();

                // heading:
                Vector3d velocityGlobal = M.block(0,0,3,3) * Vector3d(velocity.x(), velocity.y(), 0);
                double heading = atan2(velocityGlobal.y(), velocityGlobal.x());

                // find out if we are at the proper section of the road:
                ++sit; ++sindex;
                if (sit == odr_road.sub_lanes->sub_laneSection.end() || *sit->_s > S) { --sit; --sindex; }
                auto && odr_lane = *sit;

                // set the lanes:
                double offset = 0.0;
                if (odr_road.sub_lanes && !odr_road.sub_lanes->sub_laneOffset.empty())
                    offset = polyInter(S, odr_road.sub_lanes->sub_laneOffset, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_lanes->sub_laneOffset[0])>(it)->_s; });
                
                if (odr_lane.sub_center)
                    for (auto && odr_sublane : odr_lane.sub_center->sub_lane)
                    {
                        // ONLY 1 center line, actually loop not needed
                        Eigen::Vector4d Ptrf = M * (P + normal*offset);
                        vizBoundary[*odr_road._id][geom][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
                    }
                auto buildLane = [&](auto & odr_sublane, int dir, double & twidth)
                {
                    // Boundary:
                    double width = polyInter(S - *odr_lane._s, odr_sublane.sub_width, [](void * it) ->double { return *static_cast<decltype(&odr_sublane.sub_width[0])>(it)->_sOffset; });
                    twidth += dir*width;
                    Eigen::Vector4d Ptrf = M * (P + normal*twidth);
                    vizBoundary[*odr_road._id][geom][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
                    // Center:
                    Ptrf = M * (P + normal*(twidth - dir*width/2));
                    vizCenter[*odr_road._id][geom][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
                };
                double twidth = offset;
                if (odr_lane.sub_left)
                {
                    map<int, t_road_lanes_laneSection_left_lane> sublanes_map;
                    for (auto && odr_sublane : odr_lane.sub_left->sub_lane) sublanes_map[abs<int>(*odr_sublane._id)] = odr_sublane;
                    for (auto && it : sublanes_map) buildLane(it.second, 1, twidth);
                }
                twidth = offset;
                if (odr_lane.sub_right)
                {
                    map<int, t_road_lanes_laneSection_right_lane> sublanes_map;
                    for (auto && odr_sublane : odr_lane.sub_right->sub_lane) sublanes_map[abs<int>(*odr_sublane._id)] = odr_sublane;
                    for (auto && it : sublanes_map) buildLane(it.second, -1, twidth);
                }
            };
            double prevS = S;
            for (double s = 0; s < *odr_subroad._length; s += mXodrResolution, S += mXodrResolution)
            {
                buildSubroad(s, S);
            }
            S = prevS + *odr_subroad._length;
            buildSubroad(*odr_subroad._length, S);
            ++geom;
        }
    }
    return true;
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
    for (auto && r : vizBoundary)
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
                    bbox.glaneid.roadID = r.first;
                    bbox.glaneid.geoID= geom.first;
                    bbox.glaneid.sectID = s.first;
                    bbox.glaneid.laneID = it1->first >= 0 ? it1->first + 1 : it1->first;
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

    for (auto && r : vizBoundary)
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
    for (auto && r : vizCenter)
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

glaneid_t CanvasXODR::printLaneInfo(uint32_t id, Vector3d p)
{
    mSelectedLane = id;
    return mListID2LaneMap[id].glaneid;
}

void CanvasXODR::highlightRibbonSelection(const Vector3d & start, const Vector3d & end)
{
    BBox<Vector3d> selBox(start.x(), end.x(), start.y(), end.y(), -100, 100);
    mSelectedBoxes.clear();
    set<uint32_t> selectedRoads;
    for (auto && it : mListID2LaneMap)
    {
        if (it.second.isCrossingOther(selBox))
        {
            mSelectedBoxes.push_back(it.first);
            selectedRoads.insert(it.second.glaneid.roadID);
        }
    }

    XMLDocument doc;
    doc.LoadFile(mXodrFile.c_str());
    XMLElement * pRootElement = doc.RootElement();
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