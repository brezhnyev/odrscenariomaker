#include "CanvasXODR.h"

#include "odr_1_5.hpp"
#include "odrparser/odrparser.h"

#include <eigen3/Eigen/Eigen>
#include <GL/gl.h>

#include <set>
#include <iostream>
#include <deque>

using namespace odr_1_5;
using namespace odr;
using namespace std;
using namespace Eigen;

static set<int> validroads =
{45, 6, 41, 1400, 1401, 40, 1184, 1185, 39, 1091, 1092, 38, 1601, 1602, 37, 760, 761, 36, 861,
862, 35, 43, 266, 267, 42, 50, 1173, 1173, 49, 901, 902, 48, 774, 775, 47, 1072, 1073, 46, 144, 145};

std::vector<LaneElementBBox<double>> CanvasXODR::s_lboxes;

CanvasXODR::CanvasXODR(string xodrfile)
{
    OpenDRIVEFile ODR;
    loadFile(xodrfile, ODR);

    for (auto && odr_road : ODR.OpenDRIVE1_5->sub_road)
    {
        double S = 0; // total length of road
        uint32_t sindex = 0; // sections index (for storing for rendering)

        auto sit = odr_road.sub_lanes->sub_laneSection.begin(); // sections iterator
        for (auto && odr_subroad : odr_road.sub_planView->sub_geometry)
        {
            // starting matrix for this segment:
            Eigen::Matrix4d M; M.setIdentity();
            M.block(0,0,3,3) = Eigen::AngleAxisd(*odr_subroad._hdg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            M.block(0,3,3,1) = Eigen::Vector3d(*odr_subroad._x, *odr_subroad._y, 0.0);
            Eigen::Vector4d velocity; velocity.setZero();
            Eigen::Vector4d normal; normal.setZero();

            auto buildSubroad = [&](double s, double S)
            {
                Eigen::Vector4d P(0, 0, 0, 1);

                if (odr_subroad.sub_arc)
                {
                    double R = 1.0/(*odr_subroad.sub_arc->_curvature);
                    double radians = s / R;
                    M.block(0,0,3,3) = Eigen::AngleAxisd(*odr_subroad._hdg - M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

                    P = Eigen::Vector4d(R*cos(radians) - R, R*sin(radians), 0.0, 1.0);
                    // velocity as first derivative of position:
                    velocity.x() =-R*sin(radians);
                    velocity.y() = R*cos(radians);
                    if (R < 0) velocity = -velocity;
                }
                else if (odr_subroad.sub_line)
                {
                    P = Eigen::Vector4d(s, 0.0, 0.0, 1.0);
                    // velocity:
                    velocity.x() = 1;
                    velocity.y() = 0;
                }
                else if (odr_subroad.sub_paramPoly3 && odr_subroad._length)
                {
                    auto poly3 = odr_subroad.sub_paramPoly3;
                    double t = s / (*odr_subroad._length);
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
                    return;
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

                // Super-elevation:
                if (odr_road.sub_lateralProfile && !odr_road.sub_lateralProfile->sub_superelevation.empty())
                {
                    double roll = polyInter(S, odr_road.sub_lateralProfile->sub_superelevation, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_lateralProfile->sub_superelevation[0])>(it)->_s; });
                    M.block(0,0,3,3) = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix()*M.block(0,0,3,3);
                }
                // Elevation:
                if (odr_road.sub_elevationProfile && !odr_road.sub_elevationProfile->sub_elevation.empty())
                    P.z() = polyInter(S, odr_road.sub_elevationProfile->sub_elevation, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_elevationProfile->sub_elevation[0])>(it)->_s; });

                //P = P - Eigen::Vector4d(520, 87, 0, 0);
                // left normal:
                normal.x() =-velocity.y();
                normal.y() = velocity.x();
                normal.normalize();

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
                        vizBoundary[*odr_road._id][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z());
                    }
                auto buildLane = [&](auto & odr_sublane, int dir, double & twidth)
                {
                    // Boundary:
                    double width = polyInter(S - *odr_lane._s, odr_sublane.sub_width, [](void * it) ->double { return *static_cast<decltype(&odr_sublane.sub_width[0])>(it)->_sOffset; });
                    twidth += dir*width;
                    Eigen::Vector4d Ptrf = M * (P + normal*twidth);
                    vizBoundary[*odr_road._id][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z());
                    // Center:
                    Ptrf = M * (P + normal*(twidth - dir*width/2));
                    vizCenter[*odr_road._id][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z());
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
            for (double s = 0; s < *odr_subroad._length; s++, S++)
            {
                buildSubroad(s, S);
            }
            S = prevS + *odr_subroad._length;
            buildSubroad(*odr_subroad._length, S);
        }
    }
}

CanvasXODR::~CanvasXODR()
{
    glDeleteLists(listRoad, 1);
    glDeleteLists(listBounadries, 1);
    glDeleteLists(listCenterlines, 1);
}

void CanvasXODR::init()
{
    // build the BBoxes for the lane elements only for specific roads in Town04!!!
    for (auto && r : vizBoundary)
    {
        if (validroads.find(r.first) != validroads.end())
        {
            for (auto && s : r.second)
            {
                for (auto it1 = s.second.begin(); it1 != s.second.end(); ++it1)
                {
                    auto it2 = it1;
                    advance(it2, 1);
                    if (it2 == s.second.end()) break;

                    deque<Vector3d> buff;

                    for (size_t i = 0; i < min(it1->second.size(), it2->second.size()); ++i)
                    {
                        buff.emplace_back(it1->second[i]);
                        buff.emplace_back(it2->second[i]);
                    
                        if (buff.size() == 4)
                        {
                            LaneElementBBox<double> bbox;
                            bbox.addPoint(buff[0]);
                            bbox.addPoint(buff[1]);
                            bbox.addPoint(buff[2]);
                            bbox.addPoint(buff[3]);
                            // lanes are: .. -2, -1, 1, 2 .. we need: ..-2, -1, 0, 1
                            bbox.laneID = it1->first > 0 ? it1->first - 1 : it1->first;
                            // slightly increase the Z range:
                            bbox.minZ-=1.0;
                            bbox.maxZ+=1.0;
                            s_lboxes.push_back(bbox);
                            buff.pop_front();
                            buff.pop_front();
                        }
                    }
                }
            }
        }
    }
    // roads:
    listRoad = glGenLists(1);
    glNewList(listRoad, GL_COMPILE);
    for (auto && r : vizBoundary)
    {
        for (auto && s : r.second)
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
    glEndList();

    // draw boundaries:
    listBounadries = glGenLists(1);
    glNewList(listBounadries, GL_COMPILE);
    glPushMatrix();
    glTranslatef(0,0,0.1f);

    for (auto && r : vizBoundary)
    {
        for (auto && s : r.second)
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
    glEndList();

    // draw centerlines:
    listCenterlines = glGenLists(1);
    glNewList(listCenterlines, GL_COMPILE);
    for (auto && r : vizCenter)
    {
        for (auto && s : r.second)
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
    glPopMatrix();

    glEndList();
}

void CanvasXODR::draw()
{
    glColor3f(0.5f, 0.5f, 0.5f);
    glCallList(listRoad);
    glColor3f(1.0f, 1.0f, 1.0f);
    glCallList(listBounadries);
    glColor3f(0.75,0.75,1);
    glCallList(listCenterlines);
}

void CanvasXODR::drawWithNames()
{
    glPushName(m_id);
    glCallList(listRoad);
    glPopName();
}

int CanvasXODR::getLaneID(Eigen::Vector3d p)
{
    for (auto && b : s_lboxes)
    {
        if (b.isPointInside(p))
            return b.laneID;
    }
    return 1000;
}