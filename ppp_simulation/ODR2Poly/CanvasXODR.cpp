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


CanvasXODR::CanvasXODR(const string & xodrfile, float radius, float xodrResolution) : mRadius(radius), mXodrResolution(xodrResolution)
{
    parseXodr(xodrfile);
}

//                     sub-road
// |-------------------------------------------------------------|
//
//   geometry shape 1 (odr_subroad.sub_line)     geometry shape 2 (odr_subroad.sub_line)
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

void CanvasXODR::parseXodr(const string & xodrfile)
{
    OpenDRIVEFile ODR;
    loadFile(xodrfile, ODR);

    for (auto && odr_road : ODR.OpenDRIVE1_5->sub_road)
    {
        double S = 0; // total length of road
        uint32_t sindex = 0; // sections index (for storing for rendering)

        auto sit = odr_road.sub_lanes->sub_laneSection.begin(); // sections iterator
        int shape = 0;
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
                    cout << "Spiral geometry not implemented, skipping ..." << endl;
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
                        vizBoundary[*odr_road._id][shape][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
                    }
                auto buildLane = [&](auto & odr_sublane, int dir, double & twidth)
                {
                    // Boundary:
                    double width = polyInter(S - *odr_lane._s, odr_sublane.sub_width, [](void * it) ->double { return *static_cast<decltype(&odr_sublane.sub_width[0])>(it)->_sOffset; });
                    twidth += dir*width;
                    Eigen::Vector4d Ptrf = M * (P + normal*twidth);
                    vizBoundary[*odr_road._id][shape][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
                    // Center:
                    Ptrf = M * (P + normal*(twidth - dir*width/2));
                    vizCenter[*odr_road._id][shape][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
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
            ++shape;
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
    for (auto &&r : vizBoundary) // road
    {
        for (auto && shape : r.second)
        {
            for (auto && s : shape.second) // lane section
            {
                LaneElementBBox<Vector3d> bbox;
                for (auto it = s.second.begin(); it != s.second.end(); ++it) // lanes
                {
                    bbox.roadID = r.first;
                    bbox.shapeID= shape.first;
                    bbox.sectID = s.first;
                    for (size_t i = 0; i < it->second.size(); ++i) // lanes points
                    {
                        bbox.addPoint(it->second[i].block(0,0,3,1));
                    }
                }
                bbox.minZ -= 0.1;
                bbox.maxZ += 0.1;
                mLaneBoxes.push_back(bbox);
            }
        }
    }
    // roads:
    listRoad = glGenLists(1);
    glNewList(listRoad, GL_COMPILE);
    for (auto && r : vizBoundary)
    {
        for (auto && shape : r.second)
        {
            for (auto && s : shape.second)
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

    // draw boundaries:
    listBounadries = glGenLists(1);
    glNewList(listBounadries, GL_COMPILE);
    glPushMatrix();
    glTranslatef(0,0,0.1f);

    for (auto && r : vizBoundary)
    {
        for (auto && shape : r.second)
        {
            for (auto && s : shape.second)
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
        for (auto && shape : r.second)
        {
            for (auto && s : shape.second)
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

void CanvasXODR::drawSelectable()
{
    glLineWidth(2);
    glPointSize(1);
    glColor3f(0.5f, 0.5f, 0.5f);
    glCallList(listRoad);
    glColor3f(1.0f, 1.0f, 1.0f);
    glCallList(listBounadries);
    glColor3f(0.75,0.75,1);
    glCallList(listCenterlines);
}

void CanvasXODR::drawUnSelectable()
{
    // glColor3f(1.0f, 0.5f, 0.5f);
    // for (auto && b : mLocallLaneBoxes)
    // {
    //     glColor3f((float)rand()/__INT_MAX__, (float)rand()/__INT_MAX__, (float)rand()/__INT_MAX__);
    //     b.draw();
    // }
    glLineWidth(5);
    glPointSize(10);
    // visualize the Ego positino and direction;
    glColor3f(1.0f, 0.5f, 0.5f);
    glPushMatrix();
    glTranslated(mEgoTrf[0], mEgoTrf[1], mEgoTrf[2] + 0.2);
    glBegin(GL_POINTS);
    glVertex3f(0,0,0);
    glEnd();
    glRotatef(mEgoTrf[3]*180/M_PI, 0, 0, 1);
    glBegin(GL_LINES);
    glVertex3f(-2.0f,0.0f,0.0f); glVertex3f(2.0f,0.0f,0.0f);
    glEnd();
    glPopMatrix();
    // visualize all boundaries:
    float radius2 = mRadius*mRadius;
    Vector3d egoTrl = Vector3d(mEgoTrf[0], mEgoTrf[1], mEgoTrf[2]);
    for (auto && b : mLocallLaneBoxes)
    {
        auto && baundaries = vizBoundary[b.roadID][b.shapeID][b.sectID];
        for (auto && bps : baundaries)
        {
            glColor3f(0.0f, 0.0f, 1.0f);    
            glBegin(GL_LINE_STRIP);
            for (size_t i = 0; i < bps.second.size(); ++i)
            {
                if ((Vector3d(bps.second[i].x(), bps.second[i].y(), bps.second[i].z()) - egoTrl).squaredNorm() < radius2)
                    glVertex3f(bps.second[i].x(), bps.second[i].y(), bps.second[i].z() + 0.2f);
            }
            glEnd();
        }
    }

    // visualize the fitted Polys:
    for (auto && poly : mPolys)
    {
        glPushMatrix();
        glPointSize(2);
        //glColor3f((float)rand()/__INT_MAX__, (float)rand()/__INT_MAX__, (float)rand()/__INT_MAX__);
        glColor3f(1.0f, 0.0f, 0.0f);
        glTranslated(mEgoTrf[0], mEgoTrf[1], mEgoTrf[2] + 0.3);
        glRotatef(mEgoTrf[3]*180/M_PI, 0, 0, 1);
        glBegin(GL_LINE_STRIP);
        for (double t = 0.0; t <= poly.length; t += 0.1) // step in meters if used non-unitless solution in poly fitting
        //for (double t = 0.0; t <= 1; t += 0.01) // unitless (percents) step if used unitless solution in poly fitting
        {
            auto x = poly.xF[0]*t*t*t + poly.xF[1]*t*t + poly.xF[2]*t + poly.xF[3];
            auto y = poly.yF[0]*t*t*t + poly.yF[1]*t*t + poly.yF[2]*t + poly.yF[3];
            auto z = poly.zF[0]*t*t*t + poly.zF[1]*t*t + poly.zF[2]*t + poly.zF[3];
            if ((Vector3d(x, y, z)).squaredNorm() < radius2) // since we are no in Ego CS, dont need to subtract Ego trl
                glVertex3f(x, y, z + 0.3);
        }
        glEnd();
        glPopMatrix();
    }
}

void CanvasXODR::fitPoly(const vector<Vector4d> & points, PolyFactors & pf, const Matrix4d * trf)
{
    // Processing:
    // Approximation of the points by a polynomial function.
    // The fitting will be done for X,Y and Z components of the points separately (similarly to poly3 from ODR)
    // We will use notation c0*t*t*t + c1*t*t + c2*t + c3 = X(Y or Z), where t is a unit-less parameter [0,1]
    // In terms of LSA the above equation has the traditional form:
    // Ax = b, where A is the matrix with N lines of form [t3 t2 t 1], where N is number of measurements
    // x is the unknown vector [c0,c1,c2,c3]T, T stands for transposed
    // and b is w
    // The final solutiong is x = (ATA)-1ATb  where T is transposed and -1 is inverted
    const size_t S = points.size();
    MatrixXd A(S, 4); // ex. A(S,2) for a line, A(S,3) for parabol and A(S,4) for cubic approximation
    MatrixXd b(S, 3); // one column for X, one column for Y, and one for Z axis

    double t = 0; // parameter changing in [0,1]
    double closingSegmentLength = (points[S-2].block(0,0,3,1) - points[S-1].block(0,0,3,1)).norm(); // generally not equal mXodrResolution!
    double L = (S-2)*mXodrResolution + closingSegmentLength; // |.....|.....|.....|..|  // ex. 5 points: closingSegment is usually shorter!
    double s = 0;
    for (size_t i = 0; i < S; ++i)
    {
        s = (i == S -1) ? s + closingSegmentLength : i*mXodrResolution;
        //t = s/L; // unitless solution
        //t = (double)i/(S-1); // a simpler unitless solution (no need s and L)
        t = s; // meters, we apply this method, cause its more deterministic than unitless when the Poly factors are used by client.
        A(i, 0) = t*t*t;
        A(i, 1) = t*t;
        A(i, 2) = t;
        A(i, 3) = 1.0;
        Vector4d p = points[i];
        if (trf) p = *trf*Vector4d(p[0], p[1], p[2], 1);
        b(i, 0) = p[0];
        b(i, 1) = p[1];
        b(i, 2) = p[2];
    }
    Matrix<double,4,3> x = (A.transpose()*A).inverse()*A.transpose()*b;
    pf.xF = x.block(0,0,4,1);
    pf.yF = x.block(0,1,4,1);
    pf.zF = x.block(0,2,4,1);
    pf.length = L;
}

void CanvasXODR::draw()
{
    drawSelectable();
    drawUnSelectable();
}

void CanvasXODR::drawWithNames()
{
    glPushName(1);
    drawSelectable();
    glPopName();

    glPushName(2);
    drawUnSelectable();
    glPopName();
}

void CanvasXODR::computePolys(Vector3d p, Vector2f dir)
{
    mLocallLaneBoxes.clear();
    for (auto && b : mLaneBoxes)
    {
        if (b.isPointClose(p, mRadius))
            mLocallLaneBoxes.push_back(b);
    }
    // set the Ego position:
    mEgoTrf.block(0,0,3,1) = p;
    // if ori is not provided get it from XODR:
    if (dir.isZero())
    {
        // find the orientatino from the position
        double minDist2 = __DBL_MAX__;
        for (auto && b : mLocallLaneBoxes)
        {
            if (b.isPointInside(p))
            {
                auto && sections = vizCenter[b.roadID][b.shapeID][b.sectID];
                for (auto && cps : sections)
                {
                    for (size_t i = 0; i < cps.second.size(); ++i)
                    {
                        auto dist2 = (Vector3d(cps.second[i].x(), cps.second[i].y(), cps.second[i].z()) - p).squaredNorm();
                        if (dist2 < minDist2)
                        {
                            minDist2 = dist2;
                            mEgoTrf(3,0) = cps.second[i](3,0); // set the direction
                        }
                    }
                }
            }
        }
    }
    // otherwise set from provided dir:
    else mEgoTrf(3,0) = atan2(dir.y(), dir.x());

    // fit polys:
    mPolys.clear();
    // first prepare the matrix to transform the boundary points into Ego CS:
    Matrix4d egoTrf; egoTrf.setIdentity();
    egoTrf.block(0,0,3,3) = AngleAxisd(mEgoTrf[3], Vector3d::UnitZ()).toRotationMatrix();
    egoTrf.block(0,3,3,1) = mEgoTrf.block(0,0,3,1);
    Matrix4d egoTrfInv = egoTrf.inverse();

    for (auto && b : mLocallLaneBoxes)
    {
        auto && baundaries = vizBoundary[b.roadID][b.shapeID][b.sectID];
        for (auto && bps : baundaries)
        {
            PolyFactors pf;
            // having 3 and fewer points makes the solution numerically unstable:
            if (bps.second.size() > 3)
            {
                // tarnsform the points into Ego CS:
                fitPoly(bps.second, pf, &egoTrfInv);
                pf.roadID = b.roadID;
                pf.shapeID = b.shapeID;
                pf.sectionID = b.sectID;
                pf.laneID = bps.first;
                mPolys.push_back(pf);
            }
        }
    }
}