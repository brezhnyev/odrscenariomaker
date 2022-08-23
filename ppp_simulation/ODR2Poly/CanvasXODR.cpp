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

//#define AVL_STACK


CanvasXODR::CanvasXODR(const string & xodrfile, float radius, float xodrResolution) : mRadius(radius), mXodrResolution(xodrResolution)
{
    parseXodr(xodrfile);
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

void CanvasXODR::parseXodr(const string & xodrfile)
{
    OpenDRIVEFile ODR;
    loadFile(xodrfile, ODR);

    for (auto && odr_road : ODR.OpenDRIVE1_5->sub_road)
    {
        double S = 0; // total length of road

        // skip of no lanes available
        if (!odr_road.sub_lanes)
            continue;

        // skip if lanes empty
        if (odr_road.sub_lanes->sub_laneSection.empty())
            continue;

        int gindex = 0;
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
                auto sections = odr_road.sub_lanes->sub_laneSection; // sections iterator
                uint32_t sindex = 0;
                while (sindex < sections.size() && *(sections[sindex]._s) < S) ++sindex;
                if (sindex) --sindex;
                auto && odr_lane = sections[sindex];

                // set the lanes:
                double offset = 0.0;
                if (odr_road.sub_lanes && !odr_road.sub_lanes->sub_laneOffset.empty())
                    offset = polyInter(S, odr_road.sub_lanes->sub_laneOffset, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_lanes->sub_laneOffset[0])>(it)->_s; });
                
                if (odr_lane.sub_center)
                    for (auto && odr_sublane : odr_lane.sub_center->sub_lane)
                    {
                        // ONLY 1 center line, actually loop not needed
                        Eigen::Vector4d Ptrf = M * (P + normal*offset);
                        vizBoundary[*odr_road._id][gindex][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
                    }
                auto buildLane = [&](auto & odr_sublane, int dir, double & twidth)
                {
                    // Boundary:
                    double width = polyInter(S - *odr_lane._s, odr_sublane.sub_width, [](void * it) ->double { return *static_cast<decltype(&odr_sublane.sub_width[0])>(it)->_sOffset; });
                    twidth += dir*width;
                    Eigen::Vector4d Ptrf = M * (P + normal*twidth);
                    vizBoundary[*odr_road._id][gindex][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
                    // Center:
                    Ptrf = M * (P + normal*(twidth - dir*width/2));
                    vizCenter[*odr_road._id][gindex][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
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
            ++gindex;
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
    for (auto && r : vizBoundary) // road
    {
        for (auto && g : r.second)
        {
            for (auto && s : g.second) // lane section
            {
                LaneElementBBox<Vector3d> bbox; // bbox incomprizes multiple lanes
                bbox.roadID = r.first;
                bbox.geomID= g.first;
                bbox.sectID = s.first;
                for (auto && l : s.second) // lanes
                {
                    for (size_t i = 0; i < l.second.size(); ++i) // lanes points
                    {
                        bbox.addPoint(l.second[i].block(0,0,3,1));
                    }
                }
                bbox.minZ -= 0.1;
                bbox.maxZ += 0.1;
                mLaneBoxes.push_back(bbox);
            }
        }
    }
    // get the BBox of the whole scene:
    for (auto && lanebb : mLaneBoxes)
    {
        mSceneBB.addPoint(Vector3d(lanebb.minX,lanebb.minY,lanebb.minZ));
        mSceneBB.addPoint(Vector3d(lanebb.minX,lanebb.minY,lanebb.maxZ));
        mSceneBB.addPoint(Vector3d(lanebb.minX,lanebb.maxY,lanebb.minZ));
        mSceneBB.addPoint(Vector3d(lanebb.minX,lanebb.maxY,lanebb.maxZ));
        mSceneBB.addPoint(Vector3d(lanebb.maxX,lanebb.minY,lanebb.minZ));
        mSceneBB.addPoint(Vector3d(lanebb.maxX,lanebb.minY,lanebb.maxZ));
        mSceneBB.addPoint(Vector3d(lanebb.maxX,lanebb.maxY,lanebb.minZ));
        mSceneBB.addPoint(Vector3d(lanebb.maxX,lanebb.maxY,lanebb.maxZ));
    }
    // roads:
    listRoad = glGenLists(1);
    glNewList(listRoad, GL_COMPILE);
    for (auto && r : vizBoundary)
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

    // draw boundaries:
    listBounadries = glGenLists(1);
    glNewList(listBounadries, GL_COMPILE);
    glPushMatrix();
    glTranslatef(0,0,0.1f);

    for (auto && r : vizBoundary)
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
}

double CanvasXODR::getSceneRadius()
{
    return 0.5*(Eigen::Vector3d(mSceneBB.minX, mSceneBB.minY, mSceneBB.minZ) - Eigen::Vector3d(mSceneBB.maxX, mSceneBB.maxY, mSceneBB.maxZ)).norm();
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
    glColor3f(1.0f, 1.0f, 0.0f);
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
    const float radius2 = mRadius*mRadius;
    Vector3d egoTrl = Vector3d(mEgoTrf[0], mEgoTrf[1], mEgoTrf[2]);

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
#ifndef AVL_STACK
        for (double t = 0.0; t <= poly.length; t += 0.1) // step in meters if used non-unitless solution in poly fitting
        //for (double t = 0.0; t <= 1; t += 0.01) // unitless (percents) step if used unitless solution in poly fitting
        {
            auto x = poly.xF[0]*t*t*t + poly.xF[1]*t*t + poly.xF[2]*t + poly.xF[3];
            auto y = poly.yF[0]*t*t*t + poly.yF[1]*t*t + poly.yF[2]*t + poly.yF[3];
            auto z = poly.zF[0]*t*t*t + poly.zF[1]*t*t + poly.zF[2]*t + poly.zF[3];
            glVertex3f(x, y, z + 0.1);
        }
#else // AVL AD stack Polyline representation:
        float xlen = poly.xmax - poly.xmin;
        for (float t = poly.xmin; t < poly.xmax; t += 0.1)
        {
            auto y = poly.xy[0]*t*t*t + poly.xy[1]*t*t + poly.xy[2]*t + poly.xy[3];
            glVertex3f(t,y,0.1);
        }
#endif
        glEnd();
        glPopMatrix();
    }
}

void CanvasXODR::fitPoly(const vector<Vector4d> & points, PolyFactors & pf, const Matrix4d trf)
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
    // We make a copy of the points, since they should be filtered by distance:
    const float radius2 = mRadius*mRadius;
    vector<Vector4d> tfpoints;
    tfpoints.reserve(points.size());
    for (auto && p : points)
    {
        Vector4d tfp = trf*(Vector4d(p.x(), p.y(), p.z(), 1.0)); // beware p[3] is storing heading info (not the 1)!
        if (mDir*tfp[0] >= 0 && tfp.block(0,0,3,1).squaredNorm() < radius2)
            tfpoints.push_back(tfp);
    }

    const size_t S = tfpoints.size();
    if (S < 4)
        return;

    MatrixXd A(S, 4); // ex. A(S,2) for a line, A(S,3) for parabol and A(S,4) for cubic approximation
    MatrixXd b(S, 3); // one column for X, one column for Y, and one for Z axis

    double t = 0; // parameter changing in [0,1]
    double closingSegmentLength = (tfpoints[S-2].block(0,0,3,1) - tfpoints[S-1].block(0,0,3,1)).norm(); // generally not equal mXodrResolution!
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
        Vector4d p = tfpoints[i];
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


void CanvasXODR::fitPolyAVL(const vector<Vector4d> & points, PolyFactors & pf, const Matrix4d trf)
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
    // We make a copy of the points, since they should be filtered by distance:
    const float radius2 = mRadius*mRadius;
    vector<Vector4d> tfpoints;
    tfpoints.reserve(points.size());
    for (auto && p : points)
    {
        Vector4d tfp = trf*(Vector4d(p.x(), p.y(), p.z(), 1.0)); // beware p[3] is storing heading info (not the 1)!
        if (mDir*tfp[0] >= 0 && tfp.block(0,0,3,1).squaredNorm() < radius2)
            tfpoints.push_back(tfp);
    }

    const size_t S = tfpoints.size();
    if (S < 4)
        return;

    MatrixXd A(S, 4); // ex. A(S,2) for a line, A(S,3) for parabol and A(S,4) for cubic approximation
    MatrixXd b(S, 1); // one column for Y

    for (size_t i = 0; i < S; ++i)
    {
        double t = tfpoints[i].x();
        A(i, 0) = t*t*t;
        A(i, 1) = t*t;
        A(i, 2) = t;
        A(i, 3) = 1.0;
        b(i, 0) = tfpoints[i][1]; // << using for dependency y = f(x)
    }
    Matrix<double,4,1> x = (A.transpose()*A).inverse()*A.transpose()*b;
    for (size_t i = 0; i < 4; ++i) { pf.xy[i] = x(i,0); }
    pf.xmin = __FLT_MAX__;
    pf.xmax = __FLT_MIN__;
    for (auto && p : tfpoints)
    {
        if (p.x() < pf.xmin) pf.xmin = p.x();
        if (p.x() > pf.xmax) pf.xmax = p.x();
    }
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
                auto && sections = vizCenter[b.roadID][b.geomID][b.sectID];
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

#ifndef AVL_STACK
    for (auto && b : mLocallLaneBoxes)
    {
        auto && baundaries = vizBoundary[b.roadID][b.geomID][b.sectID];
        for (auto && bps : baundaries)
        {
            PolyFactors pf;
            // having 3 and fewer points makes the solution numerically unstable:
            if (bps.second.size() > 3)
            {
                // tarnsform the points into Ego CS:
                fitPoly(bps.second, pf, egoTrfInv);
                pf.roadID = b.roadID;
                pf.geomID = b.geomID;
                pf.sectID = b.sectID;
                pf.laneID = bps.first;
                mPolys.push_back(pf);
            }
        }
    }
#else
    // for AVL stack the SINGLE road will have unique lanes each for the length of the road:
    map<int, vector<Vector4d>> lanes;
    for (auto && b : mLocallLaneBoxes)
    {
        auto && boundaries = vizBoundary[b.roadID][b.geomID][b.sectID];
        for (auto & bsp : boundaries)
            lanes[bsp.first].insert(lanes[bsp.first].end(), bsp.second.begin(), bsp.second.end());
    }
    for (auto && l : lanes) // lane
    {
        PolyFactors pf;
        // having 3 and fewer points makes the solution numerically unstable:
        if (l.second.size() > 3)
        {
            // tarnsform the points into Ego CS:
            fitPolyAVL(l.second, pf, egoTrfInv);
            pf.roadID = __INT_MAX__;
            pf.geomID = __INT_MAX__;
            pf.sectID = __INT_MAX__;
            pf.laneID = l.first;
            mPolys.push_back(pf);
        }
    }
    // cout << "polylines" << endl << endl;
    // for (auto && poly : mPolys)
    // {
    //     cout << "\t" << "roadID: " << poly.roadID << endl;
    //     cout << "\t" << "geomID: " << poly.geomID << endl;
    //     cout << "\t" << "sectID: " << poly.sectID << endl;
    //     cout << "\t" << "laneID: " << poly.laneID << endl;
    //     cout << "\t" << "xF: " << poly.xF.transpose() << endl;
    //     cout << "\t" << "xY: " << poly.yF.transpose() << endl;
    //     cout << "\t" << "xZ: " << poly.zF.transpose() << endl;
    //     cout << "\t" << "length: " << poly.length << endl;
    //     cout << "\t" << "xy: " << poly.xy[0] << " " << poly.xy[1] << " " << poly.xy[2] << " " << poly.xy[3] << endl;
    //     cout << "\t" << "minx: " << poly.xmin << endl;
    //     cout << "\t" << "maxx: " << poly.xmax << endl;
    //     cout << endl;
    // }
#endif
}