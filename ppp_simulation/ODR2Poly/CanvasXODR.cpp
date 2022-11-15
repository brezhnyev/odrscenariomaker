#include "CanvasXODR.h"

#include <eigen3/Eigen/Eigen>

#include <set>
#include <iostream>
#include <deque>
#include <omp.h>

using namespace std;
using namespace Eigen;

//#define AVL_STACK


CanvasXODR::CanvasXODR(const string & xodrfile, float radius, float xodrResolution) : mRadius(radius), mXodrResolution(xodrResolution), m_xodrBuilder(xodrfile, mXodrResolution)
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
    // build the BBoxes for the lane elements
    for (auto && r : m_xodrBuilder.getBoundaries()) // road
    {
        for (auto && g : r.second) // road geomertry element (arc, line, ..)
        {
            for (auto && s : g.second) // lane section
            {
                LaneElementBBox<Vector3d> bbox; // bbox incomprizes multiple lanes
                bbox.roadID = r.first;
                bbox.geomID = g.first;
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

    // draw boundaries:
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

bool CanvasXODR::fitParamPoly(const vector<Vector4d> & points, PolyFactors & pf, const Matrix4d trf)
{
    // Processing:
    // Approximation of the points by a polynomial function.
    // The fitting will be done for X,Y and Z components of the points separately (similarly to poly3 from ODR)
    // We will use notation c0*t*t*t + c1*t*t + c2*t + c3 = X(Y or Z), where t is a unit-less parameter [0,1]
    // In terms of LSA the above equation has the traditional form:
    // Ax = b, where A is the matrix with N lines of form [t3 t2 t 1], where N is number of measurements
    // x is the unknown vector [c0,c1,c2,c3]T, T stands for transposed
    // and b is a 3d point (x,y,z)
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
        return false;

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

    return true;
}


bool CanvasXODR::fitPoly(const vector<Vector4d> & points, PolyFactors & pf, const Matrix4d trf)
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
        return false;

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

    return true;
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
                auto && sections =  const_cast<LanesContainer&>(m_xodrBuilder.getCenters()) [b.roadID][b.geomID][b.sectID];
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
        auto && boundaries = const_cast<LanesContainer&>(m_xodrBuilder.getBoundaries()) [b.roadID][b.geomID][b.sectID];
        for (auto && bps : boundaries)
        {
            PolyFactors pf;
            if (fitParamPoly(bps.second, pf, egoTrfInv))
            {
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
        auto && boundaries = const_cast<LanesContainer&>(m_xodrBuilder.getBoundaries()) [b.roadID][b.geomID][b.sectID];
        for (auto & bsp : boundaries)
            lanes[bsp.first].insert(lanes[bsp.first].end(), bsp.second.begin(), bsp.second.end());
    }
    for (auto && l : lanes) // lane
    {
        PolyFactors pf;
        if (fitPoly(l.second, pf, egoTrfInv))
        {
            pf.roadID = __INT_MAX__;
            pf.geomID = __INT_MAX__;
            pf.sectID = __INT_MAX__;
            pf.laneID = l.first;
            mPolys.push_back(pf);
        }
    }
    cout << "polylines" << endl << endl;
    for (auto && poly : mPolys)
    {
        cout << "\t" << "roadID: " << poly.roadID << endl;
        cout << "\t" << "geomID: " << poly.geomID << endl;
        cout << "\t" << "sectID: " << poly.sectID << endl;
        cout << "\t" << "laneID: " << poly.laneID << endl;
        cout << "\t" << "xF: " << poly.xF.transpose() << endl;
        cout << "\t" << "xY: " << poly.yF.transpose() << endl;
        cout << "\t" << "xZ: " << poly.zF.transpose() << endl;
        cout << "\t" << "length: " << poly.length << endl;
        cout << "\t" << "xy: " << poly.xy[0] << " " << poly.xy[1] << " " << poly.xy[2] << " " << poly.xy[3] << endl;
        cout << "\t" << "minx: " << poly.xmin << endl;
        cout << "\t" << "maxx: " << poly.xmax << endl;
        cout << endl;
    }
#endif
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
