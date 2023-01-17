#pragma once

#include <qrect.h>
#include <qimage.h>

#include <GL/gl.h>

#include <eigen3/Eigen/Eigen>
#include <XodrBuilder/XodrBuilder.h>

#include <string>
#include <vector>
#include <map>
#include <set>
#include <limits>

template <typename T>
struct LaneElementBBox
{
    typedef typename T::Scalar basetype;
    basetype minX, maxX, minY, maxY, minZ, maxZ;
    int sectID;
    int geomID;
    int roadID;
    bool isPointInside(T p)
    {
        return p[0] >= minX && p[0] <= maxX && p[1] >= minY && p[1] <= maxY && p[2] >= minZ && p[2] <= maxZ;
    }
    T center()
    {
        return T(0.5*(minX + maxX), 0.5*(minY + maxY), 0.5*(minZ + maxZ));
    }
    bool isPointClose(T p, float distance)
    {
        auto C = center();
        return isPointInside(p) || (C - p).norm() < (C - T(minX,minY,minZ)).norm() + distance && p[2] < maxZ + 2 && p[2] > minZ - 2;
    }
    void draw()
    {
        glBegin(GL_QUADS);
        glVertex3f(minX, minY, maxZ);
        glVertex3f(maxX, minY, maxZ);
        glVertex3f(maxX, maxY, maxZ);
        glVertex3f(minX, maxY, maxZ);
        glEnd();
    }
    void addPoint(T p)
    {
        if (p[0] < minX) minX = p[0];
        if (p[0] > maxX) maxX = p[0];
        if (p[1] < minY) minY = p[1];
        if (p[1] > maxY) maxY = p[1];
        if (p[2] < minZ) minZ = p[2];
        if (p[2] > maxZ) maxZ = p[2];
    }
    LaneElementBBox() : 
    minX( std::numeric_limits<basetype>::max()),
    maxX(-std::numeric_limits<basetype>::max()),
    minY( std::numeric_limits<basetype>::max()),
    maxY(-std::numeric_limits<basetype>::max()),
    minZ( std::numeric_limits<basetype>::max()),
    maxZ(-std::numeric_limits<basetype>::max())
    {}
};

class CanvasXODR
{
public:
    typedef struct 
    {
        int roadID{0};
        int geomID{0};
        int sectID{0};
        int laneID{0};
        Eigen::Vector4d xF{0,0,0,0};
        Eigen::Vector4d yF{0,0,0,0};
        Eigen::Vector4d zF{0,0,0,0};
        float length{0.0f};
        double xy [4]; ///< c0,c1,c2,c3 of y = f(x) polynomial
        float xmin{0.0f};     ///< minimal x
        float xmax{100.0f};   ///< maximal x
    } PolyFactors;

    typedef struct
    {
        odr_1_5::t_road_planView_geometry * psubroad{nullptr};
        odr_1_5::t_road_lanes_laneSection * psection{nullptr};
        int gindex{0};
        double gs{0}; // geometry s
        int sindex{0};
    } SValue;

    CanvasXODR(const std::string & xodrfile, float radius, float xodrResolution, std::string lanesMappingPath, float frustum_angle, float frustum_offset);
    ~CanvasXODR();
    void draw();
    void drawSelectable();
    void drawUnSelectable();
    void drawWithNames();
    void init(); // init OpenGL related stuff
    void computePolys(Eigen::Vector3d p, Eigen::Vector2f dir = Eigen::Vector2f(0.0f, 0.0f));
    std::vector<PolyFactors> getPolys() { return mPolys; }
    double getSceneRadius();
    void toggleDirection()
    {
        mDir *= -1;
        computePolys(mEgoTrf.block(0,0,3,1));
    }

private:
    void parseXodr(const std::string & xodrfile);
    bool fitParamPoly(const XodrBuilder::Lane & points, PolyFactors & pf, const Eigen::Matrix4d trf = Eigen::Matrix4d().setIdentity());
    bool fitPoly(const XodrBuilder::Lane & points, PolyFactors & pf, const Eigen::Matrix4d trf = Eigen::Matrix4d().setIdentity());

private:
    const float mRadius{20};
    const float mXodrResolution{1};
    XodrBuilder m_xodrBuilder;
    uint listRoad, listBounadries, listCenterlines;
    std::vector<LaneElementBBox<Eigen::Vector3d>> mLaneBoxes;
    std::vector<LaneElementBBox<Eigen::Vector3d>> mLocallLaneBoxes;
    Eigen::Vector4d mEgoTrf {0,0,0,0};
    std::vector<PolyFactors> mPolys;
    LaneElementBBox<Eigen::Vector3d> mSceneBB;
    int mDir{1}; // sets the direction of the poly computation (forward-backward)
    std::map<std::string, int> m_lanesMapping;
    std::set<int> m_validRoads;
    float m_frustumAngle{30};
    float m_frustumOffset{5};
};