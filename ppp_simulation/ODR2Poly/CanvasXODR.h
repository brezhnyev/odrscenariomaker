#pragma once

#include <qrect.h>
#include <qimage.h>

#include <GL/gl.h>

#include <eigen3/Eigen/Eigen>

#include <string>
#include <vector>
#include <map>
#include <limits>

template <typename T>
struct LaneElementBBox
{
    typedef typename T::Scalar basetype;
    basetype minX, maxX, minY, maxY, minZ, maxZ;
    int sectID;
    int shapeID;
    int roadID;
    bool isPointInside(T p)
    {
        return p[0] >= minX && p[0] <= maxX && p[1] >= minY && p[1] <= maxY && p[2] >= minZ && p[2] <= maxZ;
    }
    bool isPointClose(T p, float distance)
    {
        return
            isPointInside(p) ||
            (T(minX,minY,minZ)-p).norm() < distance ||
            (T(minX,minY,maxZ)-p).norm() < distance ||
            (T(minX,maxY,minZ)-p).norm() < distance ||
            (T(minX,maxY,maxZ)-p).norm() < distance ||
            (T(maxX,minY,minZ)-p).norm() < distance ||
            (T(maxX,minY,maxZ)-p).norm() < distance ||
            (T(maxX,maxY,minZ)-p).norm() < distance ||
            (T(maxX,maxY,maxZ)-p).norm() < distance;
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
        int shapeID{0};
        int sectionID{0};
        int laneID{0};
        Eigen::Vector4d xF{0,0,0,0};
        Eigen::Vector4d yF{0,0,0,0};
        Eigen::Vector4d zF{0,0,0,0};
        float length{0.0f};
    } PolyFactors;

    CanvasXODR(const std::string & xodrfile, float radius, float xodrResolution);
    ~CanvasXODR();
    void draw();
    void drawSelectable();
    void drawUnSelectable();
    void drawWithNames();
    void init(); // init OpenGL related stuff
    void computePolys(Eigen::Vector3d p, Eigen::Vector2f dir = Eigen::Vector2f(0.0f, 0.0f));
    std::vector<PolyFactors> getPolys() { return mPolys; }

    typedef std::map<int, std::map<int, std::map<int, std::map<int, std::vector<Eigen::Vector4d>>>>> LanesContainer;

private:
    void parseXodr(const std::string & xodrfile);
    inline void fitPoly(const std::vector<Eigen::Vector4d> & points, PolyFactors & pf, const Eigen::Matrix4d * trf = nullptr);

private:
    const float mRadius{20};
    const float mXodrResolution{1};
    // key1: roadID, key2: roadShapeID, key3: laneSection, key4: laneID. Value: {x,y,z,heading}
    LanesContainer vizBoundary;
    LanesContainer vizCenter;
    uint listRoad, listBounadries, listCenterlines;
    std::vector<LaneElementBBox<Eigen::Vector3d>> mLaneBoxes;
    std::vector<LaneElementBBox<Eigen::Vector3d>> mLocallLaneBoxes;
    Eigen::Vector4d mEgoTrf {0,0,0,0};
    std::vector<PolyFactors> mPolys;
};