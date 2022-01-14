#pragma once

#include <qrect.h>
#include <qimage.h>

#include <GL/gl.h>

#include <eigen3/Eigen/Eigen>
#include <tinyxml2.h>

#include <string>
#include <vector>
#include <map>
#include <limits>
#include <iostream>

template <typename T>
struct BBox
{
    typedef typename T::Scalar basetype;
    basetype minX, maxX, minY, maxY, minZ, maxZ;

    BBox() :
        minX( std::numeric_limits<basetype>::max()),
        maxX(-std::numeric_limits<basetype>::max()),
        minY( std::numeric_limits<basetype>::max()),
        maxY(-std::numeric_limits<basetype>::max()),
        minZ( std::numeric_limits<basetype>::max()),
        maxZ(-std::numeric_limits<basetype>::max())
        {}

    BBox(basetype minX_, basetype maxX_, basetype minY_, basetype maxY_, basetype minZ_, basetype maxZ_) :
        minX(minX_),
        maxX(maxX_),
        minY(minY_),
        maxY(maxY_),
        minZ(minZ_),
        maxZ(maxZ_) {}

    bool isPointInside(T p) const
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
    bool isInsideOther(const BBox & other)
    {
        // if all of the BBox points are inside the other
        if (
            other.isPointInside(T(minX, minY, minZ)) &&
            other.isPointInside(T(minX, minY, maxZ)) &&
            other.isPointInside(T(minX, maxY, minZ)) &&
            other.isPointInside(T(minX, maxY, maxZ)) &&
            other.isPointInside(T(maxX, minY, minZ)) &&
            other.isPointInside(T(maxX, minY, maxZ)) &&
            other.isPointInside(T(maxX, maxY, minZ)) &&
            other.isPointInside(T(maxX, maxY, maxZ)))
                return true;

        return false;
    }
    bool isCrossingOther(const BBox & other)
    {
        // if any of the BBox points are inside the other
        if (
            other.isPointInside(T(minX, minY, minZ)) ||
            other.isPointInside(T(minX, minY, maxZ)) ||
            other.isPointInside(T(minX, maxY, minZ)) ||
            other.isPointInside(T(minX, maxY, maxZ)) ||
            other.isPointInside(T(maxX, minY, minZ)) ||
            other.isPointInside(T(maxX, minY, maxZ)) ||
            other.isPointInside(T(maxX, maxY, minZ)) ||
            other.isPointInside(T(maxX, maxY, maxZ)))
                return true;

        return false;
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
};

typedef struct
{
    int roadID{0};
    int geoID{0};
    int sectID{0};
    int laneID{0};
} xodrid_t;

template <typename T>
struct LaneElementBBox : BBox<T>
{
    xodrid_t xodrid;
};

inline std::ostream & operator << (std::ostream & os, const xodrid_t & glid)
{
    os << "roadID=" << glid.roadID << "  geoID=" << glid.geoID << "  sectID=" << glid.sectID << "  laneID=" << glid.laneID << std::endl;
    return os;
}

class CanvasXODR
{
public:

    CanvasXODR(const std::string & xodrfile, float xodrResolution);
    ~CanvasXODR();
    void draw();
    void drawSelectable(uint32_t);
    void drawUnSelectable();
    void drawWithNames();
    xodrid_t highlightSelected(uint32_t glid, Eigen::Vector3d);
    void highlightSelected(uint32_t xodrid);
    void highlightRibbonSelection(const Eigen::Vector3d & , const Eigen::Vector3d &);
    void init(); // init OpenGL related stuff
    double getSceneRadius();
    typedef std::map<int, std::map<int, std::map<int, std::map<int, std::vector<Eigen::Vector4d>>>>> LanesContainer;

private:
    bool parseXodr(const std::string & xodrfile);
    void removeLinks(tinyxml2::XMLNode *, const std::string &);

private:
    const float mXodrResolution{1};
    // key1: roadID, key2: roadgeoID, key3: laneSection, key4: laneID. Value: {x,y,z,heading}
    LanesContainer vizBoundary;
    LanesContainer vizCenter;
    uint listBounadries, listCenterlines;
    Eigen::Vector4d mEgoTrf {0,0,0,0};
    LaneElementBBox<Eigen::Vector3d> mSceneBB;
    uint mSelectedLane{0};
    std::map<uint32_t, LaneElementBBox<Eigen::Vector3d>> mListID2LaneMap;
    std::vector<uint32_t> mSelectedBoxes;
    std::string mXodrFile;
};