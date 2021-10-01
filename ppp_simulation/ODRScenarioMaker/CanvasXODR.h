#pragma once

#include "Selectable.h"

#include <qrect.h>
#include <qimage.h>

#include <eigen3/Eigen/Eigen>

#include <string>
#include <vector>
#include <map>
#include <limits>

template <typename T>
struct LaneElementBBox
{
    T minX, maxX, minY, maxY, minZ, maxZ;
    int laneID;
    bool isPointInside(Eigen::Matrix<T,3,1> p)
    {
        return p[0] >= minX && p[0] <= maxX && p[1] >= minY && p[1] <= maxY && p[2] >= minZ && p[2] <= maxZ;
    }
    void addPoint(Eigen::Matrix<T,3,1> p)
    {
        if (p[0] < minX) minX = p[0];
        if (p[0] > maxX) maxX = p[0];
        if (p[1] < minY) minY = p[1];
        if (p[1] > maxY) maxY = p[1];
        if (p[2] < minZ) minZ = p[2];
        if (p[2] > maxZ) maxZ = p[2];
    }
    LaneElementBBox() : 
    minX( std::numeric_limits<T>::max()),
    maxX(-std::numeric_limits<T>::max()),
    minY( std::numeric_limits<T>::max()),
    maxY(-std::numeric_limits<T>::max()),
    minZ( std::numeric_limits<T>::max()),
    maxZ(-std::numeric_limits<T>::max())
    {}
};

class CanvasXODR : public Selectable
{
public:
    CanvasXODR(std::string xodrfile);
    ~CanvasXODR();
    void init();
    void draw() override;
    void drawWithNames() override;
    std::string getType() const override { return "CanvasXODR"; }

    static int getLaneID(Eigen::Vector3d p);

private:
    // key1: roadID, key2: sectionID, key3: laneID
    std::map<int, std::map<int, std::map<int, std::vector<Eigen::Vector3d>>>> vizBoundary;
    std::map<int, std::map<int, std::map<int, std::vector<Eigen::Vector3d>>>> vizCenter;
    uint listRoad, listBounadries, listCenterlines;
    // ATTENTION STATIC!!!
    static std::vector<LaneElementBBox<double>> s_lboxes;
};