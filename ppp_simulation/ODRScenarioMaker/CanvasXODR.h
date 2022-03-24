#pragma once

#include "Selectable.h"

#include <qrect.h>
#include <qimage.h>

#include <eigen3/Eigen/Eigen>

#include <string>
#include <vector>
#include <map>
#include <limits>

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
    float mXodrResolution {0.1};
};