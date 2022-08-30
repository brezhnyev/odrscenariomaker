#pragma once

#include "Selectable.h"

#include <qrect.h>
#include <qimage.h>

#include <eigen3/Eigen/Eigen>

#include <XodrBuilder/XodrBuilder.h>

#include <string>
#include <vector>
#include <map>
#include <limits>

class CanvasXODR : public Selectable
{
public:
    typedef struct
    {
        odr_1_5::t_road_planView_geometry * psubroad{nullptr};
        odr_1_5::t_road_lanes_laneSection * psection{nullptr};
        int gindex{0};
        double gs{0}; // geometry s
        int sindex{0};
    } SValue;

    CanvasXODR(std::string xodrfile);
    ~CanvasXODR();
    void init();
    void draw() override;
    void drawWithNames() override;
    std::string getType() const override { return "CanvasXODR"; }

    static int getLaneID(Eigen::Vector3d p);

private:
    std::multimap<double, SValue> collectSValues(const odr_1_5::t_road &);

private:
    // key1: roadID, key2: sectionID, key3: laneID
    XodrBuilder m_xodrBuilder;
    uint listRoad, listBounadries, listCenterlines;
    float mXodrResolution {0.1};
};