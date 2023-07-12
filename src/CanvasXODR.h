#pragma once

#include "Selectable.h"

#include <qrect.h>
#include <qimage.h>

#include <eigen3/Eigen/Eigen>

#include "XodrBuilder.h"

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
    void draw() const override;
    void drawWithNames() const override;
    std::string getType() const override { return "CanvasXODR"; }
    std::pair<Eigen::Vector3d, Eigen::Vector3d> getSceneBbox() { return m_sceneBbox; }

private:
    std::multimap<double, SValue> collectSValues(const odr_1_5::t_road &);
    void addToSceneBbox(const Eigen::Vector4d & v)
    {
        if (v[0] < m_sceneBbox.first[0])  m_sceneBbox.first[0]  = v[0];
        if (v[1] < m_sceneBbox.first[1])  m_sceneBbox.first[1]  = v[1];
        if (v[2] < m_sceneBbox.first[2])  m_sceneBbox.first[2]  = v[2];
        if (v[0] > m_sceneBbox.second[0]) m_sceneBbox.second[0] = v[0];
        if (v[1] > m_sceneBbox.second[1]) m_sceneBbox.second[1] = v[1];
        if (v[2] > m_sceneBbox.second[2]) m_sceneBbox.second[2] = v[2];
    }

private:
    // key1: roadID, key2: sectionID, key3: laneID
    XodrBuilder m_xodrBuilder;
    uint listRoad, listBounadries, listCenterlines;
    float mXodrResolution {0.1};
    std::pair<Eigen::Vector3d, Eigen::Vector3d> m_sceneBbox;
};