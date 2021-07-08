#pragma once

#include <eigen3/Eigen/Eigen>
#include <osi3/osi_sensorview.pb.h>

#include <vector>
#include <string>
#include <fstream>

class Osiexporter
{
public:
    Osiexporter();
    ~Osiexporter();

    void addStaticObject(std::vector<Eigen::Vector3f> & v3d, std::vector<Eigen::Vector2f> & base_polygon, uint64_t id, std::string type);
    void setFrameTime(uint32_t seconds, uint32_t nanos);
    void writeFrame();

    osi3::GroundTruth * gt_;
    osi3::SensorView sv_;

    std::ofstream ofs_;
};