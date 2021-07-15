#pragma once

#include <eigen3/Eigen/Eigen>
#include <osi3/osi_sensorview.pb.h>
#include "odrparser/odr_1_5.h"

#include <carla/client/Client.h>
#include <carla/client/ActorList.h>
#include <carla/Memory.h>

#include <vector>
#include <string>
#include <fstream>

class Osiexporter
{
public:
    Osiexporter();
    ~Osiexporter();

    void addStaticObject(std::vector<Eigen::Vector3f> & v3d, std::vector<Eigen::Vector2f> & base_polygon, uint64_t & id, std::string type);
    void addRoads(const odr_1_5::OpenDRIVE &, uint64_t & id, std::vector<std::vector<Eigen::Vector2f>> & vizCenterlines, std::vector<std::vector<Eigen::Vector2f>> & vizBoundaries);
    void updateMovingObjects(carla::SharedPtr<carla::client::ActorList> actors, std::vector<Eigen::Matrix4f> & vizActors);
    void setFrameTime(uint32_t seconds, uint32_t nanos);
    std::string toValidType(std::string);
    void writeFrame();

    osi3::GroundTruth * gt_;
    osi3::SensorView sv_;

    std::ofstream ofs_;
};