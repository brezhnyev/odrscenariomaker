#include "Osiexporter.h"
#include "odr_1_5.hpp"

#include <osi3/osi_object.pb.h>

#include <map>
#include <iostream>

#define DEG2RAD M_PI/180

using namespace osi3;
using namespace std;
using namespace odr_1_5;

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace crpc = carla::rpc;

static map<string, StationaryObject_Classification_Type> str2OsiType
{
    {"unknown", StationaryObject_Classification::TYPE_UNKNOWN },
    {"other", StationaryObject_Classification::TYPE_OTHER },
    {"bridge", StationaryObject_Classification::TYPE_BRIDGE },
    {"building", StationaryObject_Classification::TYPE_BUILDING },
    {"pole", StationaryObject_Classification::TYPE_POLE },
    {"pylon", StationaryObject_Classification::TYPE_PYLON },
    {"delineator", StationaryObject_Classification::TYPE_DELINEATOR },
    {"tree", StationaryObject_Classification::TYPE_TREE },
    {"barrier", StationaryObject_Classification::TYPE_BARRIER },
    {"vegetation", StationaryObject_Classification::TYPE_VEGETATION },
    {"curbstone", StationaryObject_Classification::TYPE_CURBSTONE },
    {"wall", StationaryObject_Classification::TYPE_WALL },
    {"vertical_structure", StationaryObject_Classification::TYPE_VERTICAL_STRUCTURE },
    {"rectangular_structure", StationaryObject_Classification::TYPE_RECTANGULAR_STRUCTURE },
    {"overhead_structure", StationaryObject_Classification::TYPE_OVERHEAD_STRUCTURE },
    {"reflective_structure", StationaryObject_Classification::TYPE_REFLECTIVE_STRUCTURE },
    {"construction_site_element", StationaryObject_Classification::TYPE_CONSTRUCTION_SITE_ELEMENT },
    //{"speed_bump", StationaryObject_Classification::TYPE_SPEED_BUMP },
};

Osiexporter::Osiexporter()
{
    gt_ = new GroundTruth();
    sv_.set_allocated_global_ground_truth(gt_);
    ofs_.open("output.osi", ios::binary);
}

Osiexporter::~Osiexporter()
{
    ofs_.close();
}

void Osiexporter::writeFrame()
{
    string frame = sv_.SerializeAsString();
    
    uint32_t size = frame.size();
    ofs_.write((char*)(&size), sizeof(uint32_t));
    ofs_.write(frame.c_str(), frame.size());
}

void Osiexporter::setFrameTime(uint32_t seconds, uint32_t nanos)
{
    Timestamp * ts = new Timestamp();
    ts->set_seconds(seconds); ts->set_nanos(nanos);
    gt_->set_allocated_timestamp(ts);
    gt_->clear_moving_object();
}

string Osiexporter::toValidType(string type)
{
    string typelow;
    for (auto && c : type) typelow.push_back(tolower(c));
    for (auto && el : str2OsiType)
    {
        if (typelow.find(el.first) != string::npos)
            return el.first;
    }
    return "";
}

void Osiexporter::addStaticObject(std::vector<Eigen::Vector3f> & v3d, std::vector<Eigen::Vector2f> & base_polygon, uint64_t & id, string type)
{
    StationaryObject * object = gt_->add_stationary_object();
    // set id:
    Identifier * oid = new Identifier();
    oid->set_value(id++); object->set_allocated_id(oid);
    // set classification:
    StationaryObject_Classification * classification = new StationaryObject_Classification();
    classification->set_type(str2OsiType[type]);
    classification->set_color(StationaryObject_Classification_Color_COLOR_YELLOW);
    object->set_allocated_classification(classification);
    
    // set stationary base:
    BaseStationary * base = new BaseStationary();

    float minx = __FLT_MAX__, maxx = -__FLT_MAX__, miny = __FLT_MAX__, maxy = -__FLT_MAX__, minz = __FLT_MAX__, maxz = -__FLT_MAX__;
    for (auto && v : v3d)
    {
        if (v.x() < minx) minx = v.x();
        if (v.x() > maxx) maxx = v.x();
        if (v.y() < miny) miny = v.y();
        if (v.y() > maxy) maxy = v.y();
        if (v.z() < minz) minz = v.z();
        if (v.z() > maxz) maxz = v.z();
    }

    // set bounding box:
    Dimension3d * bbox = new Dimension3d();
    bbox->set_length(maxx - minx); bbox->set_width(maxy - miny); bbox->set_height(maxz - minz);
    base->set_allocated_dimension(bbox);
 
    // set position:
    Vector3d * pos =  new Vector3d();
    pos->set_x(0.5f*(minx+maxx)); pos->set_y(0.5f*(miny+maxy)); pos->set_z(0.5f*(minz+maxz));
    base->set_allocated_position(pos);

    // set orientation:
    Orientation3d * ori = new Orientation3d();
    ori->set_roll(0); ori->set_pitch(0); ori->set_yaw(M_PI);
    base->set_allocated_orientation(ori);

    // set base_polygon:
    for (auto && v : base_polygon)
    {
        Vector2d * p2d = base->add_base_polygon();
        p2d->set_x(v.x() - pos->x()), p2d->set_y(v.y() - pos->y());
    }

    // add base to stationary object:
    object->set_allocated_base(base);

    return;

}

void Osiexporter::addRoads(const OpenDRIVE & odr, uint64_t & id, vector<vector<Eigen::Vector2f>> & vizCenterLines, vector<vector<Eigen::Vector2f>> & vizBoundaries)
{
    // export center line:
    vizCenterLines.reserve(odr.sub_road.size());
    vizBoundaries.reserve(odr.sub_road.size());


    for (auto && odr_road : odr.sub_road)
    {
        map<int, LaneBoundary*> bmap;
        map<int, Lane_Classification*> lmap;
        map<int, vector<Eigen::Vector2f>> vizBoundary;
        map<int, vector<Eigen::Vector2f>> vizCenter;

        //if (*road._name != "Taunusstraße") continue;
        for (auto && odr_lane : odr_road.sub_lanes->sub_laneSection)
        {
            auto addboundary = [&](int laneid, LaneBoundary * osiboundary)
            {
                Identifier * oid = new Identifier();
                oid->set_value(id++); osiboundary->set_allocated_id(oid);
                bmap[laneid] = osiboundary; // should be id=0 for center lane
            };
            auto addlane = [&](int laneid, Lane * osilane)
            {
                Identifier * oid = new Identifier();
                oid->set_value(id++); osilane->set_allocated_id(oid);
                Lane_Classification * lc = new Lane_Classification();
                osilane->set_allocated_classification(lc);
                lmap[laneid] = lc; // should be id=0 for center lane
            };
            if (odr_lane.sub_center)
                for (auto && odr_sublane : odr_lane.sub_center->sub_lane) // should be just one center
                {
                    addboundary(*odr_sublane._id, gt_->add_lane_boundary()); // should be lineid = 0 for center
                }
            if (odr_lane.sub_left)
                for (auto && odr_sublane : odr_lane.sub_left->sub_lane)
                {
                    addboundary(*odr_sublane._id, gt_->add_lane_boundary());
                    addlane(*odr_sublane._id, gt_->add_lane());
                }
            if (odr_lane.sub_right)
                for (auto && odr_sublane : odr_lane.sub_right->sub_lane)
                {
                    addboundary(*odr_sublane._id, gt_->add_lane_boundary());
                    addlane(*odr_sublane._id, gt_->add_lane());
                }
        }

        for (auto && odr_RefPoint : odr_road.sub_planView->sub_geometry)
        {
            // starting matrix for this segment:
            Eigen::Matrix4d M; M.setIdentity();
            M.block(0,0,3,3) = Eigen::AngleAxisd(*odr_RefPoint._hdg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            M.block(0,3,3,1) = Eigen::Vector3d(*odr_RefPoint._x, *odr_RefPoint._y, 0.0);

            // for OSI subdivide the length into segments of 1 meter:
            for (double s = 0; s < (*odr_RefPoint._length + 1); ++s)
            {
                Eigen::Vector4d P(0,0,0,1);

                if (s > *odr_RefPoint._length)
                    s = *odr_RefPoint._length;

                if (odr_RefPoint.sub_arc)
                {

                }
                else if (odr_RefPoint.sub_line)
                {
                    P = M*Eigen::Vector4d(s, 0.0, 0.0, 1.0);
                }
                else if (odr_RefPoint.sub_paramPoly3 && odr_RefPoint._length)
                {
                    auto poly3 = odr_RefPoint.sub_paramPoly3;
                    double t = s/(*odr_RefPoint._length);
                    P = M*Eigen::Vector4d(
                        *poly3->_aU + *poly3->_bU*t + *poly3->_cU*t*t + *poly3->_dU*t*t*t,
                        *poly3->_aV + *poly3->_bV*t + *poly3->_cV*t*t + *poly3->_dV*t*t*t,
                        0.0,
                        1.0);
                }
                else if (odr_RefPoint.sub_poly3 && odr_RefPoint._length)
                {
                    auto poly3 = odr_RefPoint.sub_paramPoly3;
                    double t = s/(*odr_RefPoint._length);
                    P = M*Eigen::Vector4d(
                        s,
                        *poly3->_aV + *poly3->_bV*t + *poly3->_cV*t*t + *poly3->_dV*t*t*t,
                        0.0,
                        1.0);
                }
                else if (odr_RefPoint.sub_spiral)
                {

                }

                P = P - Eigen::Vector4d(520, 87, 0, 0);

                for (auto && odr_lane : odr_road.sub_lanes->sub_laneSection)
                {
                    if (odr_lane.sub_center)
                        for (auto && odr_sublane : odr_lane.sub_center->sub_lane)
                        {
                            LaneBoundary_BoundaryPoint * bp = bmap[*odr_sublane._id]->add_boundary_line();
                            Vector3d * pos =  new Vector3d();
                            pos->set_x(P.x()); pos->set_y(P.y()); pos->set_z(P.z());
                            bp->set_allocated_position(pos);
                            vizBoundary[*odr_sublane._id].emplace_back(P.x(), P.y());
                        }
                    double width = 0;
                    if (odr_lane.sub_left)
                        for (auto && odr_sublane : odr_lane.sub_left->sub_lane)
                        {
                            // Boundary:
                            LaneBoundary_BoundaryPoint * bp = bmap[*odr_sublane._id]->add_boundary_line();
                            width += 4; // TODO!!!
                            Vector3d * pos =  new Vector3d();
                            Eigen::Vector3d Poff = P.block(0,0,3,1) + M.block(0,0,3,3)*Eigen::Vector3d(0,width,0);
                            pos->set_x(Poff.x()); pos->set_y(Poff.y()); pos->set_z(Poff.z());
                            bp->set_allocated_position(pos);
                            vizBoundary[*odr_sublane._id].emplace_back(Poff.x(), Poff.y());
                            // Center:
                            Vector3d * center = lmap[*odr_sublane._id]->add_centerline();
                            Poff = P.block(0,0,3,1) + M.block(0,0,3,3)*Eigen::Vector3d(0,width-2,0); // TODO!!!
                            center->set_x(Poff.x()); center->set_y(Poff.y()); center->set_z(0); // Z is 0 !!!
                            vizCenter[*odr_sublane._id].emplace_back(Poff.x(), Poff.y());

                        }
                    if (odr_lane.sub_right)
                        for (auto && odr_sublane : odr_lane.sub_right->sub_lane)
                        {
                            // Boundary:
                            LaneBoundary_BoundaryPoint * bp = bmap[*odr_sublane._id]->add_boundary_line();
                            width -= 4; // TODO!!!
                            Vector3d * pos =  new Vector3d();
                            Eigen::Vector3d Poff = P.block(0,0,3,1) + M.block(0,0,3,3)*Eigen::Vector3d(0,width,0);
                            pos->set_x(Poff.x()); pos->set_y(Poff.y()); pos->set_z(Poff.z());
                            bp->set_allocated_position(pos);
                            vizBoundary[*odr_sublane._id].emplace_back(Poff.x(), Poff.y());
                            // Center:
                            Vector3d * center = lmap[*odr_sublane._id]->add_centerline();
                            Poff = P.block(0,0,3,1) + M.block(0,0,3,3)*Eigen::Vector3d(0,width+2,0); // TODO!!!
                            center->set_x(Poff.x()); center->set_y(Poff.y()); center->set_z(0); // Z is 0 !!!
                            vizCenter[*odr_sublane._id].emplace_back(Poff.x(), Poff.y());
                        }
                }

                if (s == *odr_RefPoint._length)
                    break;
            }
            for (auto && b : vizBoundary)
                vizBoundaries.push_back(move(b.second));

            for (auto && c : vizCenter)
                vizCenterLines.push_back(move(c.second));
        }
        // for visual check:
    }
}

void Osiexporter::updateMovingObjects(carla::SharedPtr<cc::ActorList> actors, std::vector<Eigen::Matrix4f> & vizActors)
{
    for (auto && actor : *actors)
    {
        cg::Transform trf = actor->GetTransform();
        cg::BoundingBox bbox = actor->GetBoundingBox();
        // We will use the 4x4 matrix to fill out vizActors as follows:
        // M.block(0,0,3,3) - rotation
        // M.block(0,3,3,1) - translation
        // M.block(3,0,1,3) - scale
        // M.block(3,3,1,1) - type
        Eigen::Matrix4f M; M.setIdentity();

        M.block(0,0,3,3) = (
            Eigen::AngleAxisf(trf.rotation.yaw*DEG2RAD, Eigen::Vector3f::UnitZ())*
            Eigen::AngleAxisf(trf.rotation.pitch*DEG2RAD, Eigen::Vector3f::UnitY())*
            Eigen::AngleAxisf(trf.rotation.roll*DEG2RAD, Eigen::Vector3f::UnitX())).toRotationMatrix();

        M.block(0,3,3,1) = Eigen::Vector3f(trf.location.x, trf.location.y, trf.location.z);
        M.block(3,0,1,3) = Eigen::Vector3f(bbox.extent.x, bbox.extent.y, bbox.extent.z).transpose();
        // transform from Left-hand to Right-hand system:
        Eigen::Matrix4f mirror; mirror.setIdentity(); mirror(1,1) = -1;
        M = mirror*M;

        MovingObject * mo = gt_->add_moving_object();
        Identifier * oid = new Identifier();
        oid->set_value(actor->GetId()); mo->set_allocated_id(oid);

        // set type:
        if (dynamic_cast<cc::Vehicle*>(actor.get()))
        {
            mo->set_type(MovingObject_Type_TYPE_VEHICLE);
            // classification of vehicle:
            MovingObject_VehicleClassification * classification = new MovingObject_VehicleClassification();
            classification->set_type(MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR);
            mo->set_allocated_vehicle_classification(classification);
            M(3,3) = 0.0f;
        }
        else if (dynamic_cast<cc::Walker*>(actor.get()))
        {
            mo->set_type(MovingObject_Type_TYPE_PEDESTRIAN);
            M(3,3) = 1.0f;
        }

        vizActors.push_back(M);

        // set moving base:
        BaseMoving * base = new BaseMoving();
        // set bounding box:
        Dimension3d * dim = new Dimension3d();
        dim->set_length(2*bbox.extent.x); dim->set_width(2*bbox.extent.y); dim->set_height(2*bbox.extent.z);
        base->set_allocated_dimension(dim);

        // set position:
        Vector3d * pos =  new Vector3d();
        pos->set_x(M(0,3)); pos->set_y(M(1,3)); pos->set_z(M(2,3));
        base->set_allocated_position(pos);

        // set orientation:
        Orientation3d * ori = new Orientation3d();
        ori->set_roll(trf.rotation.roll*DEG2RAD); ori->set_pitch(trf.rotation.pitch*DEG2RAD); ori->set_yaw(-trf.rotation.yaw*DEG2RAD);
        base->set_allocated_orientation(ori);

        mo->set_allocated_base(base);
    }
}