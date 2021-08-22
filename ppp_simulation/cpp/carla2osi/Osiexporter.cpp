#include "Osiexporter.h"
#include "odr_1_5.hpp"

#include <osi3/osi_object.pb.h>

#include <map>
#include <iostream>

#define DEG2RAD M_PI/180

using namespace osi3;
using namespace std;
using namespace odr_1_5;

#ifdef USE_CARLA
namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace crpc = carla::rpc;
#endif

// will be filled out by customer (config.yaml)
static map<string, StationaryObject_Classification_Type> str2OsiType
{
    // {"unknown", StationaryObject_Classification::TYPE_UNKNOWN },
    // {"other", StationaryObject_Classification::TYPE_OTHER },
    // {"bridge", StationaryObject_Classification::TYPE_BRIDGE },
    // {"building", StationaryObject_Classification::TYPE_BUILDING },
    // {"house", StationaryObject_Classification::TYPE_BUILDING },
    // {"pole", StationaryObject_Classification::TYPE_POLE },
    // {"pylon", StationaryObject_Classification::TYPE_PYLON },
    // {"delineator", StationaryObject_Classification::TYPE_DELINEATOR },
    // {"tree", StationaryObject_Classification::TYPE_TREE },
    // {"barrier", StationaryObject_Classification::TYPE_BARRIER },
    // {"vegetation", StationaryObject_Classification::TYPE_VEGETATION },
    // {"curbstone", StationaryObject_Classification::TYPE_CURBSTONE },
    // {"wall", StationaryObject_Classification::TYPE_WALL },
    // {"vertical_structure", StationaryObject_Classification::TYPE_VERTICAL_STRUCTURE },
    // {"rectangular_structure", StationaryObject_Classification::TYPE_RECTANGULAR_STRUCTURE },
    // {"overhead_structure", StationaryObject_Classification::TYPE_OVERHEAD_STRUCTURE },
    // {"reflective_structure", StationaryObject_Classification::TYPE_REFLECTIVE_STRUCTURE },
    // {"construction_site_element", StationaryObject_Classification::TYPE_CONSTRUCTION_SITE_ELEMENT },
    // {"speed_bump", StationaryObject_Classification::TYPE_SPEED_BUMP },
};

static map<string, LaneBoundary_Classification_Color> str2OsiColor
{
    {"standard", LaneBoundary_Classification_Color_COLOR_WHITE},
    {"while", LaneBoundary_Classification_Color_COLOR_WHITE},
    {"yellow", LaneBoundary_Classification_Color_COLOR_YELLOW},
    {"red", LaneBoundary_Classification_Color_COLOR_RED},
    {"blue", LaneBoundary_Classification_Color_COLOR_BLUE},
    {"green", LaneBoundary_Classification_Color_COLOR_GREEN}
};

static map<string, LaneBoundary_Classification_Type> str2OsiBoudary
{
    {"none",LaneBoundary_Classification_Type_TYPE_NO_LINE},
    {"solid",LaneBoundary_Classification_Type_TYPE_SOLID_LINE},
    {"broken",LaneBoundary_Classification_Type_TYPE_DASHED_LINE},
    {"botts dots",LaneBoundary_Classification_Type_TYPE_BOTTS_DOTS},
    {"grass",LaneBoundary_Classification_Type_TYPE_GRASS_EDGE},
    {"curb",LaneBoundary_Classification_Type_TYPE_CURB}
};

static map<string, Lane_Classification_Subtype> str2LaneSubType
{
    {"stop", Lane_Classification_Subtype_SUBTYPE_STOP},
    {"shoulder", Lane_Classification_Subtype_SUBTYPE_SHOULDER},
    {"biking", Lane_Classification_Subtype_SUBTYPE_BIKING},
    {"sidewalk", Lane_Classification_Subtype_SUBTYPE_SIDEWALK},
    {"border", Lane_Classification_Subtype_SUBTYPE_BORDER},
    {"restricted", Lane_Classification_Subtype_SUBTYPE_RESTRICTED},
    {"parking", Lane_Classification_Subtype_SUBTYPE_PARKING},
    {"entry", Lane_Classification_Subtype_SUBTYPE_ENTRY},
    {"exit", Lane_Classification_Subtype_SUBTYPE_EXIT},
    {"offRamp", Lane_Classification_Subtype_SUBTYPE_OFFRAMP},
    {"onRamp", Lane_Classification_Subtype_SUBTYPE_ONRAMP}
};

#ifdef USE_CARLA
static map<carla::rpc::TrafficLightState, TrafficLight_Classification_Color> carla2OsiTRLColor
{
  {carla::rpc::TrafficLightState::Unknown, TrafficLight_Classification_Color_COLOR_UNKNOWN},
  {carla::rpc::TrafficLightState::Red, TrafficLight_Classification_Color_COLOR_RED},
  {carla::rpc::TrafficLightState::Yellow, TrafficLight_Classification_Color_COLOR_YELLOW},
  {carla::rpc::TrafficLightState::Green, TrafficLight_Classification_Color_COLOR_GREEN}
};

static map<string, MovingObject_VehicleClassification_Type> string2OsiVehicleType
{
    {"vehicle.audi.a2", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR},
    {"vehicle.tesla.model3", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR},
    {"vehicle.bh.crossbike", MovingObject_VehicleClassification_Type_TYPE_BICYCLE},
    {"vehicle.bmw.grandtourer", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR},
    {"vehicle.audi.etron", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR},
    {"vehicle.seat.leon", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR},
    {"vehicle.volkswagen.t2", MovingObject_VehicleClassification_Type_TYPE_DELIVERY_VAN},
    {"vehicle.kawasaki.ninja", MovingObject_VehicleClassification_Type_TYPE_MOTORBIKE},
    {"vehicle.mustang.mustang", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.tesla.cybertruck", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.lincoln.mkz2017", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.lincoln2020.mkz2020", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.dodge_charger.police", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR},
    {"vehicle.gazelle.omafiets", MovingObject_VehicleClassification_Type_TYPE_BICYCLE},
    {"vehicle.yamaha.yzf", MovingObject_VehicleClassification_Type_TYPE_MOTORBIKE},
    {"vehicle.audi.tt", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.jeep.wrangler_rubicon", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.harley-davidson.low_rider", MovingObject_VehicleClassification_Type_TYPE_MOTORBIKE},
    {"vehicle.chevrolet.impala", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR},
    {"vehicle.nissan.patrol", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.nissan.micra", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR},
    {"vehicle.mercedesccc.mercedesccc", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.bmw.isetta", MovingObject_VehicleClassification_Type_TYPE_SMALL_CAR},
    {"vehicle.mini.cooperst", MovingObject_VehicleClassification_Type_TYPE_SMALL_CAR},
    {"vehicle.chargercop2020.chargercop2020", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.toyota.prius", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR},
    {"vehicle.mercedes-benz.coupe", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.diamondback.century", MovingObject_VehicleClassification_Type_TYPE_BICYCLE},
    {"vehicle.citroen.c3", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR},
    {"vehicle.charger2020.charger2020", MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR},
    {"vehicle.carlamotors.carlacola", MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR}
};
#endif

// https://opensimulationinterface.github.io/osi-documentation/open-simulation-interface/structosi3_1_1TrafficSign.html#exhale-struct-structosi3-1-1trafficsign
static map<string, TrafficSign_MainSign_Classification_Type> string2OsiMainTS
{
    {"unknown", TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN},
    {"traffic.speed_limit", TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN},
    {"traffic.yield", TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY},
    {"traffic.stop", TrafficSign_MainSign_Classification_Type_TYPE_STOP}
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

void Osiexporter::extendStaticNames(const map<string, int> & customNames)
{
    for (auto && n : customNames)
    {
        str2OsiType[n.first] = (StationaryObject_Classification_Type)n.second;
        cout << "Added custom static object name: " << n.first << " " << n.second << endl;
    }
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
    gt_->clear_traffic_light(); // eventually only state update is needed (not delete)
    gt_->clear_traffic_sign();  // eventually the delete is not needed at all
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

void Osiexporter::addStaticObject(const std::vector<Eigen::Vector3f> & v3d, const std::vector<Eigen::Vector2f> & base_polygon, uint64_t & id, string type, float scale)
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
    bbox->set_length(scale*(maxx - minx)); bbox->set_width(scale*(maxy - miny)); bbox->set_height(scale*(maxz - minz));
    base->set_allocated_dimension(bbox);
 
    // set position:
    Vector3d * pos =  new Vector3d();
    pos->set_x(0.5f*(minx+maxx)*scale); pos->set_y(0.5f*(miny+maxy)*scale); pos->set_z(0.5f*(minz+maxz)*scale);
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

void Osiexporter::addRoads(const OpenDRIVE & odr, uint64_t & id, vector<vector<Eigen::Vector3f>> & vizCenterLines, vector<vector<Eigen::Vector3f>> & vizBoundaries)
{
    // export center line:
    vizCenterLines.reserve(odr.sub_road.size());
    vizBoundaries.reserve(odr.sub_road.size());


    for (auto && odr_road : odr.sub_road)
    {
        // key1 = section id of the road, key2 = lane id of the section
        map<int, map<int, LaneBoundary*>> bmap;
        map<int, map<int, Lane_Classification*>> lmap;
        map<int, map<int, vector<Eigen::Vector3f>>> vizBoundary;
        map<int, map<int, vector<Eigen::Vector3f>>> vizCenter;

        //if (*road._name != "Taunusstraße") continue;
        uint32_t sindex = 0; // sections index (for storing for rendering)
        for (auto && odr_lane : odr_road.sub_lanes->sub_laneSection)
        {
            auto addboundary = [&](int sindex, int laneid, LaneBoundary * osiboundary)
            {
                Identifier * oid = new Identifier();
                oid->set_value(id++); osiboundary->set_allocated_id(oid);
                bmap[sindex][laneid] = osiboundary; // should be id=0 for center lane
            };
            auto addlane = [&](int sindex, int laneid, Lane * osilane)
            {
                Identifier * oid = new Identifier();
                oid->set_value(id++); osilane->set_allocated_id(oid);
                Lane_Classification * lc = new Lane_Classification();
                osilane->set_allocated_classification(lc);
                lmap[sindex][laneid] = lc; // should be id=0 for center lane
            };
            if (odr_lane.sub_center)
                for (auto && odr_sublane : odr_lane.sub_center->sub_lane) // should be just one center
                {
                    addboundary(sindex, *odr_sublane._id, gt_->add_lane_boundary()); // should be lineid = 0 for center
                }
            if (odr_lane.sub_left)
                for (auto && odr_sublane : odr_lane.sub_left->sub_lane)
                {
                    addboundary(sindex, *odr_sublane._id, gt_->add_lane_boundary());
                    addlane(sindex, *odr_sublane._id, gt_->add_lane());
                }
            if (odr_lane.sub_right)
                for (auto && odr_sublane : odr_lane.sub_right->sub_lane)
                {
                    addboundary(sindex, *odr_sublane._id, gt_->add_lane_boundary());
                    addlane(sindex, *odr_sublane._id, gt_->add_lane());
                }
            ++sindex;
        }

        double S = 0; // total length of road
        sindex = 0; // sections index (for storing for rendering)

        auto sit = odr_road.sub_lanes->sub_laneSection.begin(); // sections iterator
        for (auto && odr_subroad : odr_road.sub_planView->sub_geometry)
        {
            // starting matrix for this segment:
            Eigen::Matrix4d M; M.setIdentity();
            M.block(0,0,3,3) = Eigen::AngleAxisd(*odr_subroad._hdg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            M.block(0,3,3,1) = Eigen::Vector3d(*odr_subroad._x, *odr_subroad._y, 0.0);
            Eigen::Vector4d velocity; velocity.setZero();
            Eigen::Vector4d normal; normal.setZero();

            auto buildSubroad = [&](double s, double S)
            {
                Eigen::Vector4d P(0, 0, 0, 1);

                if (odr_subroad.sub_arc)
                {
                    double R = 1.0/(*odr_subroad.sub_arc->_curvature);
                    double radians = s / R;
                    M.block(0,0,3,3) = Eigen::AngleAxisd(*odr_subroad._hdg - M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

                    P = Eigen::Vector4d(R*cos(radians) - R, R*sin(radians), 0.0, 1.0);
                    // velocity as first derivative of position:
                    velocity.x() =-R*sin(radians);
                    velocity.y() = R*cos(radians);
                    if (R < 0) velocity = -velocity;
                }
                else if (odr_subroad.sub_line)
                {
                    P = Eigen::Vector4d(s, 0.0, 0.0, 1.0);
                    // velocity:
                    velocity.x() = 1;
                    velocity.y() = 0;
                }
                else if (odr_subroad.sub_paramPoly3 && odr_subroad._length)
                {
                    auto poly3 = odr_subroad.sub_paramPoly3;
                    double t = s / (*odr_subroad._length);
                    P = Eigen::Vector4d(
                        *poly3->_aU + *poly3->_bU * t + *poly3->_cU * t * t + *poly3->_dU * t * t * t,
                        *poly3->_aV + *poly3->_bV * t + *poly3->_cV * t * t + *poly3->_dV * t * t * t,
                        0.0,
                        1.0);
                    // velocity as first derivative of position:
                    velocity.x() = *poly3->_bU + *poly3->_cU * 2 * t + *poly3->_dU * 3 * t * t;
                    velocity.y() = *poly3->_bV + *poly3->_cV * 2 * t + *poly3->_dV * 3 * t * t;
                }
                else if (odr_subroad.sub_poly3 && odr_subroad._length)
                {
                    auto poly3 = odr_subroad.sub_paramPoly3;
                    P = Eigen::Vector4d(
                        s,
                        *poly3->_aV + *poly3->_bV * s + *poly3->_cV * s * s + *poly3->_dV * s * s * s,
                        0.0,
                        1.0);
                    // velocity as first derivative of position:
                    velocity.x() = 1;
                    velocity.y() = *poly3->_bV + *poly3->_cV * 2 * s + *poly3->_dV * 3 * s * s;
                }
                else if (odr_subroad.sub_spiral)
                {
                    return;
                }
                else
                    return;


                auto polyInter = [&](double T, auto & polis, double (*getS)(void * itt) ) -> double
                {
                    auto pit = polis.begin(); // polis iterator
                    while (pit != polis.end() && getS(&(*pit)) < T) ++pit;
                    if (pit != polis.begin()) --pit;
                    double t = T - getS(&(*pit));
                    return *(pit->_a) + *(pit->_b)*t + *(pit->_c)*t*t + *(pit->_d)*t*t*t;
                };

                // Super-elevation:
                if (odr_road.sub_lateralProfile && !odr_road.sub_lateralProfile->sub_superelevation.empty())
                {
                    double roll = polyInter(S, odr_road.sub_lateralProfile->sub_superelevation, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_lateralProfile->sub_superelevation[0])>(it)->_s; });
                    M.block(0,0,3,3) = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix()*M.block(0,0,3,3);
                }
                // Elevation:
                if (odr_road.sub_elevationProfile && !odr_road.sub_elevationProfile->sub_elevation.empty())
                    P.z() = polyInter(S, odr_road.sub_elevationProfile->sub_elevation, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_elevationProfile->sub_elevation[0])>(it)->_s; });

                //P = P - Eigen::Vector4d(520, 87, 0, 0);
                // left normal:
                normal.x() =-velocity.y();
                normal.y() = velocity.x();
                normal.normalize();

                // find out if we are at the proper section of the road:
                ++sit; ++sindex;
                if (sit == odr_road.sub_lanes->sub_laneSection.end() || *sit->_s > S) { --sit; --sindex; }
                auto && odr_lane = *sit;

                // set the lanes:
                double offset = 0.0;
                if (odr_road.sub_lanes && !odr_road.sub_lanes->sub_laneOffset.empty())
                    offset = polyInter(S, odr_road.sub_lanes->sub_laneOffset, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_lanes->sub_laneOffset[0])>(it)->_s; });
                
                if (odr_lane.sub_center)
                    for (auto && odr_sublane : odr_lane.sub_center->sub_lane)
                    {
                        // ONLY 1 center line, actually loop not needed
                        LaneBoundary_BoundaryPoint *bp = bmap[sindex][*odr_sublane._id]->add_boundary_line();
                        Vector3d *pos = new Vector3d();
                        Eigen::Vector4d Ptrf = M * (P + normal*offset);
                        pos->set_x(Ptrf.x());
                        pos->set_y(Ptrf.y());
                        pos->set_z(Ptrf.z());
                        bp->set_allocated_position(pos);
                        // Adding type and color properly for all xodr elements would mean subdividing the OSI boundaries (TODO)
                        if (!odr_sublane.sub_roadMark.empty())
                        {
                            LaneBoundary_Classification * lbclassification = new LaneBoundary_Classification();
                            if (odr_sublane.sub_roadMark[0]._width)
                                bp->set_width(*odr_sublane.sub_roadMark[0]._width); // TODO
                            if (odr_sublane.sub_roadMark[0]._type)
                                lbclassification->set_type(str2OsiBoudary[*odr_sublane.sub_roadMark[0]._type]); // TODO: for all elements!!!
                            if (odr_sublane.sub_roadMark[0]._color)
                                lbclassification->set_color(str2OsiColor[*odr_sublane.sub_roadMark[0]._color]); // TODO: for all elements!!!
                            bmap[sindex][*odr_sublane._id]->set_allocated_classification(lbclassification);
                        }
                        vizBoundary[sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z());
                    }
                auto buildLane = [&](auto & odr_sublane, int dir, double & twidth)
                {
                    // Boundary:
                    LaneBoundary_BoundaryPoint *bp = bmap[sindex][*odr_sublane._id]->add_boundary_line();
                    double width = polyInter(S - *odr_lane._s, odr_sublane.sub_width, [](void * it) ->double { return *static_cast<decltype(&odr_sublane.sub_width[0])>(it)->_sOffset; });
                    twidth += dir*width;
                    Vector3d *pos = new Vector3d();
                    Eigen::Vector4d Ptrf = M * (P + normal*twidth);
                    pos->set_x(Ptrf.x());
                    pos->set_y(Ptrf.y());
                    pos->set_z(Ptrf.z());
                    bp->set_allocated_position(pos);
                    // Adding type and color properly for all xodr elements would mean subdividing the OSI boundaries (TODO)
                    if (!odr_sublane.sub_roadMark.empty())
                    {
                        LaneBoundary_Classification * lbclassification = new LaneBoundary_Classification();
                        if (odr_sublane.sub_roadMark[0]._width)
                            bp->set_width(*odr_sublane.sub_roadMark[0]._width); // TODO
                        if (odr_sublane.sub_roadMark[0]._type)
                            lbclassification->set_type(str2OsiBoudary[*odr_sublane.sub_roadMark[0]._type]); // TODO: for all elements!!!
                        if (odr_sublane.sub_roadMark[0]._color)
                            lbclassification->set_color(str2OsiColor[*odr_sublane.sub_roadMark[0]._color]); // TODO: for all elements!!!
                        bmap[sindex][*odr_sublane._id]->set_allocated_classification(lbclassification);
                    }
                    vizBoundary[sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z());
                    // Center:
                    Vector3d *center = lmap[sindex][*odr_sublane._id]->add_centerline();
                    Ptrf = M * (P + normal*(twidth - dir*width/2));
                    center->set_x(Ptrf.x());
                    center->set_y(Ptrf.y());
                    center->set_z(Ptrf.z());
                    vizCenter[sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z());
                    if (odr_sublane._type)
                        lmap[sindex][*odr_sublane._id]->set_subtype(str2LaneSubType[*odr_sublane._type]);
                };
                double twidth = offset;
                if (odr_lane.sub_left)
                {
                    map<int, t_road_lanes_laneSection_left_lane> sublanes_map;
                    for (auto && odr_sublane : odr_lane.sub_left->sub_lane) sublanes_map[abs<int>(*odr_sublane._id)] = odr_sublane;
                    for (auto && it : sublanes_map) buildLane(it.second, 1, twidth);
                }
                twidth = offset;
                if (odr_lane.sub_right)
                {
                    map<int, t_road_lanes_laneSection_right_lane> sublanes_map;
                    for (auto && odr_sublane : odr_lane.sub_right->sub_lane) sublanes_map[abs<int>(*odr_sublane._id)] = odr_sublane;
                    for (auto && it : sublanes_map) buildLane(it.second, -1, twidth);
                }
            };
            double prevS = S;
            for (double s = 0; s < *odr_subroad._length; s++, S++)
            {
                buildSubroad(s, S);
            }
            S = prevS + *odr_subroad._length;
            buildSubroad(*odr_subroad._length, S);

            for (auto && bb : vizBoundary)
                for (auto && b : bb.second)
                    vizBoundaries.push_back(move(b.second));

            for (auto && cc : vizCenter)
                for (auto && c : cc.second)
                    vizCenterLines.push_back(move(c.second));
        }
    }
}

#ifdef USE_CARLA
void Osiexporter::updateMovingObjects(carla::SharedPtr<cc::ActorList> actors, std::vector<Eigen::Matrix4f> & vizActors)
{
    for (auto &&actor : *actors)
    {
        if (
            dynamic_cast<cc::Vehicle *>(actor.get()) ||
            dynamic_cast<cc::Walker *>(actor.get()) ||
            dynamic_cast<cc::TrafficSign *>(actor.get()))
        {
            cg::Transform trf = actor->GetTransform();
            cg::BoundingBox bbox = actor->GetBoundingBox();
            // some bykes and cycles have dimension y == 0, set to 1 meter width (extent.y = 0.5):
            if (!bbox.extent.y)
                bbox.extent.y = 0.5;
            // We will use the 4x4 matrix to fill out vizActors as follows:
            // M.block(0,0,3,3) - rotation
            // M.block(0,3,3,1) - translation
            // M.block(3,0,1,3) - scale (bbox)
            // M.block(3,3,1,1) - type
            Eigen::Matrix4f M;
            M.setIdentity();

            M.block(0, 0, 3, 3) = (Eigen::AngleAxisf(trf.rotation.yaw * DEG2RAD, Eigen::Vector3f::UnitZ()) *
                                   Eigen::AngleAxisf(trf.rotation.pitch * DEG2RAD, Eigen::Vector3f::UnitY()) *
                                   Eigen::AngleAxisf(trf.rotation.roll * DEG2RAD, Eigen::Vector3f::UnitX()))
                                      .toRotationMatrix();

            M.block(0, 3, 3, 1) = Eigen::Vector3f(trf.location.x, trf.location.y, trf.location.z);
            M.block(3, 0, 1, 3) = Eigen::Vector3f(bbox.extent.x, bbox.extent.y, bbox.extent.z).transpose();
            // transform from Left-hand to Right-hand system:
            Eigen::Matrix4f mirror;
            mirror.setIdentity();
            mirror(1, 1) = -1;
            M = mirror * M;

            // set bounding box:
            Dimension3d *dim = new Dimension3d();
            dim->set_length(2 * bbox.extent.x);
            dim->set_width(2 * bbox.extent.y);
            dim->set_height(2 * bbox.extent.z);

            // set position:
            Vector3d *pos = new Vector3d();
            pos->set_x(M(0, 3));
            pos->set_y(M(1, 3));
            pos->set_z(M(2, 3));

            // set orientation:
            Orientation3d *ori = new Orientation3d();
            ori->set_roll(trf.rotation.roll * DEG2RAD);
            ori->set_pitch(trf.rotation.pitch * DEG2RAD);
            ori->set_yaw(-trf.rotation.yaw * DEG2RAD);

            Identifier *oid = new Identifier();
            oid->set_value(actor->GetId());

            if (dynamic_cast<cc::Vehicle *>(actor.get()) || dynamic_cast<cc::Walker *>(actor.get()))
            {
                MovingObject *mo = gt_->add_moving_object();
                mo->set_allocated_id(oid);

                // set moving base:
                BaseMoving *base = new BaseMoving();
                base->set_allocated_dimension(dim);
                base->set_allocated_position(pos);
                base->set_allocated_orientation(ori);

                mo->set_allocated_base(base);

                // set type:
                if (dynamic_cast<cc::Vehicle *>(actor.get()))
                {
                    mo->set_type(MovingObject_Type_TYPE_VEHICLE);
                    // classification of vehicle:
                    MovingObject_VehicleClassification *classification = new MovingObject_VehicleClassification();
                    // KB: this may be unreliable if the Display ID format is changed in Carla:
                    // example of current GetDisplayId():
                    // Actor 250 (vehicle.harley-davidson.low_rider)
                    // |         |
                    // 0         11
                    string name = actor->GetDisplayId().substr(11, actor->GetDisplayId().size() - 12);
                    classification->set_type(string2OsiVehicleType[name]);
                    mo->set_allocated_vehicle_classification(classification);
                    M(3, 3) = 0.0f;
                }
                else if (dynamic_cast<cc::Walker *>(actor.get()))
                {
                    mo->set_type(MovingObject_Type_TYPE_PEDESTRIAN);
                    M(3, 3) = 1.0f;
                }
            }
            else if (dynamic_cast<cc::TrafficLight *>(actor.get()))
            {
                cc::TrafficLight *carla_tfl = dynamic_cast<cc::TrafficLight *>(actor.get());
                switch (carla_tfl->GetState())
                {
                case carla::rpc::TrafficLightState::Red:
                    M(3, 3) = 2.0f;
                    break;
                case carla::rpc::TrafficLightState::Yellow:
                    M(3, 3) = 3.0f;
                    break;
                case carla::rpc::TrafficLightState::Green:
                    M(3, 3) = 4.0f;
                    break;
                default:
                    M(3, 3) = 2.0f;
                    break;
                }

                TrafficLight *osi_tfl = gt_->add_traffic_light();
                osi_tfl->set_allocated_id(oid);
                BaseStationary * base = new BaseStationary();
                base->set_allocated_dimension(dim);
                base->set_allocated_position(pos);
                base->set_allocated_orientation(ori);
                osi_tfl->set_allocated_base(base);

                TrafficLight_Classification * classification = new TrafficLight_Classification();
                classification->set_color(carla2OsiTRLColor[carla_tfl->GetState()]);
                osi_tfl->set_allocated_classification(classification);
            }
            else if (dynamic_cast<cc::TrafficSign *>(actor.get()))
            {
                TrafficSign_MainSign_Classification_Type signType;
                if (actor->GetDisplayId().find("speed") != string::npos)
                {
                    signType = TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN;
                    M(3,3) = 5.0f;
                }
                if (actor->GetDisplayId().find("yield") != string::npos)
                {
                    signType = TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY;
                    M(3,3) = 6.0f;
                }
                if (actor->GetDisplayId().find("stop") != string::npos)
                {
                    signType = TrafficSign_MainSign_Classification_Type_TYPE_STOP;
                    M(3,3) = 7.0f;
                }
                if (actor->GetDisplayId().find("unknown") != string::npos)
                {
                    signType = TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN;
                    M(3,3) = 8.0f;
                }

                cc::TrafficSign *carla_tfs = dynamic_cast<cc::TrafficSign *>(actor.get());

                TrafficSign *osi_tfs = gt_->add_traffic_sign();
                osi_tfs->set_allocated_id(oid);
                BaseStationary * base = new BaseStationary();
                base->set_allocated_dimension(dim);
                base->set_allocated_position(pos);
                base->set_allocated_orientation(ori);
                TrafficSign_MainSign * main_sign = new TrafficSign_MainSign();
                main_sign->set_allocated_base(base);
                TrafficSign_MainSign_Classification * msg_classification = new TrafficSign_MainSign_Classification();
                msg_classification->set_type(signType);
                main_sign->set_allocated_classification(msg_classification);
                osi_tfs->set_allocated_main_sign(main_sign);
            }
            vizActors.push_back(M);
        }
    }
}
#endif