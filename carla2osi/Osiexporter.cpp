#include "Osiexporter.h"

#include <osi3/osi_object.pb.h>

#include <map>

using namespace osi3;
using namespace std;

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
    gt_->clear_stationary_object();
}

void Osiexporter::addStaticObject(std::vector<Eigen::Vector3f> & v3d, std::vector<Eigen::Vector2f> & base_polygon, uint64_t id, string type)
{
    StationaryObject * object = gt_->add_stationary_object();
    // set id:
    Identifier * oid = new Identifier();
    oid->set_value(id); object->set_allocated_id(oid);
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