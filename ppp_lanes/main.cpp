#include <iostream>

#include "FolderReader.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <eigen3/Eigen/Eigen>

#include <deque>
#include <vector>
#include <fstream>

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace Eigen;

struct __attribute__((packed)) Point
{
    float x, y, z;
    int32_t intensity;
    int32_t t_lo, t_hi;
};

int main()
{
    FolderReader<> fr("./data");
    Matrix4f M;
    M.setIdentity();
    static_assert(sizeof(Point) == 24);

    map<string, string> topics = {
        {"cont", "/LanePoints/cont_lane"},
        {"dash", "/LanePoints/dash_lane"},
        {"curb", "/LanePoints/curbstone"},
        {"stop", "/LanePoints/stop_lane"}};

    while (true)
    {
        string fileName = fr.getNext("bag");
        if (fileName.empty())
            break;

        for (auto &&t : topics)
        {
            rosbag::Bag bag;
            bag.open(fileName);
            auto &&messages = rosbag::View(bag);
            deque<Point> lanes;

            for (rosbag::MessageInstance const &msg : messages)
            {
                if (msg.getTopic() == t.second)
                {
                    auto pc = msg.instantiate<PointCloud2>();
                    std::vector<uint8_t> data = pc->data;
                    for (int i = 0; i < data.size(); i += pc->point_step)
                    {
                        Point point;
                        memcpy(&point, &data[i], pc->point_step);
                        lanes.emplace_back(point);
                    }
                }
            }
            bag.close();

            ofstream ofs("./out/" + fileName + "." + t.first + ".ply");
            ofs << R"(ply
format ascii 1.0
comment VCGLIB generated
element vertex )";
            ofs << lanes.size() / 3 << endl;
            ofs << R"(property float x
property float y
property float z
element face 0
property list uchar int vertex_indices
end_header
)";
            for (int i = 0; i < lanes.size(); i += 3)
            {
                ofs << lanes[i].x << " " << lanes[i].y << " " << lanes[i].z << endl;
            }

            ofs.close();
        }

    }

    return 0;
}