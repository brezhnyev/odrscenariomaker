#include <iostream>

#include "FolderReader.h"
//#include "kdTree.h"
#include "quantizer.h"
#include "pathfinder.h"

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

#define LANEW 0.4

void storePly(string folderName, string laneType, string fileName, BBoxPC &lane)
{
    ofstream ofs("./out/" + folderName + "/" + laneType + "/" + fileName + ".ply");
    ofs << R"(ply
format ascii 1.0
comment VCGLIB generated
element vertex )";
    ofs << lane.size() << endl;
    ofs << R"(property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
element face 0
property list uchar int vertex_indices
end_header
)";
    for (auto && p : lane)
    {
        ofs << p[0] << " " << p[1] << " " << p[2] << " " << (int)p.color[0] << " " << (int)p.color[1] << " " << (int)p.color[2] << endl;
    }
    ofs.close();
}


int main()
{
    FolderReader<> fr("./data");
    Matrix4f M;
    M.setIdentity();
    //static_assert(sizeof(Point) == 24);

    map<string, string> topics = {
        {"cont", "/LanePoints/cont_lane"},
        {"curb", "/LanePoints/curbstone"},
        {"dash", "/LanePoints/dash_lane"},
        {"stop", "/LanePoints/stop_lane"}};

    while (true)
    {
        string fileName = fr.getNext("bag");
        string baseName = fileName.substr(0, fileName.size() - 4);
        baseName = basename(&baseName[0]);

        if (fileName.empty())
            break;

        for (auto && t : topics)
        {
            rosbag::Bag bag;
            bag.open(fileName);
            int count = 0;
            auto && messages = rosbag::View(bag);
            deque<BBoxPC> lanes;

            auto storePC = [&]()
            {
                BBoxPC flatLane;
                for (auto && l : lanes) copy(l.begin(), l.end(), back_inserter(flatLane));
                Quantizer q(flatLane, LANEW);
                PathFinder(flatLane, LANEW);
                storePly(baseName, t.first, to_string(count++), flatLane);
                lanes.clear();
            };

            for (rosbag::MessageInstance const &msg : messages)
            {
                if (msg.getTopic() != t.second)
                    continue;

                BBoxPC lane;

                auto pc = msg.instantiate<PointCloud2>();
                std::vector<uint8_t> data = pc->data;
                for (int i = 0; i < data.size(); i += pc->point_step)
                {
                    Point point(count);
                    memcpy(&point, &data[i], pc->point_step); // the id will not be overriten
                    lane.push_back(point);
                }

                BBoxPC::removeOutliers(lane);

                //storePly(baseName, t.first, to_string(count++), lane);

                if (!lanes.empty() && !lanes.front().bbox.crossing(lane.bbox)) storePC();
                else lanes.push_back(lane);
            }
            // store the rest of the PC:
            storePC();
            bag.close();
        }
    }

    return 0;
}