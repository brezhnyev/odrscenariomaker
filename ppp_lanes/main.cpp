#include <iostream>

#include "FolderReader.h"
#include "kdTree.h"
#include "quantizer.h"
#include "bbox.h"

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
                    lane.addPoint(point);
                }

                removeOutliers(lane);
                Quantizer<decltype(lane)> q(lane, 0.5f);

                ofstream ofs("./out/" + baseName + "/" + t.first + "/" + to_string(count++) + ".ply");
                ofs << R"(ply
format ascii 1.0
comment VCGLIB generated
element vertex )";
                ofs << lane.size() << endl;
                ofs << R"(property float x
property float y
property float z
element face 0
property list uchar int vertex_indices
end_header
)";

                // 1) printing as is
                // for (int i = 0; i < lane.size(); ++i)
                // {
                //     ofs << lane[i][0] << " " << lane[i][1] << " " << lane[i][2] << endl;
                // }

                // 2) printing kdTree
                // KdTree tree;
                // for (auto && l : lane) tree.addNode(new KdNode(l));
                // ofs << tree;

                // 3) printing quantized
                for (int i = 0; i < lane.size(); ++i)
                {
                    ofs << lane[i][0] << " " << lane[i][1] << " " << lane[i][2] << endl;
                }

                lanes.push_back(lane);
                while (lanes.size() > 1)
                {
                    if (!lanes.front().bbox.crossing(lanes.back().bbox))
                        lanes.pop_front();
                    else break;
                }
                

                lane.clear();

                ofs.close();
            }
            bag.close();
        }
    }

    return 0;
}