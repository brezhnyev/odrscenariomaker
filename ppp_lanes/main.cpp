#include <iostream>

#include "FolderReader.h"
#include "kdTree.h"
#include "quantizer.h"

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
    static_assert(sizeof(Point) == 24);

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

                    //if (lanes.size() > 1000)
                    {
                        Quantizer<decltype(lanes)> q(lanes, 0.5f);
                        auto qc = q.getQuantized();

                        ofstream ofs("./out/" + baseName + "/" + t.first + "/" + to_string(count++) + ".ply");
                        ofs << R"(ply
format ascii 1.0
comment VCGLIB generated
element vertex )";
                        //ofs << lanes.size() << endl;
                        ofs << qc.size() << endl;
                        ofs << R"(property float x
property float y
property float z
element face 0
property list uchar int vertex_indices
end_header
)";

                        // 1) printing as is
                        // for (int i = 0; i < lanes.size(); ++i)
                        // {
                        //     ofs << lanes[i][0] << " " << lanes[i][1] << " " << lanes[i][2] << endl;
                        // }

                        // 2) printing kdTree
                        // KdTree tree;
                        // for (auto && l : lanes) tree.addNode(new KdNode(l));
                        // ofs << tree;

                        // 3) printing quantized
                        for (int i = 0; i < qc.size(); ++i)
                        {
                            ofs << qc[i][0] << " " << qc[i][1] << " " << qc[i][2] << endl;
                        }

                        auto mid = lanes.begin(); advance(mid, lanes.size()*0.5);
                        lanes.clear();

                        ofs.close();
                    }
                }
            }
            bag.close();
        }
    }

    return 0;
}