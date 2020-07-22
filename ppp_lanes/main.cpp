#include <iostream>

#include "FolderReader.h"
//#include "kdTree.h"
#include "quantizer.h"
#include "pathfinder.h"
#include "pathmerger.h"
#include "markingdetector.h"

#include <omp.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <eigen3/Eigen/Eigen>

#include <fstream>
#include <map>

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace Eigen;

int MarkingDetector::markID = 0;

void storePly(string folderName, int index, BBoxPC &lane)
{
    ofstream ofs(folderName + "/" + to_string(index) + ".ply");
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
end_header
)";
    for (auto && p : lane)
    {
        ofs << p[0] << " " << p[1] << " " << p[2] << " " << (int)p.color[0] << " " << (int)p.color[1] << " " << (int)p.color[2] << endl;
    }
    ofs.close();
}

void storeLines(ofstream & ofs, string type, map<int, BBoxPC> & lanesmap)
{
    for (auto && it : lanesmap)
    {
        ofs << it.first << " " << type << " ";
        for (auto & p : it.second) { ofs << p.v[0] << " " << p.v[1] << " " << p.v[2] << " "; }
        ofs << endl;
    }
}


int main(int argc, char ** argv)
{
    if (argc < 2)
    {
        cout << "Please provide folder to the bagfiles: ./lanes_detection ./path/to/folder" << endl;
        return 0;
    }
    FolderReader<> fr(argv[1]);
    //static_assert(sizeof(Point) == 24);

    system("mkdir ply_output");

    map<string, string> topics = {
        {"cont", "/LanePoints/cont_lane"},
        {"curb", "/LanePoints/curbstone"},
        {"dash", "/LanePoints/dash_lane"},
        {"stop", "/LanePoints/stop_lane"}
        };

    map<string, string> outfolders;

        map<string, MarkingDetector*> processors; // use Quantizer, PathFinder or PathMerger for debugging these classes
        processors["cont"] = new MarkingDetectorCont(LANEW);
        processors["curb"] = new MarkingDetectorCurb(LANEW);
        processors["dash"] = new MarkingDetectorDash(LANEW);
        processors["stop"] = new MarkingDetectorStop(0.5f*LANEW);

        assert(topics.size() == processors.size());

    while (true)
    {
        string fileName = fr.getNext("bag");
        string baseName = basename(&fileName.c_str()[0]);

        if (fileName.empty())
            break;

        rosbag::Bag bag;
        bag.open(fileName);
        auto && messages = rosbag::View(bag);
        map<string, deque<BBoxPC>> inlanes; // lanes is not the best name for the container, some other name should be used (paths, markings, points?)

        for (auto && t : topics)
        {
            outfolders[t.first] = "./ply_output/" + baseName  + "/" + t.first;
            system(string("mkdir -p " + outfolders[t.first]).c_str());
        }

        ofstream ofslines("linesout_" + baseName + ".txt");

        auto processPC = [&](string type)
        {
            BBoxPC flatLane;
            map<int, BBoxPC> outlanes;
            // flatten the lanes into one array:
            for (auto && l : inlanes[type]) copy(l.begin(), l.end(), back_inserter(flatLane));

            processors[type]->process(flatLane, outlanes); // if this is commented the raw data will be stored (bunch of PCs from multiple messages)

            if (!flatLane.empty())
            {
                storePly(outfolders[type], outlanes.begin()->first, flatLane);
                storeLines(ofslines, type, outlanes);
            }
        };

        for (rosbag::MessageInstance const &msg : messages)
        {
            string msgTopic = msg.getTopic();
            auto it = find_if(topics.begin(), topics.end(), [&](auto p){ return p.second == msgTopic; });
            if (it == topics.end())
                continue;

            auto t = *it;

            auto && lanes = inlanes[t.first];

            BBoxPC lane;

            auto pc = msg.instantiate<PointCloud2>();
            std::vector<uint8_t> data = pc->data;
            for (int i = 0; i < data.size(); i += pc->point_step)
            {
                Point point;
                memcpy(&point, &data[i], pc->point_step); // the id will not be overriten
                lane.push_back(point);
            }

            // storePly(baseName, t.first, to_string(count++), lane); // could be used for debugging
            // continue;

            BBoxPC::removeOutliers(lane);
            if (lane.size() < 2) continue;

            // sliding window principle. Add into lanes until the bboxes of front and end do not cross:
            if (!lanes.empty() && !lanes.front().bbox.crossing(lane.bbox))
            {
                processPC(t.first);
                size_t s = lanes.size();
                // Remove half of the elements from the from of queue:
                while (!lanes.empty() && lanes.size() > 0.5*s) lanes.pop_front();
            }
            else lanes.push_back(lane);
        }

        for (auto && t : topics)
        {
            // Process the remaining pc (still in lanes):
            processPC(t.first);

            // for Lanes aggregator - there still not aggregated lanes sitting there, get them:
            BBoxPC flatLane; map<int, BBoxPC> outlanes;
            processors[t.first]->getResults(flatLane, outlanes, true);

            if (!flatLane.empty())
            {
                storePly(outfolders[t.first], outlanes.begin()->first, flatLane);
                storeLines(ofslines, t.first, outlanes);
            }
        }

        ofslines.close();

        bag.close();
    }

    return 0;
}