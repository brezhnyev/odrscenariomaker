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

void storePly(string folderName, string type, BBoxPC & lane)
{
    static int counter = 0;
    BBoxPC *lanep = &lane;
    while(lanep)
    {
        ofstream ofs(folderName + "/" + to_string(counter++) + ".ply");
        ofs << R"(ply
format ascii 1.0
comment Altran generated
element vertex )";
        ofs << lanep->size() << endl;
        ofs << R"(property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
)";
        for (int i = 0; i < lanep->size(); ++i)
        {
            auto && p = (*lanep)[i];
            ofs << p[0] << " " << p[1] << " " << p[2];
            if (i == 0) ofs << " 255 255 255";
            else
            {
                if (type == "cont") ofs << " 255 0 0";
                if (type == "curb") ofs << " 0 255 0";
                if (type == "dash") ofs << " 0 0 255";
                if (type == "stop") ofs << " 255 255 0";
            }
            ofs << endl;
        }
        ofs.close();
        lanep = lanep->getNext();
    }
}

void storeLines(ofstream & ofs, string type, BBoxPC & lane)
{
    static int counter = 0;
    BBoxPC *lanep = &lane;
    while(lanep)
    {
        ofs << counter++ << " " << type << " ";
        for (auto & p : *lanep) { ofs << p.v[0] << " " << p.v[1] << " " << p.v[2] << " "; }
        ofs << endl;
        lanep = lanep->getNext();
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
            // flatten the lanes into one array:
            for (auto && l : inlanes[type]) copy(l.begin(), l.end(), back_inserter(flatLane));

            processors[type]->process(flatLane); // if this is commented the raw data will be stored (bunch of PCs from multiple messages)

            if (!flatLane.empty())
            {
                storePly(outfolders[type], type, flatLane);
                storeLines(ofslines, type, flatLane);
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
                // Remove half of the elements from of queue:
                while (!lanes.empty() && lanes.size() > 0.5*s) lanes.pop_front();
            }
            else lanes.push_back(lane);
        }

        for (auto && t : topics)
        {
            // Process the remaining pc (still in lanes):
            processPC(t.first);

            // for Lanes marks detector: there still not aggregated lanes sitting there, get them:
            BBoxPC flatLane;
            processors[t.first]->finishProcess(flatLane, true);

            if (!flatLane.empty())
            {
                storePly(outfolders[t.first], t.first, flatLane);
                storeLines(ofslines, t.first, flatLane);
            }
        }

        ofslines.close();

        bag.close();
    }

    return 0;
}