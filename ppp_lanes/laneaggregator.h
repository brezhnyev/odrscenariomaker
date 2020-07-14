#include "pathmerger.h"

class LaneAggregator : public PathMerger
{
public:
    LaneAggregator(BBoxPC & container, float _cS, std::map<int, BBoxPC> & lanesmap) : PathMerger(container, _cS)
    {
        using namespace std;
        using namespace Eigen;
        
        if (lanes.empty())
        {
            lanes = paths;
            direction.setZero();
            bbox = container.bbox;
        }
        else if (direction.isZero())
        {
            direction = container.bbox.center() - bbox.center();
            bbox = container.bbox;
            for (auto && lane : lanes)
            {
                if ((Vector3f(lane.back().v) - Vector3f(lane.front().v)).dot(direction) < 0)
                    reverse(lane.begin(), lane.end());
            }
            process(lanesmap, container);
        }
        else
        {
            direction = container.bbox.center() - bbox.center();
            bbox = container.bbox;
            process(lanesmap, container);
        }
    }

private:

    bool process(std::map<int, BBoxPC> & lanesmap, BBoxPC & container)
    {
        using namespace std;
        using namespace Eigen;

        for (auto &&path : paths)
        {
            if ((Vector3f(path.back().v) - Vector3f(path.front().v)).dot(direction) < 0)
                reverse(path.begin(), path.end());
        }

        auto lanescp = move(lanes);
        auto pathscp = paths;

        for (auto &&lane : lanescp)
        {
            bool isExtending = false;
            for (auto &&path : pathscp)
            {
                if (path.empty()) continue;
                int col = (lane.back()[0] - container.bbox.minp[0]) / cS;
                int row = (lane.back()[1] - container.bbox.minp[1]) / cS;
                for (auto it = path.begin(); it != path.end(); ++it)
                {
                    Point p = (*it);
                    int pcol = (p[0] - container.bbox.minp[0]) / cS;
                    int prow = (p[1] - container.bbox.minp[1]) / cS;
                    if (abs(pcol - col) < 2 && abs(prow - row) < 2)
                    {
                        for (; it != path.end(); ++it) lane.push_back(*it);
                        lanes.push_back(move(lane));
                        path.clear();
                        isExtending = true;
                        break;
                    }
                }
            }
            if (!isExtending)
                lanesmap[laneID++] = move(lane);
        }

        // store:
        container.clear();
        // Fillout the container
        int li = 0;
        for (auto && it : lanesmap)
        {
            auto && l = it.second;

            for (int i = 0; i < l.size(); ++i)
            {
                if (i != 0)
                    memcpy(l[i].color, &colors[li][0], 3*sizeof(unsigned char)); 
                else 
                {
                    unsigned char RGB[3] = {255,255,255}; // This will visually mark the start of the path
                    memcpy(l[i].color, RGB, 3*sizeof(unsigned char));
                }
                container.push_back(l[i]);
            }
            ++li;
        }
    }

private:
    static BBox bbox;
    static Eigen::Vector3f direction;
    static std::deque<BBoxPC> lanes;
    static int laneID;
};