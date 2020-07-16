#include "pathmerger.h"

#define MINLANESZ 16 // minimum 15 points making lane

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
            container.clear();
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

        auto isOverlapping = [&](auto P, auto && it, auto itend) -> bool
        {
            int col = (P[0] - container.bbox.minp[0]) / cS;
            int row = (P[1] - container.bbox.minp[1]) / cS;
            for (; it != itend; ++it)
            {
                Point p = (*it);
                int pcol = (p[0] - container.bbox.minp[0]) / cS;
                int prow = (p[1] - container.bbox.minp[1]) / cS;
                if (abs(pcol - col) < 2 && abs(prow - row) < 2)
                {
                    return true;
                }
            }
            return false;
        };

        auto extendPath = [&](auto P, auto it, auto itend, auto & path, bool pushfront) -> bool
        {
            if (isOverlapping(P, it, itend))
            {
                if (pushfront)
                    for (; it != itend; ++it) path.push_front(*it);
                else
                    for (; it != itend; ++it) path.push_back(*it);

                return true;
            }
            return false;
        };

        for (auto &&lane : lanes)
        {
            bool isOverlap = false;

            for (auto &&path : paths)
            {
                // front extension of path:
                isOverlap = extendPath(path.front(), lane.rbegin(), lane.rend(), path, true);
                // back extension of path:
                isOverlap = extendPath(path.back(), lane.begin(), lane.end(), path, false) || isOverlap;

                // if lane is part of path then the lane will be removed
                if (!isOverlap)
                    isOverlap = isOverlapping(lane.front(), path.rbegin(), path.rend());
                if (!isOverlap)
                    isOverlap = isOverlapping(lane.back(), path.begin(), path.end()) || isOverlap;

                if (isOverlap)
                {
                    lane.clear();
                    break;
                }
            }
            if (!isOverlap)
            {
                if (lane.size() > MINLANESZ) // filter out too short lanes
                    lanesmap[laneID++] = move(lane);
                else
                    lane.clear();
            }
        }

        // get rid of all empty lanes:
        auto lanescp = move(lanes);
        for (auto && lane : lanescp) if (!lane.empty()) lanes.push_back(lane);
        // rest of the non-overlapping paths:
        for (auto && path : paths) lanes.push_front(path);

        getLanes(container, lanesmap);
    }

public:

    static void getLanes(BBoxPC & container, std::map<int, BBoxPC> & lanesmap, bool isFinal = false)
    {
        container.clear();
        if (isFinal)
            for (auto && lane : lanes) if (lane.size() > MINLANESZ) lanesmap[laneID++] = move(lane);

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