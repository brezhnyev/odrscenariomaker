#pragma once

#include "quantizer.h"

#include <map>
#include <list>
#include <map>

#include <eigen3/Eigen/Eigen>

#define SCANC 4

class PathFinder : public Quantizer
{
public:
    PathFinder(BBoxPC & container, float _cS) : Quantizer(container, _cS)
    {
        using namespace Eigen;
        using namespace std;

        // figure out the longest paths from the unvisited point:
        for (auto && bucket : buckets)
        {
            Point & bP = bucket.second.front(); // we do not check if the front exists, cause it should, since the bucket is here.
            
            if (bP.isVisited)
                continue;

            BBoxPC lane;
            searchPath(bP, lane);

            if (!lane.empty()) lanes.push_back(lane);
        }

        // set all the points in the buckets again as isVisited = false
        for (auto && bucket : buckets) bucket.second.front().isVisited = false;

        // do another iteration to make sure the lanes are spanning really the extreme start-end points:
        auto lanescp = move(lanes);
        for (auto && l : lanescp)
        {
            Point & bP = l.back();

            BBoxPC lane;
            searchPath(bP, lane);

            if (!lane.empty()) lanes.push_back(lane);
        }

        removeRedundansies();
        //closeHoles();

        container.clear();

        // Fillout the container
        // the labda is used due to some unclear gdb behaviour in case the block is just inlined as usual code
        auto store = [&]()
        {
            for (auto && l : lanes)
            {
                if (l.size() < SCANC) continue;
                unsigned char rgb [3] = { (unsigned char)((float)rand()/RAND_MAX*255), (unsigned char)((float)rand()/RAND_MAX*255), (unsigned char)((float)rand()/RAND_MAX*255)};

                for (int i = 0; i < l.size(); ++i)
                {
                    if (i != 0)
                        memcpy(l[i].color, rgb, 3*sizeof(unsigned char)); 
                    else {
                        unsigned char RGB[3] = {255,255,255}; // This will visually mark the start of the path
                        memcpy(l[i].color, RGB, 3*sizeof(unsigned char));
                    }
                    container.push_back(l[i]);
                }
            }
        };

        store();
    }


private:

    void searchPath(Point & bP, BBoxPC & lane)
    {
        using namespace Eigen;
        using namespace std;

        buckets[bP.index].front().isVisited = true;
        lane.push_back(bP);

        BBoxPC local;
        int index = bP.index;
        int r = index / W;
        int c = index - r * W;
        for (int nc = 0; nc >= -SCANC; nc = nc < 0 ? -nc : -(nc + 1) ) // neighbour col
        {
            for (int nr = 0; nr >= -SCANC; nr = nr < 0 ? -nr : -(nr + 1) ) // neighbour row
            {
                int i = (r + nr) * W + (c + nc);
                auto it = buckets.find(i);
                if (it == buckets.end())
                    continue;
                if (it->second.front().isVisited)
                    continue;
                local.push_back(it->second.front());
            }
        }

        if (local.empty()) return;

        bool firstChild = true;
        BBoxPC snapshot;
        if (local.size() > 1) snapshot = lane; // do copy only if needed (optimization)

        for (auto && p : local)
        {
            if (firstChild)
            {
                searchPath(p, lane);
                firstChild = false;
            }
            else
            {
                BBoxPC cpl = snapshot;
                searchPath(p, cpl);
                lanes.push_back(cpl);
            }
        }
    }

    // remove the commong points, ex.:

    // the two paths having the commong points:
    // ........................    path 1
    // ..............
    //                .
    //                  . path 2

    // will be reduced to:

    // ........................    path 1
    //                .
    //                  . path 2 (only two points left)
    void removeRedundansies()
    {
        using namespace std;

        multimap<size_t, BBoxPC*> lanesM; // lanes by size
        for (auto && l : lanes) lanesM.insert(make_pair<size_t, BBoxPC*>(l.size(), &l));

        for (auto it = lanesM.begin(); it != lanesM.end(); ++it)
        {
            auto && lane = *(*it).second;
            int lMax2Pop = 0;
            auto nit = it; nit++;
            for (; nit != lanesM.end(); ++nit) // next iterator after it
            {
                auto && nlane = *(*nit).second;
                int nMax2Pop = 0;
                for (int i = 0; i < lane.size(); ++i)
                {
                    if (lane[i].index == nlane[i].index) ++nMax2Pop;
                }
                if (lMax2Pop < nMax2Pop) lMax2Pop = nMax2Pop;
            }
            for (int i = 0; i < lMax2Pop; i++) lane.pop_front();
        }
    }

    void closeHoles()
    {
        using namespace std;

        auto lanescp = move(lanes);

        // iterate the lanes and figure out which pieces may be welded
        for (auto it = lanescp.begin(); it != lanescp.end(); ++it)
        {
            for (auto nit = it+1; nit != lanescp.end(); ++nit)
            {

            }
        }

    }

protected:
    std::deque<BBoxPC> lanes;
};