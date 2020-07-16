#pragma once

#include "quantizer.h"

#include <map>
#include <list>

#include <eigen3/Eigen/Eigen>

#define NSCAN 4 // distance (in cells) for neighbours scan for longest path detection

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

            if (!lane.empty()) paths.push_back(lane);
        }

        // set all the points in the buckets again as isVisited = false
        for (auto && bucket : buckets) bucket.second.front().isVisited = false;

        removeRedundansies();

        // do another iteration to make sure the paths are spanning really the extreme start-end points:
        auto pathscp = move(paths);
        for (auto && l : pathscp)
        {
            Point & bP = l.back();

            BBoxPC lane;
            searchPath(bP, lane);

            if (!lane.empty()) paths.push_back(lane);
        }

        removeRedundansies();

        store(container);
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
        for (int nc = 0; nc >= -NSCAN; nc = nc < 0 ? -nc : -(nc + 1) ) // neighbour col
        {
            for (int nr = 0; nr >= -NSCAN; nr = nr < 0 ? -nr : -(nr + 1) ) // neighbour row
            {
                if ((r + nr) >= H || (r + nr) < 0 || (c + nc) >= W || (c + nc) < 0 ) continue;
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
                paths.push_back(cpl);
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

        multimap<size_t, BBoxPC*> pathsM; // paths by size
        for (auto && l : paths) pathsM.insert(make_pair<size_t, BBoxPC*>(l.size(), &l));

        for (auto it = pathsM.begin(); it != pathsM.end(); ++it)
        {
            auto && lane = *(*it).second;
            int lMax2Pop = 0;
            auto nit = it; nit++;
            for (; nit != pathsM.end(); ++nit) // next iterator after it
            {
                auto && nlane = *(*nit).second;
                int nMax2Pop = 0;
                for (int i = 0; i < lane.size(); ++i)
                {
                    if (lane[i].index == nlane[i].index) ++nMax2Pop;
                }
                if (lMax2Pop < nMax2Pop) lMax2Pop = nMax2Pop;
            }
            for (int i = 0; i < lMax2Pop; i++) lane.pop_front(); // KB: the BBox is not updated!!
        }

        auto cpaths = move(paths);
        for (auto && l : cpaths) if (l.size() >= NSCAN) paths.push_back(l);
    }


protected:

    void store(BBoxPC & container)
    {
        container.clear();
        auto pathscp = move(paths);
        // Fillout the container
        int li = 0;
        for (auto && l : pathscp)
        {
            if (l.size() < NSCAN) continue;

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
            paths.push_back(l);
        }
    }


protected:
    std::deque<BBoxPC> paths;
};