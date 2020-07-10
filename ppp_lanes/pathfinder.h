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
    PathFinder(BBoxPC & container, float _cS) : Quantizer(container, _cS, 1)
    {
        using namespace Eigen;
        using namespace std;

        for (auto && bucket : buckets)
        {
            Point & bP = bucket.second.front(); // we do not check if the front exists, cause it should, since the bucket is here.
            
            if (bP.isVisited)
                continue;

            BBoxPC lane;
            searchPath(bP, lane);

            if (!lane.empty()) lanes.push_back(lane);
        }

        container.clear();
        multimap<size_t, BBoxPC*> lanesM; // lanes by size
        for (auto && l : lanes) lanesM.insert(make_pair<size_t, BBoxPC*>(l.size(), &l));

        for (auto it = lanesM.begin(); it != lanesM.end(); ++it)
        {
            auto && lane = *(*it).second;
            int lMax2Pop = 0;
            auto nit = it; nit++;
            for (; nit != lanesM.end(); ++nit) // next iterator
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

        for (auto it = lanesM.rbegin(); it != lanesM.rend(); ++it)
        {
            auto l = *(*it).second;
            if (l.size() < SCANC) continue;
            unsigned char rgb [] = { (unsigned char)((float)rand()/RAND_MAX*255), (unsigned char)((float)rand()/RAND_MAX*255), (unsigned char)((float)rand()/RAND_MAX*255)};
            if (rgb[0] < 50 && rgb[1] < 50 && rgb[2] < 50)
            {
                int index = rand()%3;
                rgb[index] = 255 - rgb[index];
            }

            for (auto & p : l)
            {
                memcpy(p.color, rgb, 3*sizeof(unsigned char));
                container.push_back(p);
            }
        }
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
        for (int nc = -1; nc >= -SCANC; nc = nc < 0 ? -nc : -(nc + 1) ) // neighbour col
        {
            for (int nr = -1; nr >= -SCANC; nr = nr < 0 ? -nr : -(nr + 1) ) // neighbour row
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

    std::deque<BBoxPC> lanes;
};