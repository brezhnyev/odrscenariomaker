#pragma once

#include "quantizer.h"

#include <map>
#include <list>

#include <eigen3/Eigen/Eigen>

class PathFinder : public Quantizer
{
public:
    PathFinder(BBoxPC & container, float _cS) : Quantizer(container, _cS, 1)
    {
        using namespace Eigen;
        using namespace std;

        for (auto && bucket : buckets)
        {
            vector<Vector3f> local;
            // get its neighbours within 2xcellSize
            int index = bucket.second.front().id; // we do not check if the front exist, it should, since the bucket is here.
            int r = index/W;
            int c = index - r*W;
            for (int nc = -2; nc <= 2; ++nc) // neighbour col
            {
                for (int nr = -2; nr <= 2; ++nr) // neighbour row
                {
                    int i = (r + nr)*W + (c + nc);
                    auto it = buckets.find(i);
                    if (it == buckets.end()) continue;
                    local.push_back(Vector3f(it->second.front().v));
                }
            }
            auto res = getPCEigenvalues<vector<Vector3f>, Vector3f, Matrix3f>(local, true);
            bool stop = true;
        }
    }
};