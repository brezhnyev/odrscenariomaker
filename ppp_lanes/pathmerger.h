#pragma once

#include "pathfinder.h"

#define HSCAN 8 // distance (in meters) for closing holes

class PathMerger : public PathFinder
{
public:

    PathMerger(BBoxPC & container, float _cS) : PathFinder(container, _cS)
    {
        using namespace std;
        using namespace Eigen;

        auto pathscp = move(paths);
        deque<Vector3f> pcl1, pcl2;
        int index1, index2; // the indices of the "bridge" points of two paths
        float D = MAXVAL;
        Vector3f bridge(0,0,0);

        auto fillForPCA = [&](auto it1, auto it2) 
        {
            auto d = (Vector3f((*it1).v) - Vector3f((*it2).v)).norm();
            if (d < HSCAN && d < D)
            {
                D = d;
                pcl1.clear(); pcl2.clear();
                bridge = Vector3f((*it1).v) - Vector3f((*it2).v);
                index1 = (*it1).index; index2 = (*it2).index;
                for (int i = 0; i < NSCAN; ++i)
                {
                    pcl1.push_back(Vector3f((*it1).v)); ++it1;
                    pcl2.push_back(Vector3f((*it2).v)); ++it2;
                }
            }
        };

        // iterate the paths and figure out which pieces may be welded
        for (auto it = pathscp.begin(); it != pathscp.end(); ++it)
        {
            auto && l1 = *it;
            if (l1.empty()) continue; // this can happen due to processing on previous steps
            for (auto nit = it+1; nit != pathscp.end(); ++nit)
            {
                // first condition: the distance between the end-end points not larger than some constant
                auto && l2 = *nit;
                if (l2.empty()) continue; // this can happen due to processing on previous steps

                D = MAXVAL;
                bridge = Vector3f(0,0,0);

                fillForPCA(l1.rbegin(), l2.begin());
                fillForPCA(l1.begin(),  l2.rbegin());
                fillForPCA(l1.begin(),  l2.begin());
                fillForPCA(l1.rbegin(), l2.rbegin());

                if (bridge.isZero()) continue;

                auto pca1 = getPCEigenvalues<decltype(pcl1), Vector3f, Matrix3f>(pcl1, true);
                auto pca2 = getPCEigenvalues<decltype(pcl2), Vector3f, Matrix3f>(pcl2, true);
                bridge.normalize();

                // now check if the pca1 and pca2 main components and the bridge between pclounds are aligned
                Vector3f v1 = pca1.second.block(0,2,3,1);
                Vector3f v2 = pca2.second.block(0,2,3,1);
                if (abs(v1.dot(v2)) < 0.9f)     continue;
                if (abs(bridge.dot(v1)) < 0.9f) continue;
                if (abs(bridge.dot(v2)) < 0.9f) continue;

                // the actual merging:
                if (l1.back() .index != index1) reverse(l1.begin(), l1.end());
                if (l2.front().index != index2) reverse(l2.begin(), l2.end());
                for (auto && p : l2) l1.push_back(p);
                l2.clear();
            }
        }
        paths = move(pathscp);
        store(container);
    }

};