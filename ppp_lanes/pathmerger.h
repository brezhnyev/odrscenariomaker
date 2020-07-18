#pragma once

#include "pathfinder.h"
#include <algorithm>

#define HSCAN 8 // distance (in meters) for closing holes
#define HNSCAN 8 // number of neighbours to scan for closing holes

class PathMerger : public PathFinder
{
public:

    PathMerger(float _cS, float _holesSZ = HSCAN) : PathFinder(_cS), holesSZ(_holesSZ) {}

    void process(BBoxPC & container)
    {
        PathFinder::process(container);

        using namespace std;
        using namespace Eigen;

        auto pathscp = move(paths);
        deque<Vector3f> pcl1, pcl2;
        int index1, index2; // the indices of the "bridge" points of two paths
        float D = MAXVAL;
        Vector3f bridge(0,0,0);

        auto fillForPCA = [&](auto it1, auto it1end, auto it2, auto it2end) 
        {
            auto d = (Vector3f((*it1).v) - Vector3f((*it2).v)).norm();
            if (d < holesSZ && d < D)
            {
                D = d;
                pcl1.clear(); pcl2.clear();
                index1 = (*it1).index; index2 = (*it2).index;
                // get the bridge first;
                auto it1cp = it1; auto it2cp = it2;
                Vector3f bp1(0,0,0);
                Vector3f bp2(0,0,0);
                for (int i = 0; i < NSCAN; ++i)
                {
                    bp1 += Vector3f((*it1cp).v); ++it1cp;
                    bp2 += Vector3f((*it2cp).v); ++it2cp;
                }
                bp1 /= NSCAN; bp2 /= NSCAN;
                bridge = bp1 - bp2;
                // Fillout pcl1 and pcl2:
                for (int i = 0; i < HNSCAN; ++i)
                {
                    const Point & p1 = (*it1);
                    for (int wc = 0; wc < p1.weight; ++wc) pcl1.push_back(Vector3f(p1.v)); ++it1; if (it1 == it1end) break;
                    const Point & p2 = (*it2);
                    for (int wc = 0; wc < p2.weight; ++wc) pcl2.push_back(Vector3f(p2.v)); ++it2; if (it2 == it2end) break;
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

                fillForPCA(l1.rbegin(), l1.rend(), l2.begin(), l2.end());
                fillForPCA(l1.begin(),  l1.end(),  l2.rbegin(),l2.rend());
                fillForPCA(l1.begin(),  l1.end(),  l2.begin(), l2.end());
                fillForPCA(l1.rbegin(), l1.rend(), l2.rbegin(),l2.rend());

                if (bridge.isZero()) continue;

                auto pca1 = getPCEigenvalues<decltype(pcl1), Vector3f, Matrix3f>(pcl1, true);
                auto pca2 = getPCEigenvalues<decltype(pcl2), Vector3f, Matrix3f>(pcl2, true);
                bridge.normalize();

                // now check if the pca1 and pca2 main components and the bridge between pclounds are aligned
                Vector3f v1 = pca1.second.block(0,2,3,1);
                Vector3f v2 = pca2.second.block(0,2,3,1);
                if (abs(v1.dot(v2))     < 0.98f) continue;
                if (abs(bridge.dot(v1)) < 0.98f) continue;
                if (abs(bridge.dot(v2)) < 0.98f) continue;

                // the actual merging:
                if (l1.back() .index != index1) reverse(l1.begin(), l1.end());
                if (l2.front().index != index2) reverse(l2.begin(), l2.end());
                for (auto && p : l2) l1.push_back(p);
                l2.clear();
            }
        }
        paths = move(pathscp);
        // sort with greater predicate
        sort(paths.begin(), paths.end(), [](const BBoxPC & pc1, const BBoxPC & pc2){ return pc1.size() > pc2.size(); });
        getLanes(container);
    }

    const int holesSZ;

};