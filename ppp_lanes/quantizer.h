#pragma once

#include "point.h"
#include "bbox.h"

#include <map>
#include <list>

#include <eigen3/Eigen/Eigen>

class Quantizer
{
public:
    
    Quantizer(float _cS, int _shakes = 64)
        : W(0), H(0), cS(_cS), shakes(_shakes) {}

    void process(BBoxPC & container)
    {
        using namespace Eigen;
        using namespace std;

        assert(cS);

        srand(0);

        auto rnd = []() -> float { return float(rand())/RAND_MAX; };

        for (int i = 0; i < shakes; ++i)
        {
            buckets.clear();

            Vector3f minp = container.bbox.minp - Vector3f(rnd(), rnd(), rnd())*(!!i);
            Vector3f maxp = container.bbox.maxp + Vector3f(rnd(), rnd(), rnd())*(!!i);

            W = ceil((maxp[0] - minp[0])/cS);
            H = ceil((maxp[1] - minp[1])/cS);

            for (auto && p : container)
            {
                int col = (p[0] - minp[0])/cS;
                int row = (p[1] - minp[1])/cS;

                p.index = row*W + col;
                buckets[p.index].push_back(p);
            }

            container.clear();

            for (auto && bucket : buckets)
            {
                Point sP(0,0,0);
                for (auto && p : bucket.second)
                {
                    sP[0] += p[0];
                    sP[1] += p[1];
                    sP[2] += p[2];
                    sP.weight += p.weight;
                }
                sP[0] /= bucket.second.size();
                sP[1] /= bucket.second.size();
                sP[2] /= bucket.second.size();
                
                container.push_back(sP);
            }
        }
    }

protected:
    std::map<int, std::list<Point>> buckets;
    int W, H;
    float cS; // cell size
    int shakes;
};