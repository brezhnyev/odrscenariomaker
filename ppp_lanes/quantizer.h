#pragma once

#include "point.h"
#include "bbox.h"

#include <map>
#include <list>

#include <eigen3/Eigen/Eigen>

class Quantizer
{
public:
    
    Quantizer(BBoxPC & container, float _cS, int _shakes = 64) 
        : W(0), H(0), cS(_cS), shakes(_shakes)
    {
        using namespace Eigen;
        assert(cS);

        for (int i = 0; i < shakes; ++i)
        {
            buckets.clear();
            quantized.clear();

            auto minp = container.bbox.minp - (!!i)*Eigen::Vector3f(float(rand())/RAND_MAX, float(rand())/RAND_MAX, float(rand())/RAND_MAX);
            auto maxp = container.bbox.maxp + (!!i)*Eigen::Vector3f(float(rand())/RAND_MAX, float(rand())/RAND_MAX, float(rand())/RAND_MAX);

            W = ceil((maxp[0] - minp[0])/cS);
            H = ceil((maxp[1] - minp[1])/cS);

            for (auto && p : container)
            {
                int col = (p[0] - minp[0])/cS;
                int row = (p[1] - minp[1])/cS;

                p.index = row*W + col;
                buckets[p.index].push_back(p);
            }

            for (auto && bucket : buckets)
            {
                Point sP(0,0,0);
                for (auto && p : bucket.second)
                {
                    sP[0] += p[0];
                    sP[1] += p[1];
                    sP[2] += p[2];
                }
                sP[0] /= bucket.second.size();
                sP[1] /= bucket.second.size();
                sP[2] /= bucket.second.size();
                
                quantized.push_back(sP);
            }
            container = move(quantized);
        }
    }

protected:
    std::map<int, std::list<Point>> buckets;
    int W, H;
    BBoxPC quantized;
    float cS; // cell size
    int shakes;
};