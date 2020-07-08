#pragma once

#include "point.h"
#include "bbox.h"

#include <map>
#include <list>

#include <eigen3/Eigen/Eigen>

template<typename C>
class Quantizer
{
public:
    typedef typename C::value_type T;
    Quantizer(C & container, float _cS) 
        : W(0), H(0), cS(_cS)
    {
        using namespace Eigen;
        assert(cS);

        quantized = container;
        for (int i = 0; i < 10; ++i)
        {
            buckets.clear();
            quantized.clear();

            auto minp = container.bbox.minp - Eigen::Vector3f(float(rand())/RAND_MAX, float(rand())/RAND_MAX, float(rand())/RAND_MAX);
            auto maxp = container.bbox.maxp + Eigen::Vector3f(float(rand())/RAND_MAX, float(rand())/RAND_MAX, float(rand())/RAND_MAX);

            W = ceil((maxp[0] - minp[0])/cS);
            H = ceil((maxp[1] - minp[1])/cS);

            for (auto && p : container)
            {
                int col = (p[0] - minp[0])/cS;
                int row = (p[1] - minp[1])/cS;

                int index = row*W + col;
                buckets[index].push_back(p);
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
            container = quantized;
        }
    }

private:
    std::map<int, std::list<Point>> buckets;

private:
    int W, H;
    C quantized;
    float cS; // cell size
};