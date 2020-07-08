#pragma once

#include "point.h"

#include <map>
#include <list>

#include <eigen3/Eigen/Eigen>

#define MAXVAL 1000000000

template<typename C>
class Quantizer
{

    struct Bucket
    {
        int index;
        std::list<Point> points;
        Point sumP;
    };

public:
    typedef typename C::value_type T;
    Quantizer(C container, float _cS) 
        : minp(Eigen::Vector3f( MAXVAL,  MAXVAL,  MAXVAL))
        , maxp(Eigen::Vector3f(-MAXVAL, -MAXVAL, -MAXVAL))
        , W(0), H(0), cS(_cS)
    {
        using namespace Eigen;
        assert(cS);

        for (auto && p : container)
        {
            if (p[0] < minp[0]) minp[0] = p[0];
            if (p[1] < minp[1]) minp[1] = p[1];
            if (p[2] < minp[2]) minp[2] = p[2]; // we will not use z
            if (p[0] > maxp[0]) maxp[0] = p[0];
            if (p[1] > maxp[1]) maxp[1] = p[1];
            if (p[2] > maxp[2]) maxp[2] = p[2]; // we will not use z
        }

        for (int i = 0; i < 10; ++i)
        {
            buckets.clear();
            quantized.clear();

            auto _minp = minp;
            auto _maxp = maxp;
            minp = _minp - Eigen::Vector3f(float(rand())/RAND_MAX, float(rand())/RAND_MAX, float(rand())/RAND_MAX);
            maxp = _maxp + Eigen::Vector3f(float(rand())/RAND_MAX, float(rand())/RAND_MAX, float(rand())/RAND_MAX);

            W = ceil((maxp[0] - minp[0])/cS);
            H = ceil((maxp[1] - minp[1])/cS);

            for (auto && p : container)
            {
                int col = (p[0] - minp[0])/cS;
                int row = (p[1] - minp[1])/cS;

                int index = row*W + col;
                buckets[index].points.push_back(p);
            }

            for (auto && bucket : buckets)
            {
                Point sP(0,0,0);
                for (auto && p : bucket.second.points)
                {
                    sP[0] += p[0];
                    sP[1] += p[1];
                    sP[2] += p[2];
                }
                sP[0] /= bucket.second.points.size();
                sP[1] /= bucket.second.points.size();
                sP[2] /= bucket.second.points.size();
                
                quantized.push_back(sP);
            }
            container = quantized;
        }
    }

    C getQuantized() { return quantized; }

    C dilate(int rep = 1)
    {
        for (int i = 0; i < rep; ++i)
        {
            for (auto && bucket : buckets)
            {

            }
        }
    }

private:
    std::map<int, Bucket> buckets;
    Eigen::Vector3f minp;
    Eigen::Vector3f maxp;
    int W, H;
    C quantized;
    float cS; // cell size
};