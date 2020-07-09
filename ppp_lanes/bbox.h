#pragma once

#include "point.h"

#include <eigen3/Eigen/Eigen>

#include <deque>

#define MAXVAL 1000000000
struct BBox
{
    void addPoint(float v [3])
    {
        if (v[0] < minp[0]) minp[0] = v[0];
        if (v[1] < minp[1]) minp[1] = v[1];
        if (v[2] < minp[2]) minp[2] = v[2];
        if (v[0] > maxp[0]) maxp[0] = v[0];
        if (v[1] > maxp[1]) maxp[1] = v[1];
        if (v[2] > maxp[2]) maxp[2] = v[2];
    }
    bool crossing(const BBox & other)
    {
        if (minp[0] > other.maxp[0] || other.minp[0] > maxp[0]) return false;
        if (minp[1] > other.maxp[1] || other.minp[1] > maxp[1]) return false;
        if (minp[2] > other.maxp[2] || other.minp[2] > maxp[2]) return false;

        return true;
    }
    void clear()
    {
        minp = Eigen::Vector3f( MAXVAL, MAXVAL, MAXVAL);
        maxp = Eigen::Vector3f(-MAXVAL,-MAXVAL,-MAXVAL);
    }
    Eigen::Vector3f minp = Eigen::Vector3f{ MAXVAL,  MAXVAL,  MAXVAL};
    Eigen::Vector3f maxp = Eigen::Vector3f{-MAXVAL, -MAXVAL, -MAXVAL};
};


struct BBoxPC : public std::deque<Point>
{
    void push_back(Point p)
    {
        emplace_back(p);
        bbox.addPoint(p.v);
    }
    BBox bbox;
};

static void removeOutliers(BBoxPC & pc)
{
    using namespace Eigen;
    Vector3f center(0,0,0);
    for (auto p : pc)
        center += Vector3f(p.v[0], p.v[1], p.v[2]);
    center /= pc.size();
    float R = 0;
    for (auto p : pc)
        R += (Vector3f(p.v[0], p.v[1], p.v[2]) - center).norm();
    R /= pc.size();
    auto newpc = pc; newpc.clear(); newpc.bbox.clear();
    for (auto p : pc)
        if ((Vector3f(p.v[0], p.v[1], p.v[2]) - center).norm() < 5*R)
            newpc.push_back(p);

    pc = newpc;
}