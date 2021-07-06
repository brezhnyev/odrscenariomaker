#pragma once

#include "ObjLoader.h"

#include <eigen3/Eigen/Eigen>

#include <vector>
#include <assert.h>
#include <iostream>


class BasepolyExtractor
{
public:

    static void printBaseline(std::vector<Eigen::Vector3f> & v)
    {
        for (auto && e : v)
            std::cout << e.x() << " " << e.y() << " " << e.z() << std::endl;
    }

    static std::vector<Eigen::Vector3f> Obj2basepoly(const objl::Mesh & mesh, const objl::Loader & loader, bool isConcave)
    {
        using namespace std;
        using namespace objl;
        using namespace Eigen;

        // Jarvi's march to find the convex hull

        vector<Vector3f> baseline;

        Vertex blp, blpc; // base line point, base line point candidate
        blp.Position = Vector3(0, 0, __FLT_MAX__);
        blp.Position.index = 1; // in OBJ the indices start with 1
        float blpAngle = M_PI;
        float minAngleDiff = 2*M_PI;
        float maxDist = 0.0f;

        for (auto &&v : mesh.Vertices)
            if (v.Position.Z < blp.Position.Z)
                blp = v;
        Vector3 startP = blp.Position;
        uint64_t start = blp.Position.index;

        while (true)
        {
            blp.Position.Y = 0.0f; // KB: should be improved! Here should be a real lowerst number.
            baseline.push_back(blp.Position);
            for (auto &&v : mesh.Vertices)
            {
                if (v.Position.index == blp.Position.index)
                    continue;

                Vector2 dir(v.Position.X - blp.Position.X, v.Position.Z - blp.Position.Z);
                if (dir.norm2() < THRESPOS2)
                    continue;

                if (isConcave)
                {
                    // additional adjacency constraint to Jarvis march:
                    auto it = loader.Adjacency.find(blp.Position.index);
                    assert(it != loader.Adjacency.end());
                    if (it->second.find(v.Position.index) == it->second.end())
                        continue;
                }

                float angle = atan2(dir.Y, dir.X);
                if (angle < 0)
                    angle += 2 * M_PI;
                float angleDiff = angle - blpAngle;
                // angleDiff ranges in (0,2PI]:
                if (angleDiff <= 0)
                    angleDiff += 2*M_PI;
                // ignore too small differences
                if (angleDiff < THRESANG)
                    continue;
                // find the minimal rotation to the next candidate or similar rotation with larger distance from blp (except that blpc is the start vertex to close the contour):
                if (angleDiff < minAngleDiff)
                // || (angleDiff - minAngleDiff < THRESANG && dir.norm() - maxDist > THRESPOS && blpc.Position.index != start)*/)
                {
                    blpc = v;
                    minAngleDiff = angleDiff;
                    maxDist = dir.norm();
                }
            }
            if (blpc.Position.index == start)
                break;
            // additionally check the 3d proximity since the adjacency may be not reliable enough
            Vector2 dist(startP.X - blpc.Position.X, startP.Z - blpc.Position.Z);
            if (dist.norm2() < THRESPOS2)
                break;

            Vector2 dir(blp.Position.X - blpc.Position.X, blp.Position.Z - blpc.Position.Z);
            blpAngle = atan2(dir.Y, dir.X);
            if (blpAngle < 0)
                blpAngle += 2*M_PI;
            blp = blpc;
            minAngleDiff = 2*M_PI;
            maxDist = 0.0f;

            if (baseline.size() > mesh.Vertices.size())
                break;
        }

        return baseline;
    }

    static constexpr float THRESPOS = 0.01f;
    static constexpr float THRESPOS2 = THRESPOS*THRESPOS;
    static constexpr float THRESANG = 0.01f;
};