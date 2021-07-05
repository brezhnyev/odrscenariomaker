#pragma once

#include "ObjLoader.h"

#include <vector>
#include <assert.h>
#include <iostream>


class BasepolyExtractor
{
public:

    template<typename T> static void printBaseline(std::vector<T> & v)
    {
        for (auto && e : v)
            std::cout << e.X << " " << e.Y << " " << e.Z << std::endl;
    }

    static std::vector<objl::Vector3> Obj2basepoly(const objl::Mesh & mesh, const objl::Loader & loader, bool isConcave)
    {
        using namespace std;
        using namespace objl;

        // Jarvi's march to find the convex hull

        vector<Vector3> baseline;

        Vertex blp, blpc; // base line point, base line point candidate
        blp.Position = Vector3(0, 0, __FLT_MAX__);
        blp.Position.index = 1; // in OBJ the indices start with 1
        float blpAngle = M_PI;
        float minAngleDiff = __FLT_MAX__;

        for (auto &&v : mesh.Vertices)
            if (v.Position.Z < blp.Position.Z && v.Position.Y > 1)
                blp = v;
        Vector3 startP = blp.Position;
        uint64_t start = blp.Position.index;

        while (true)
        {
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
                if (angleDiff <= 0)
                    angleDiff += 2 * M_PI;
                if (abs(angleDiff < THRESANG))
                    continue;
                if (angleDiff < minAngleDiff)
                {
                    blpc = v;
                    minAngleDiff = angleDiff;
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
                blpAngle += 2 * M_PI;
            blp = blpc;
            minAngleDiff = __FLT_MAX__;

            if (baseline.size() > mesh.Vertices.size())
                break;
        }

        //printVector3(baseline);
        return baseline;
    }

    static constexpr float THRESPOS = 0.1f;
    static constexpr float THRESPOS2 = THRESPOS*THRESPOS;
    static constexpr float THRESANG = 0.01f;
};