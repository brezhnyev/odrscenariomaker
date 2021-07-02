#include <iostream>
#include <algorithm>
#include <assert.h>
#include <list>

#include "ObjLoader.h"
//#include "points2concave/utils.h"

static constexpr float THRESPOS = 0.1f;
static constexpr float THRESPOS2 = THRESPOS*THRESPOS;
static constexpr float THRESANG = 0.01f;

using namespace std;
using namespace objl;

vector<Vector3> obj2baseline(const Mesh & mesh, const Loader & loader, bool isConcave);


template<typename T> void printVector3(vector<T> & v)
{
    for (auto && e : v)
        cout << e.X << " " << e.Y << " " << e.Z << endl;
}


int main(int argc, char ** argv)
{
    if (argc == 1)
    {
        cout << "Specify the OBJ file as argument: carla2osi /path/to/file.obj" << endl;
        return 0;
    }

    Loader loader;
    loader.LoadFile(argv[1]);

    for (auto && mesh : loader.LoadedMeshes)
    {
        if (mesh.MeshName.find("Building354") != string::npos)
        {
            vector<Vector3> convexBaseline = obj2baseline(mesh, loader, false);
            vector<Vector3> concaveBaseline = obj2baseline(mesh, loader, true);
            printVector3(concaveBaseline);
            if (concaveBaseline.size() < convexBaseline.size())
            {
                cout << mesh.MeshName << "  will use convex hull" << endl;
                printVector3(convexBaseline);
            }
            list<uint64_t> uniqueIds;
            for (auto && v : concaveBaseline) uniqueIds.push_back(v.index);
            uniqueIds.unique();
            if (uniqueIds.size() < concaveBaseline.size())
            {
                cout << mesh.MeshName << "  will use convex hull" << endl;
                printVector3(convexBaseline);
            }
        }
    }

    return 0;
}


vector<Vector3> obj2baseline(const Mesh & mesh, const Loader & loader, bool isConcave)
{
    // Jarvi's march to find the convex hull

    vector<Vector3> baseline;

    Vertex blp, blpc; // base line point, base line point candidate
    blp.Position = Vector3(0,0,__FLT_MAX__);
    float blpAngle = M_PI;
    float minAngleDiff = __FLT_MAX__;

    for (auto && v : mesh.Vertices) if (v.Position.Z < blp.Position.Z && v.Position.Y > 1) blp = v;
    Vector3 startP = blp.Position;
    uint64_t start = blp.Position.index;

    while (true)
    {
        baseline.push_back(blp.Position);
        for (auto && v : mesh.Vertices)
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
            if (angle < 0) angle += 2*M_PI;
            float angleDiff = angle - blpAngle;
            if (angleDiff <= 0) angleDiff += 2*M_PI;
            if (abs(angleDiff < THRESANG)) continue;
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
        if (blpAngle < 0) blpAngle += 2*M_PI;
        blp = blpc;
        minAngleDiff = __FLT_MAX__;

        if (baseline.size() > mesh.Vertices.size())
            break;
    }

    //printVector3(baseline);
    return baseline;
}