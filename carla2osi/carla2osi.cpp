#include <iostream>
#include <algorithm>
#include <assert.h>
#include <list>

#include "BasepolyExtractor.h"

using namespace std;
using namespace objl;


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
        if (mesh.MeshName.find("Road") != string::npos)
        {
            vector<Vector3> convexBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, false);
            vector<Vector3> concaveBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, true);
            if (convexBaseline.size() < 3)
            {
                cout << mesh.MeshName << "   convex hull size less than 3! The shape is skipped!" << endl;
                continue;
            }
            if (concaveBaseline.size() < convexBaseline.size())
            {
                cout << mesh.MeshName << "  will use convex hull" << endl;
                BasepolyExtractor::printBaseline(convexBaseline);
            }
            list<uint64_t> uniqueIds;
            for (auto && v : concaveBaseline) uniqueIds.push_back(v.index);
            uniqueIds.unique();
            if (uniqueIds.size() < concaveBaseline.size())
            {
                cout << mesh.MeshName << "  will use convex hull" << endl;
                BasepolyExtractor::printBaseline(convexBaseline);
            }
        }
    }

    return 0;
}