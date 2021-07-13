#include <iostream>
#include <algorithm>
#include <assert.h>
#include <vector>
#include <list>

#include <qapplication.h>

#include "Viewer.h"
#include "BasepolyExtractor.h"
#include "Osiexporter.h"

#include "odrparser/odrparser.h"

using namespace std;
using namespace objl;
using namespace Eigen;
using namespace odr;
using namespace odr_1_5;


static void printMeshvertexes(const std::vector<Vertex> & v)
{
    for (auto && e : v)
        std::cout << e.Position.X << " " << e.Position.Y << " " << e.Position.Z << std::endl;
}


int main(int argc, char ** argv)
{
    
    if (argc < 3)
    {
        cout << "Specify the OBJ and XODR file as arguments: carla2osi /path/to/file.obj /path/to/file.xodr" << endl;
        return 0;
    }

    QApplication application(argc, argv);

    Loader loader;
    loader.LoadFile(argv[1]);

    Viewer viewer;
    viewer.setWindowTitle("Base_polygon visualizer");
    viewer.show();

    Osiexporter osiex;

    uint64_t id = 0;

    // simulate 10 frames:
    for (int fr = 0; fr < 1; ++fr)
    {
        osiex.setFrameTime(fr,0);

        // export stationary
        for (auto && mesh : loader.LoadedMeshes)
        {
            if (mesh.MeshName.find("Building") != string::npos)
            {
                vector<Vector2f> convexBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, false);
                vector<Vector2f> concaveBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, true);
                // degenerated geometry case:
                if (convexBaseline.size() < 3)
                {
                    cout << mesh.MeshName << "   convex hull size less than 3! The shape is skipped!" << endl;
                    concaveBaseline = convexBaseline;
                }
                // concave cannot be smaller than convex (something went wrong in computing the concave form):
                if (concaveBaseline.size() < convexBaseline.size())
                {
                    cout << mesh.MeshName << "  concave hull is smaller than convex hull. Convex hull will be used." << endl;
                    concaveBaseline = convexBaseline;
                }
                // if computation of concave hull went into iternal loop and was broken by "convex.size > mesh.size" condition:
                if (concaveBaseline.size() > mesh.Vertices.size())
                {
                    cout << mesh.MeshName << "  concave hull is larger than original point cloud. Convex hull will be used." << endl;
                    concaveBaseline = convexBaseline;
                }
                // store the stationary object into OSI:
                vector<Vector3f> v3d; v3d.reserve(mesh.Vertices.size());
                for (auto && v : mesh.Vertices) v3d.push_back(v.Position);
                osiex.addStaticObject(v3d, concaveBaseline, id, "building");
                // visualize
                viewer.addDataStatic(move(concaveBaseline));
            }
        }

        // export road
        OpenDRIVEFile odr;
        loadFile(argv[2], odr);
        vector<vector<Vector2f>> centerlines, boundaries;
        osiex.addRoads(*odr.OpenDRIVE1_5, id, centerlines, boundaries);
        // visualize
        viewer.updateDataRoads(move(centerlines), move(boundaries));

        osiex.writeFrame();
    }

    application.exec();

    return 0;
}