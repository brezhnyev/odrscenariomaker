#include <iostream>
#include <algorithm>
#include <assert.h>
#include <list>

#include "Viewer.h"

#include "BasepolyExtractor.h"

#include <qapplication.h>

using namespace std;
using namespace objl;
using namespace Eigen;


static void printMeshvertexes(const std::vector<Vertex> & v)
{
    for (auto && e : v)
        std::cout << e.Position.X << " " << e.Position.Y << " " << e.Position.Z << std::endl;
}


int main(int argc, char ** argv)
{
    
    if (argc == 1)
    {
        cout << "Specify the OBJ file as argument: carla2osi /path/to/file.obj" << endl;
        return 0;
    }

    QApplication application(argc, argv);

    Loader loader;
    loader.LoadFile(argv[1]);

    Viewer viewer;
    viewer.setWindowTitle("Base_polygon visualizer");
    viewer.show();

    for (auto && mesh : loader.LoadedMeshes)
    {
        if (mesh.MeshName.find("Fence") != string::npos)
        {
            vector<Vector3f> convexBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, false);
            vector<Vector3f> concaveBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, true);
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
            viewer.addData(concaveBaseline);
        }
    }

    application.exec();

    return 0;
}