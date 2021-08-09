#include <iostream>
#include <algorithm>
#include <assert.h>
#include <vector>
#include <list>
#include <thread>
#include <condition_variable>

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


static condition_variable cv;
static bool isStaticParsed = false;


static void printMeshvertexes(const std::vector<Vertex> & v)
{
    for (auto && e : v)
        std::cout << e.Position.X << " " << e.Position.Y << " " << e.Position.Z << std::endl;
}


int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        cout << "Usage: " << argv[0] << " path/to/file.obj" << " path/to/file.xodr" << endl;
        return 0;
    }

    string mapName = "Munich02";
    if (argc > 3)
        mapName = argv[3];

    float scale = 1.0f;
    if (argc > 4)
    {
        stringstream ss(argv[4]); // atof and stof are not reliable
        ss >> scale;
    }
    bool doBuildings = true;
    if (argc > 5)
    {
        stringstream ss; ss << boolalpha << argv[5];
        ss >> doBuildings;
    }

    // Load the static parts:
    uint64_t id;
    Osiexporter osiex;
    Loader loader;
    vector<vector<Eigen::Vector3f>> centerlines, boundaries;
    vector<vector<Eigen::Vector2f>> baselines;

    thread t1([&](){

        cout << "Started parsing XODR file ..." << endl;
        // export road
        OpenDRIVEFile odr;
        loadFile(argv[2], odr);
        osiex.addRoads(*odr.OpenDRIVE1_5, id, centerlines, boundaries);
        cout << "Finished parsing XODR file ..." << endl;
        if (doBuildings)
        {
            cout << "Started parsing OBJ file ..." << endl;
            loader.LoadFile(argv[1]);
            cout << "Started extracting base_poly ..." << endl;

            // export stationary
            mutex mtx1;
    #pragma omp parallel for
            //for (auto && mesh : loader.LoadedMeshes)
            for (int i = 0; i < loader.LoadedMeshes.size(); ++i)
            {
                //cout << std::this_thread::get_id() << endl;
                auto && mesh = loader.LoadedMeshes[i];
                string type = osiex.toValidType(mesh.MeshName);
                if (!type.empty())
                {
                    vector<Eigen::Vector2f> convexBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, false);
                    vector<Eigen::Vector2f> concaveBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, true);
                    // degenerated geometry case:
                    if (convexBaseline.size() < 3)
                    {
                        cerr << mesh.MeshName << "   convex hull size less than 3! The shape is skipped!" << endl;
                        concaveBaseline = convexBaseline;
                    }
                    // concave cannot be smaller than convex (something went wrong in computing the concave form):
                    if (concaveBaseline.size() < convexBaseline.size())
                    {
                        cerr << mesh.MeshName << "  concave hull is smaller than convex hull. Convex hull will be used." << endl;
                        concaveBaseline = convexBaseline;
                    }
                    // if computation of concave hull went into iternal loop and was broken by "convex.size > mesh.size" condition:
                    if (concaveBaseline.size() > mesh.Vertices.size())
                    {
                        cerr << mesh.MeshName << "  concave hull is larger than original point cloud. Convex hull will be used." << endl;
                        concaveBaseline = convexBaseline;
                    }
                    vector<Eigen::Vector3f> v3d; v3d.reserve(mesh.Vertices.size());
                    for (auto && v: concaveBaseline) v = v*scale;
                    for (auto && v : mesh.Vertices) v3d.push_back(v.Position);
                    {
                        lock_guard<mutex> lk(mtx1);
                        osiex.addStaticObject(v3d, concaveBaseline, id, type, scale);
                        baselines.push_back(move(concaveBaseline));
                    }
                }
            }
            cout << "Finished extracting base_poly" << endl;
        }
        isStaticParsed = true;
        cv.notify_all();
    }
    );

    mutex mtx;
    unique_lock<std::mutex> lk(mtx);
    cv.wait(lk, [&]{return isStaticParsed;});

    // store static objects:
    osiex.setFrameTime(0, 0);
    osiex.writeFrame();

    // Qt part should come after spawning, otherwise the application suspends
    Viewer * viewer = nullptr;

    QApplication application(argc, argv);

    viewer = new Viewer(move(baselines), move(centerlines), move(boundaries));
    viewer->setWindowTitle("Osi visualizer");
    viewer->show();

    t1.join();

    application.exec();

}