#include <iostream>
#include <algorithm>
#include <assert.h>
#include <vector>
#include <list>
#include <thread>
#include <condition_variable>
#include <unistd.h>

#include <gflags/gflags.h>
#include <qapplication.h>
#include <yaml-cpp/yaml.h>

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

DEFINE_string(obj_file, "", "Path to OBJ file. If left out, the OBJ file is skipped. Optional.");
DEFINE_string(xodr_file, "", "Path to XODR file. Required.");
DEFINE_double(scale, 1.0, "Scale factor between OBJ and XODR. For standard Carla maps is 0.01.");
DEFINE_string(config_file, "", "Additional settings. Optional.");


static void printMeshvertexes(const std::vector<Vertex> & v)
{
    for (auto && e : v)
        std::cout << e.Position.X << " " << e.Position.Y << " " << e.Position.Z << std::endl;
}


int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

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
        loadFile(FLAGS_xodr_file, odr);
        osiex.addRoads(*odr.OpenDRIVE1_5, id, centerlines, boundaries);
        cout << "Finished parsing XODR file ..." << endl;

        if (!FLAGS_obj_file.empty() && !access(FLAGS_obj_file.c_str(), F_OK))
        {
            cout << "Started parsing OBJ file ..." << endl;
            loader.LoadFile(FLAGS_obj_file);

            // extend the static names:
            map<string, string> custom_static_names;
            if (!FLAGS_config_file.empty() && !access(FLAGS_config_file.c_str(), F_OK))
            {
                YAML::Node config = YAML::LoadFile(FLAGS_config_file);
                YAML::Node static_names = config["static_names"];
                for (YAML::const_iterator it = static_names.begin(); it != static_names.end(); ++it)
                    custom_static_names[it->first.as<std::string>()] = it->second.as<string>();
            }
            osiex.extendStaticNames(custom_static_names);

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
                    for (auto && v: concaveBaseline) v = v*FLAGS_scale;
                    for (auto && v : mesh.Vertices) v3d.push_back(v.Position);
                    {
                        lock_guard<mutex> lk(mtx1);
                        osiex.addStaticObject(v3d, concaveBaseline, id, type, FLAGS_scale);
                        baselines.push_back(move(concaveBaseline));
                    }
                }
            }
            cout << "Finished extracting base_poly" << endl;
        }
        else if (!FLAGS_obj_file.empty() && access(FLAGS_obj_file.c_str(), F_OK))
        {
            cerr << "The OBJ file does not exist. Proceeding without it." << endl;
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