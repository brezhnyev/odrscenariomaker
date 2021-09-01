#include <iostream>
#include <algorithm>
#include <assert.h>
#include <vector>
#include <list>
#include <thread>
#include <condition_variable>
#include <unistd.h>

#include <boost/next_prior.hpp>

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

template<typename T>
static void printMeshvertexes2D(const std::vector<T> & v)
{
    for (auto && e : v)
        std::cout << e[0] << " " << e[1] << std::endl;
}

template<typename T>
static void printMeshvertexes3D(const std::vector<T> & v)
{
    for (auto && e : v)
        std::cout << e[0]<< " " << e[1] << " " << e[2] << std::endl;
}


int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_config_file.empty() || access(FLAGS_config_file.c_str(), F_OK))
    {
        cout << "check the path to the configuration file!!!" << endl;
        return 0;
    }

    YAML::Node config = YAML::LoadFile(FLAGS_config_file);
    string obj_file = config["obj_file"].as<string>();
    string xodr_file = config["xodr_file"].as<string>();
    uint32_t FPS = config["fps"].as<uint32_t>();
    float scale = config["scale"].as<float>();

    // Load the static parts:
    uint64_t id;
    Osiexporter osiex(config["write_output"].as<string>());
    Loader loader;
    vector<vector<Eigen::Vector3f>> centerlines, boundaries;
    vector<vector<Eigen::Vector2f>> baselines;
    vector<Eigen::Vector2f> baselinesZ; // Z min and Z max to extrude baselines for rendering

    thread t1([&](){

        cout << "Started parsing XODR file ..." << endl;
        // export road
        OpenDRIVEFile odr;
        loadFile(xodr_file, odr);
        osiex.addRoads(*odr.OpenDRIVE1_5, id, centerlines, boundaries);
        cout << "Finished parsing XODR file ..." << endl;
        mutex mtx;

        if (!obj_file.empty() && !access(obj_file.c_str(), F_OK))
        {
            cout << "Started parsing OBJ file ..." << endl;
            if (!loader.LoadFile(obj_file)) return;

            // extend the static names:
            map<string, int> custom_static_types;
            std::vector<string> exclude_names;
            YAML::Node static_types = config["static_types"];
            for (YAML::const_iterator it = static_types.begin(); it != static_types.end(); ++it)
                custom_static_types[it->first.as<std::string>()] = it->second.as<int>();
            exclude_names = config["exclude_names"].as<std::vector<string>>();
            osiex.extendStaticNames(custom_static_types);

            cout << "Started extracting base_poly ..." << endl;
            // export stationary
            #pragma omp parallel for
            //for (auto && mesh : loader.LoadedMeshes)
            for (int i = 0; i < loader.LoadedMeshes.size(); ++i)
            {
                //cout << std::this_thread::get_id() << endl;
                auto && mesh = loader.LoadedMeshes[i];
                string type = osiex.toValidType(mesh.MeshName);
                if (!type.empty() && (find_if(exclude_names.begin(), exclude_names.end(), [&](const string & name){ return mesh.MeshName.find(name) != string::npos; }) == exclude_names.end()))
                {
                    vector<Eigen::Vector2f> convexBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, false);
                    // vector<Eigen::Vector2f> concaveBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, true);
                    // KB: we skip computing the concaveBaseline at the moment, since the improperly designed meshes
                    // (ex. with redundansies) will cause degreated/corrupted base lines
                    // This can be solved by removing the redundancies (extending this algo)
                    // however since the Lime is not capable of processing the baselines at all, we leave the convex only. 
                    vector<Eigen::Vector2f> concaveBaseline = convexBaseline;
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
                    {
                        lock_guard<mutex> lk(mtx);
                        baselinesZ.emplace_back(); baselinesZ.back()[0] = __FLT_MAX__; baselinesZ.back()[1] = -__FLT_MAX__;
                        for (auto && v : mesh.Vertices)
                        {
                            if (v.Position.Y < baselinesZ.back()[0]) baselinesZ.back()[0] = v.Position.Y;
                            if (v.Position.Y > baselinesZ.back()[1]) baselinesZ.back()[1] = v.Position.Y;
                            v3d.push_back(v.Position);
                        }
                        baselinesZ.back() *= scale;
                        osiex.addStaticObject(v3d, concaveBaseline, id, type, scale);
                        baselines.push_back(move(concaveBaseline));
                    }
                }
            }
            cout << "Finished extracting base_poly" << endl;
        }
        else if (!obj_file.empty() && access(obj_file.c_str(), F_OK))
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
    osiex.writeFrame(true);

    // Qt part should come after spawning, otherwise the application suspends
    Viewer * viewer = nullptr;

    QApplication application(argc, argv);

    viewer = new Viewer(move(baselines), move(baselinesZ), move(centerlines), move(boundaries));
    viewer->setWindowTitle("Osi previewer");
    viewer->show();

    t1.join();

    application.exec();

}