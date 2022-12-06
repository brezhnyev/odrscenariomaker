
#include <QtWidgets/QApplication>
#include "Viewer.h"

#include <gflags/gflags.h>

DEFINE_string(path_to_xodr, "/path/to/file.xodr", "Full path to XODR file, required");
DEFINE_double(distance, 20.0f, "Visibility distance");
DEFINE_double(resolution, 0.1f, "XODR parsing resolution");
DEFINE_string(configuration, "/path/to/configuration.txt", "Full path to lanes configuration file, optional");
DEFINE_double(frustum_angle, 30, "visibility frustum opening in degrees");
DEFINE_double(frustum_offset, 5, "visibility frustum sidewards offset");

#include <iostream>

using namespace std;

int main(int argc, char ** argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    QApplication app(argc, argv);
    Viewer viewer(FLAGS_path_to_xodr, FLAGS_distance, FLAGS_resolution, FLAGS_configuration, FLAGS_frustum_angle, FLAGS_frustum_offset);
    viewer.show();
    app.exec();

    return 0;
}