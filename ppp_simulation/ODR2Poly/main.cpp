
#include <QtWidgets/QApplication>
#include "Viewer.h"

#include <iostream>

using namespace std;

int main(int argc, char ** argv)
{
    float radius = 20.0f;
    float resolution = 0.1f;
    string roadConfig;
    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " /path/to/file.xodr  radius(20m)  xodr_resolution_step(0.1m) road-config.txt" << endl;
        return 0;
    }
    if (argc > 2)
        radius = atof(argv[2]);
    if (argc > 3)
        resolution = atof(argv[3]);
    if (argc > 4)
        roadConfig = argv[4];

    QApplication app(argc, argv);

    Viewer viewer(argv[1], radius, resolution, roadConfig);

    viewer.show();

    app.exec();

    return 0;
}