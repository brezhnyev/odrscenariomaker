
#include <QtWidgets/QApplication>
#include "Viewer.h"

#include <iostream>

using namespace std;

int main(int argc, char ** argv)
{
    if (argc < 4)
    {
        cout << "Usage: " << argv[0] << " /path/to/file.xodr  radius(20m)  xodr_resolution_step(0.1m)" << endl;
        return 0;
    }
    QApplication app(argc, argv);

    Viewer viewer(argv[1], atof(argv[2]), atof(argv[3]));

    viewer.show();

    app.exec();

    return 0;
}