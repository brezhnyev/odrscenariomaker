
#include <QtWidgets/QApplication>
#include "Viewer.h"

#include <iostream>

using namespace std;

int main(int argc, char ** argv)
{
    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " /path/to/file.xodr (or path to folder with xodr files)  xodr_resolution_step(default 0.1m)" << endl;
        return 0;
    }
    QApplication app(argc, argv);

    Viewer viewer(argv[1], argc > 2 ? atof(argv[2]) : 0.1);

    viewer.show();

    app.exec();

    return 0;
}