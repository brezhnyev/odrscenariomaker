
#include <iostream>
#include <unistd.h>
#include <thread>
#include <condition_variable>
#include <mutex>

#include <eigen3/Eigen/Eigen>
#include <QtWidgets/QApplication>

#include "MainWindow.h"

using namespace std;
using namespace Eigen;

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);
    MainWindow * mw;

    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " path/to/file.xodr" << " path/to/file.obj (optionally)" << endl;
        return 0;
    }

    if (argc == 2)
        mw = new MainWindow(argv[1]);
    else if (argc == 3)
        mw = new MainWindow(argv[1], argv[2]);

    mw->show();

    app.exec();

    return 0;
}