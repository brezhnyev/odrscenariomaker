
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

int playStatus;
condition_variable playCondVar;
mutex playCondVarMtx;
Matrix4f camTrf;
MainWindow * mw;
int FPS = 30;
bool realtime_playback = true;
bool is_synchronous = true;

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);

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

    playStatus = 0;

    usleep(1000000);

    return 0;
}