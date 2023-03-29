
#include <iostream>
#include <unistd.h>

#include <QtWidgets/QApplication>
#include "MainWindow.h"

using namespace std;

MainWindow * mw;
extern int playStatus;

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