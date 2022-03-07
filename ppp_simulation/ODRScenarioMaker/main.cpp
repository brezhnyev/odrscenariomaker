
#include <iostream>

#include <QtWidgets/QApplication>
#include "MainWindow.h"

using namespace std;

MainWindow * mw;
extern bool doStop;

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);

    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " path/to/file.xodr" << endl;
        return 0;
    }

    mw = new MainWindow(argv[1]);

    mw->show();

    app.exec();

    doStop = true;

    usleep(1000000);

    return 0;
}