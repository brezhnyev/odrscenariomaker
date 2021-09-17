
#include <QtWidgets/QApplication>

#include "MainWindow.h"

MainWindow * mw;
extern bool doStop;

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);

    mw = new MainWindow();

    mw->show();

    app.exec();

    doStop = true;

    usleep(1000000);

    return 0;
}