
#include <QtWidgets/QApplication>

#include "MainWindow.h"

MainWindow * mw;

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);

    mw = new MainWindow();

    mw->show();

    return (app.exec());
}