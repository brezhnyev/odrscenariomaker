
#include <iostream>

#include <QtWidgets/QApplication>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QLabel>
#include <QtWidgets/QGroupBox>

#include "Viewer.h"


using namespace std;

int main(int argc, char ** argv)
{
    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " /path/to/file.xodr (or path to folder with xodr files)  xodr_resolution_step(default 0.1m)" << endl;
        return 0;
    }
    QApplication app(argc, argv);

    QWidget * mainWidget = new QWidget();

    QVBoxLayout * vbl = new QVBoxLayout(mainWidget);
    mainWidget->setLayout(vbl);

    Viewer * viewer = new Viewer(mainWidget, argv[1], argc > 2 ? atof(argv[2]) : 0.1);
    vbl->addWidget(viewer);
    vbl->setSpacing(0);
    vbl->setMargin(0);

    QGroupBox * searchWidget = new QGroupBox(mainWidget);
    QHBoxLayout * hbl = new QHBoxLayout(searchWidget);
    QLabel * label = new QLabel(searchWidget);
    label->setText("Search ID: ");
    hbl->addWidget(label);
    QLineEdit * le = new QLineEdit(searchWidget);
    hbl->addWidget(le);
    searchWidget->setLayout(hbl);
    searchWidget->setMaximumHeight(2*label->height());
    hbl->setSpacing(0);
    hbl->setMargin(0);
    vbl->addWidget(searchWidget);
    QObject::connect(le, &QLineEdit::returnPressed, [&](){ viewer->selectID(le->text()); } );

    mainWidget->show();
    app.exec();

    return 0;
}