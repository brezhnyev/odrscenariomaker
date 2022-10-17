#include "ActorProps.h"
#include "Waypath.h"
#include "Camera.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QColorDialog>

#include <eigen3/Eigen/Eigen>

#include <sstream>
#include <iostream>

#define MINMAXPOS 1000000

ActorProps::ActorProps(Actor & actor) : m_actor(actor)
{
    QVBoxLayout * mainLayout = new QVBoxLayout();
    mainLayout->addWidget(new QLabel(QString(actor.getType().c_str()) + " ID: " + QString::number(actor.getID()), this));

    QPushButton * addWaypath = new QPushButton("Add waypath", this);
    mainLayout->addWidget(addWaypath);

    QPushButton * addCamera = new QPushButton("Add Camera", this);
    mainLayout->addWidget(addCamera);

    setLayout(mainLayout);

    connect(addWaypath, &QPushButton::clicked, [this]()
    { 
        int id = m_actor.addChild(new Waypath());
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Waypath: index not found!");
            return;
        }
        emit signal_addWaypath(id);
    });
    connect(addCamera, &QPushButton::clicked, [this]()
    {
        Camera * camera = new Camera();
        int id = m_actor.addChild(camera);
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Waypath: index not found!");
            return;
        }
        emit signal_addCamera(id);
    });
}

VehicleProps::VehicleProps(Vehicle & vehicle) : ActorProps(vehicle), m_vehicle(vehicle)
{
    auto mainLayout = layout();
    QComboBox * typeCombo = new QComboBox(this);
    QStringList ls;
    ls << 
    "vehicle.audi.a2" <<
    "vehicle.audi.etron" <<
    "vehicle.audi.tt" <<
    "vehicle.bh.crossbike" <<
    "vehicle.bmw.grandtourer" <<
    "vehicle.carlamotors.carlacola" <<
    "vehicle.carlamotors.firetruck" <<
    "vehicle.chevrolet.impala" <<
    "vehicle.citroen.c3" <<
    "vehicle.diamondback.century" <<
    "vehicle.dodge.charger_2020" <<
    "vehicle.dodge.charger_police" <<
    "vehicle.dodge.charger_police_2020" <<
    "vehicle.ford.ambulance" <<
    "vehicle.ford.mustang" <<
    "vehicle.gazelle.omafiets" <<
    "vehicle.harley-davidson.low_rider" <<
    "vehicle.jeep.wrangler_rubicon" <<
    "vehicle.kawasaki.ninja" <<
    "vehicle.lincoln.mkz_2017" <<
    "vehicle.lincoln.mkz_2020" <<
    "vehicle.mercedes.coupe" <<
    "vehicle.mercedes.coupe_2020" <<
    "vehicle.mercedes.sprinter" <<
    "vehicle.micro.microlino" <<
    "vehicle.mini.cooper_s" <<
    "vehicle.mini.cooper_s_2021" <<
    "vehicle.nissan.micra" <<
    "vehicle.nissan.patrol" <<
    "vehicle.nissan.patrol_2021" <<
    "vehicle.seat.leon" <<
    "vehicle.tesla.cybertruck" <<
    "vehicle.tesla.model3" <<
    "vehicle.toyota.prius" <<
    "vehicle.vespa.zx125" <<
    "vehicle.volkswagen.t2" <<
    "vehicle.yamaha.yzf";

    typeCombo->addItems(ls);

    // vehicle color:
    m_colorPicker = new QPushButton(this);
    std::string scolor = m_vehicle.colorToString();
    m_colorPicker->setStyleSheet("background-color: rgb("+QString(m_vehicle.colorToString().c_str()) + ")");
    m_colorPicker->setText("Color");
    connect(m_colorPicker, &QPushButton::clicked, [this]()
    {
        QColor color = QColorDialog::getColor();
        QString scolor(QString::number(color.red()) + "," + QString::number(color.green()) + "," + QString::number(color.blue()));
        m_colorPicker->setStyleSheet("background-color: rgb("+scolor+")");
        m_vehicle.m_color = Eigen::Vector3i(color.red(), color.green(), color.blue());
        emit signal_update();
    });
    mainLayout->addWidget(m_colorPicker);

    mainLayout->addWidget(typeCombo);
    connect(typeCombo, &QComboBox::currentTextChanged, [this, typeCombo](const QString & name){ m_actor.setName(name.toStdString()); });

    ((QVBoxLayout*)mainLayout)->addStretch(1);

    QPushButton * delButton = new QPushButton();
    delButton->setText("Delete");
    mainLayout->addWidget(delButton);

    connect(delButton, &QPushButton::clicked, [this]()
    { 
        int id = m_actor.getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error deleting Element", "Failed to delete Waypath: index not found!");
            return;
        }
        emit signal_delete(id);
        close();
    });
}