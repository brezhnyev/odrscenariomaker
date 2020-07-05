#include "ActorProps.h"
#include "Waypath.h"

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
    QLabel * m_idInfo = new QLabel(this);
    m_idInfo->setText(QString(actor.getType().c_str()) + " ID: " + QString::number(actor.getID()));

    QPushButton * addWaypath = new QPushButton(this);
    addWaypath->setText("Add waypath");

    mainLayout->addWidget(m_idInfo);
    mainLayout->addWidget(addWaypath);

    QVBoxLayout * bl1 = new QVBoxLayout();
    QComboBox * delCombo = new QComboBox(this);
    for (auto && child : m_actor.children()) delCombo->addItem(QString::number(child.second->getID()));
    QPushButton * delButton = new QPushButton(this);
    delButton->setText("Delete");
    bl1->addWidget(delButton);
    bl1->addWidget(delCombo);
    QGroupBox * delGroup = new QGroupBox(this);
    delGroup->setTitle("Delete path");
    delGroup->setLayout(bl1);
    mainLayout->addWidget(delGroup);

    setLayout(mainLayout);

    connect(addWaypath, &QPushButton::pressed, [this, delCombo]()
    { 
        int id = m_actor.addChild(new Waypath());
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Waypath: index not found!");
            return;
        }
        emit signal_addWaypath(id);
        delCombo->clear();
        for (auto && child : m_actor.children()) delCombo->addItem(QString::number(child.second->getID()));
    });
    connect(delButton, &QPushButton::pressed, [this, delCombo]()
    { 
        int id = m_actor.delChild(delCombo->currentText().toInt());
        if (id == -1)
        {
            QMessageBox::warning(this, "Error deleting Element", "Failed to delete Waypath: index not found!");
            return;
        }
        emit signal_delWaypath(id);
        delCombo->clear();
        for (auto && child : m_actor.children()) delCombo->addItem(QString::number(child.second->getID()));
    });
}

VehicleProps::VehicleProps(Vehicle & vehicle) : ActorProps(vehicle), m_vehicle(vehicle)
{
    auto mainLayout = layout();
    QComboBox * typeCombo = new QComboBox(this);
    QStringList ls;
    ls << "vehicle.audi.a2" <<
    "vehicle.audi.tt" <<
    "vehicle.carlamotors.carlacola" <<
    "vehicle.dodge_charger.police" <<
    "vehicle.jeep.wrangler_rubicon" <<
    "vehicle.chevrolet.impala" <<
    "vehicle.mini.cooperst" <<
    "vehicle.bmw.isetta" <<
    "vehicle.audi.etron" <<
    "vehicle.mercedes-benz.coupe" <<
    "vehicle.bmw.grandtourer" <<
    "vehicle.toyota.prius" <<
    "vehicle.citroen.c3" <<
    "vehicle.mustang.mustang" <<
    "vehicle.tesla.model3" <<
    "vehicle.diamondback.century" <<
    "vehicle.gazelle.omafiets" <<
    "vehicle.harley-davidson.low_rider" <<
    "vehicle.bh.crossbike" <<
    "vehicle.tesla.cybertruck" <<
    "vehicle.volkswagen.t2" <<
    "vehicle.kawasaki.ninja" <<
    "vehicle.lincoln.mkz2017" <<
    "vehicle.seat.leon" <<
    "vehicle.yamaha.yzf" <<
    "vehicle.nissan.patrol" <<
    "vehicle.nissan.micra";
    typeCombo->addItems(ls);

    // vehicle color:
    m_colorPicker = new QPushButton(this);
    std::string scolor = m_vehicle.colorToString();
    m_colorPicker->setStyleSheet("background-color: rgb("+QString(m_vehicle.colorToString().c_str()) + ")");
    m_colorPicker->setText("Color");
    connect(m_colorPicker, &QPushButton::pressed, [this]()
    {
        QColor color = QColorDialog::getColor();
        QString scolor(QString::number(color.red()) + "," + QString::number(color.green()) + "," + QString::number(color.blue()));
        m_colorPicker->setStyleSheet("background-color: rgb("+scolor+")");
        m_vehicle.m_color = Eigen::Vector3i(color.red(), color.green(), color.blue());
        emit signal_update();
    });
    mainLayout->addWidget(m_colorPicker);

    mainLayout->addWidget(typeCombo);
    // KB: crashes if setName() function is used, should be checked later
    connect(typeCombo, &QComboBox::currentTextChanged, [this, typeCombo](const QString & name){ m_actor.m_name = name.toStdString(); });

    //mainLayout->addStretch(1);
}