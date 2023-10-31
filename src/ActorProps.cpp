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
#include <QtWidgets/QCheckBox>

#include <eigen3/Eigen/Eigen>

#include <sstream>
#include <iostream>

#define MINMAXPOS 1000000

ActorProps::~ActorProps() {}

ActorProps::ActorProps(Actor & actor, std::list<QMetaObject::Connection> & cons) : m_actor(actor)
{
    QVBoxLayout * mainLayout = new QVBoxLayout();
    setLayout(mainLayout);

    QPushButton * addWaypath = new QPushButton("Add waypath", this);
    mainLayout->addWidget(addWaypath);

    QPushButton * addCamera = new QPushButton("Add Camera", this);
    mainLayout->addWidget(addCamera);

    cons.push_back(connect(addWaypath, &QPushButton::clicked, [this]()
    { 
        int id = (new Waypath(&m_actor))->getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Waypath: index not found!");
            return;
        }
        emit signal_addWaypath(id);
    }));
    cons.push_back(connect(addCamera, &QPushButton::clicked, [this]()
    {
        int id = (new Camera(&m_actor))->getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Camera: index not found!");
            return;
        }
        emit signal_addCamera(id);
    }));

    m_colorPicker = new QPushButton(this);
    std::string scolor = m_actor.colorToString();
    m_colorPicker->setStyleSheet("background-color: rgb("+QString(m_actor.colorToString().c_str()) + ")");
    m_colorPicker->setText("Color");
    cons.push_back(connect(m_colorPicker, &QPushButton::clicked, [this]()
    {
        QColor color = QColorDialog::getColor();
        QString scolor(QString::number(color.red()) + "," + QString::number(color.green()) + "," + QString::number(color.blue()));
        m_colorPicker->setStyleSheet("background-color: rgb("+scolor+")");
        m_actor.set_color(Eigen::Vector3i(color.red(), color.green(), color.blue()));
        emit signal_update();
    }));
    mainLayout->addWidget(m_colorPicker);

    m_delButton = new QPushButton();
    m_delButton->setText("Delete");
    m_delButton->setStyleSheet("background-color: red");

    cons.push_back(connect(m_delButton, &QPushButton::clicked, [this]()
    { 
        int id = m_actor.getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error deleting Element", "Failed to delete Vehicle: index not found!");
            return;
        }
        emit signal_delete(id);
        close();
    }));
}

void ActorProps::addTypes(const QStringList & ls, std::list<QMetaObject::Connection> & cons)
{
    QComboBox * typeCombo = new QComboBox(this);
    typeCombo->addItems(ls);
    auto mainLayout = layout();
    mainLayout->addWidget(typeCombo);
    typeCombo->setCurrentIndex(typeCombo->findText(QString(m_actor.get_name().c_str())));
    cons.push_back(connect(typeCombo, &QComboBox::currentTextChanged, [this, typeCombo](const QString & name){ m_actor.set_name(name.toStdString()); }));
}

VehicleProps::VehicleProps(Vehicle & vehicle, std::list<QMetaObject::Connection> & cons) : ActorProps(vehicle, cons), m_vehicle(vehicle)
{
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

    addTypes(ls, cons);

    auto mainLayout = layout();

    QCheckBox * chbox = new QCheckBox("Ego vehicle ", this);
    if (m_vehicle.get_isEgo())
        chbox->setChecked(true);
    mainLayout->addWidget(chbox);

    cons.push_back(connect(chbox, &QCheckBox::toggled, [this](bool toggle)
    {
        if (toggle)
        {
            emit signal_uncheckEgo();
        }
        m_vehicle.set_isEgo(toggle);
    }));

    mainLayout->addWidget(m_delButton);
    ((QVBoxLayout*)mainLayout)->addStretch(1);
}


WalkerProps::WalkerProps(Walker & walker, std::list<QMetaObject::Connection> & cons) : ActorProps(walker, cons), m_walker(walker)
{
    QStringList ls;
    ls << 
    "walker.pedestrian.0001" <<
    "walker.pedestrian.0002" <<
    "walker.pedestrian.0003" <<
    "walker.pedestrian.0004" <<
    "walker.pedestrian.0005" <<
    "walker.pedestrian.0006" <<
    "walker.pedestrian.0007" <<
    "walker.pedestrian.0008" <<
    "walker.pedestrian.0009" <<
    "walker.pedestrian.0010" <<
    "walker.pedestrian.0011" <<
    "walker.pedestrian.0012" <<
    "walker.pedestrian.0013" <<
    "walker.pedestrian.0014" <<
    "walker.pedestrian.0015" <<
    "walker.pedestrian.0016" <<
    "walker.pedestrian.0017" <<
    "walker.pedestrian.0018" <<
    "walker.pedestrian.0019" <<
    "walker.pedestrian.0020" <<
    "walker.pedestrian.0021" <<
    "walker.pedestrian.0022" <<
    "walker.pedestrian.0023" <<
    "walker.pedestrian.0024" <<
    "walker.pedestrian.0025" <<
    "walker.pedestrian.0026" <<
    "walker.pedestrian.0027" <<
    "walker.pedestrian.0028" <<
    "walker.pedestrian.0029" <<
    "walker.pedestrian.0030" <<
    "walker.pedestrian.0031" <<
    "walker.pedestrian.0032" <<
    "walker.pedestrian.0033" <<
    "walker.pedestrian.0034" <<
    "walker.pedestrian.0035" <<
    "walker.pedestrian.0036" <<
    "walker.pedestrian.0037" <<
    "walker.pedestrian.0038" <<
    "walker.pedestrian.0039" <<
    "walker.pedestrian.0040" <<
    "walker.pedestrian.0041";

    addTypes(ls, cons);
    
    auto mainLayout = layout();
    mainLayout->addWidget(m_delButton);
    ((QVBoxLayout*)mainLayout)->addStretch(1);
}