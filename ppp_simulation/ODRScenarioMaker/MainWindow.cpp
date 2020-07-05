#include "MainWindow.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QPushButton>

#include <iostream>

using namespace std;

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent)
, m_pointProps(nullptr)
, m_pathProps(nullptr)
, m_vehicleProps(nullptr)
, m_scenarioProps(nullptr)
, m_IPC(m_scenario)
{
    m_viewer = new Viewer(m_scenario);
    setCentralWidget(m_viewer);

    m_treeView = new TreeView(m_scenario);
    QDockWidget *treeDock = new QDockWidget(tr("Paths"), this);
    addDockWidget(Qt::LeftDockWidgetArea, treeDock);
    treeDock->setWidget(m_treeView);

    connect(m_viewer,   SIGNAL(signal_addVehicle(int)),     m_treeView,     SLOT(slot_addVehicle(int)));
    connect(m_viewer,   SIGNAL(signal_addWaypath(int)),     m_treeView,     SLOT(slot_addWaypath(int)));
    connect(m_viewer,   SIGNAL(signal_addWaypoint(int)),    m_treeView,     SLOT(slot_addWaypoint(int)));
    connect(m_viewer,   SIGNAL(signal_select(int)),         m_treeView,     SLOT(slot_select(int)));
    connect(m_treeView, SIGNAL(signal_select(int)),         m_viewer,       SLOT(slot_select(int)));

    QDockWidget * propsDock = new QDockWidget(tr("Properties"), this);
    addDockWidget(Qt::RightDockWidgetArea, propsDock);
    connect(m_viewer, &Viewer::signal_select, 
    [&, this, propsDock](int id){
        auto item = m_scenario.findSelectable(id);

        if (!item) return;

        if (item->getType() == "Waypoint")
        {
            if (m_pointProps)
            {
                m_pointProps->close();
                delete m_pointProps;
            }
            m_pointProps = new WaypointProps(*dynamic_cast<Waypoint*>(item));
            propsDock->setWidget(m_pointProps);
            connect(m_pointProps, &WaypointProps::update, [this](){m_viewer->update(); });
        }
        else if (item->getType() == "Waypath")
        {
            if (m_pathProps)
            {
                m_pathProps->close();
                delete m_pathProps;
            }
            m_pathProps = new WaypathProps(*dynamic_cast<Waypath*>(item));
            propsDock->setWidget(m_pathProps);
            connect(m_pathProps, &WaypathProps::signal_delWaypoint, [this](int id){m_viewer->update(); });
            connect(m_pathProps, &WaypathProps::signal_delWaypoint, [this](int id){m_treeView->slot_delItem(id); });
        }
        else if (item->getType() == "Vehicle")
        {
            if (m_vehicleProps)
            {
                m_vehicleProps->close();
                delete m_vehicleProps;
            }
            m_vehicleProps = new VehicleProps(*dynamic_cast<Vehicle*>(item));
            propsDock->setWidget(m_vehicleProps);
            connect(m_vehicleProps, &VehicleProps::signal_addWaypath, [this](int id){m_viewer->update(); });
            connect(m_vehicleProps, &VehicleProps::signal_addWaypath, [this](int id){m_treeView->slot_addWaypath(id); });
            connect(m_vehicleProps, &VehicleProps::signal_delWaypath, [this](int id){m_viewer->update(); });
            connect(m_vehicleProps, &VehicleProps::signal_delWaypath, [this](int id){m_treeView->slot_delItem(id); });
            connect(m_vehicleProps, &VehicleProps::signal_update, [this](){m_viewer->update(); }); // color update
        }
        else if (item->getType() == "Scenario")
        {
            if (m_scenarioProps)
            {
                m_scenarioProps->close();
                delete m_scenarioProps;
            }
            m_scenarioProps = new ScenarioProps(*dynamic_cast<Scenario*>(item));
            propsDock->setWidget(m_scenarioProps);
            connect(m_scenarioProps, &ScenarioProps::signal_addVehicle, [this](int id){m_viewer->update(); });
            connect(m_scenarioProps, &ScenarioProps::signal_addVehicle, [this](int id){m_treeView->slot_addVehicle(id); });
            connect(m_scenarioProps, &ScenarioProps::signal_delVehicle, [this](int id){m_viewer->update(); });
            connect(m_scenarioProps, &ScenarioProps::signal_delVehicle, [this](int id){m_treeView->slot_delItem(id); });
        }
        propsDock->setMaximumWidth(200);
        propsDock->setMinimumWidth(200);
    }
    );

    QDockWidget *playDock = new QDockWidget("Animation");
    QHBoxLayout * playLayout = new QHBoxLayout();

    QPushButton *playButton = new QPushButton(playDock);
    playButton->setText("Play");
    connect(playButton, SIGNAL(pressed()), &m_IPC, SLOT(slot_play()));

    QPushButton *stopButton = new QPushButton(playDock);
    stopButton->setText("Stop");
    connect(stopButton, SIGNAL(pressed()), &m_IPC, SLOT(slot_stop()));

    connect(&m_IPC, &IPC::signal_update, [this](){m_viewer->update();} );

    playLayout->addWidget(playButton);
    playLayout->addWidget(stopButton);

    QWidget * playWidget = new QWidget();
    playWidget->setLayout(playLayout);
    playDock->setWidget(playWidget);
    addDockWidget(Qt::BottomDockWidgetArea, playDock);
}