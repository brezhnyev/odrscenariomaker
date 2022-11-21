#include "MainWindow.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QPushButton>

#include <iostream>
#include <thread>
#include <condition_variable>

using namespace std;

int playStatus;
condition_variable playCondVar;
mutex playCondVarMtx;

#ifdef USE_CARLA
extern void play(Scenario & scenario);
#else
void play(Scenario & scenario) {} // dummy play to link successfully
#endif

MainWindow::MainWindow(const string & xodrfile, string objfile, QWidget * parent) : QMainWindow(parent)
{
    m_viewer = new Viewer(xodrfile, objfile);
    setCentralWidget(m_viewer);

    m_treeView = new TreeView(m_viewer->getScenario());
    QDockWidget *treeDock = new QDockWidget(tr("Paths"), this);
    addDockWidget(Qt::LeftDockWidgetArea, treeDock);
    treeDock->setWidget(m_treeView);

    connect(m_viewer,   &Viewer::signal_addWaypoint,  [this](int id){ m_treeView->slot_addWaypoint(id); });
    connect(m_viewer,   &Viewer::signal_select,       [this](int id){ m_treeView->slot_select(id); });
    connect(m_treeView, &TreeView::signal_select,     [this](int id){ m_viewer->slot_select(id); update(); });

    QDockWidget * propsDock = new QDockWidget(tr("Properties"), this);
    addDockWidget(Qt::RightDockWidgetArea, propsDock);
    connect(m_viewer, &Viewer::signal_select, 
    [&, this, propsDock](int id){
        auto item = m_viewer->getScenario().findSelectable(id);

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
            connect(m_pointProps, &WaypointProps::signal_update, [this](){ update(); });
            connect(m_pointProps, &WaypointProps::signal_delete, [this](int id)
            {
                m_treeView->slot_delItem(id);
                m_viewer->getScenario().getActiveWaypath()->delChild(id);
                update();
            });
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
            connect(m_pathProps, &WaypathProps::signal_update, [this](){ m_viewer->getScenario().getActiveActor()->updatePose(); update(); });
            connect(m_pathProps, &WaypathProps::signal_delete, [this](int id)
            { 
                m_treeView->slot_delItem(id);
                m_viewer->getScenario().getActiveActor()->delChild(id);
                update();
            });
        }
        else if (item->getType() == "Camera")
        {
            if (m_camProps)
            {
                m_camProps->close();
                delete m_camProps;
            }
            m_camProps = new CameraProps(*dynamic_cast<Camera*>(item));
            propsDock->setWidget(m_camProps);
            //connect(m_camProps, &CameraProps::update, [this](){ m_viewer->update(); });
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
            connect(m_vehicleProps, &VehicleProps::signal_delete, [this](int id)
            {
                m_treeView->slot_delItem(id);
                m_viewer->getScenario().delChild(id);
                update();
            });
            connect(m_vehicleProps, &VehicleProps::signal_update, [this](){ update(); }); // color update
            connect(m_vehicleProps, &VehicleProps::signal_addWaypath, [this](int id){ m_treeView->slot_addItem(id, "Waypath"); update(); });
            connect(m_vehicleProps, &VehicleProps::signal_addCamera , [this](int id){ m_treeView->slot_addItem(id, "Camera");  update(); });
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
            connect(m_scenarioProps, &ScenarioProps::signal_addVehicle, [this](int id){ m_treeView->slot_addItem(id, "Vehicle"); update(); });
            connect(m_scenarioProps, &ScenarioProps::signal_update, [this](){ m_treeView->loadScenario(); update(); });
        }
        propsDock->setMaximumWidth(200);
        propsDock->setMinimumWidth(200);
    }
    );

    QDockWidget *playDock = new QDockWidget("Animation");
    QHBoxLayout * playLayout = new QHBoxLayout();

    QPushButton *playButton = new QPushButton(playDock);
    playButton->setText("Play");
    QLabel * rosbagImage = new QLabel(); // need to be created in the parent thread, creating in the thread does not update it

    connect(playButton, &QPushButton::clicked, [&, rosbagImage, playButton]()
    {
        if (0 == playStatus)
        {
            playStatus = 2;
        }
        else if (1 == playStatus)
        {
            playStatus = 2;
            playCondVar.notify_all();
            return;
        }
        else if (2 == playStatus)
        {
            playStatus = 1;
            playCondVar.notify_all();
            return;
        }

        std::thread t1([this]()
        {
            play(m_viewer->getScenario());
        });
        t1.detach();

        std::thread t2([&, playButton]()
        {
            if (2 == playStatus) // we wait for the first tick (since scene loading/actors spawning can take time)
            {
                unique_lock<mutex> lk(playCondVarMtx);
                playCondVar.wait(lk);
            }
            time_t ts = time(nullptr);
            while (true)
            {
                usleep(1000000);
                playButton->setText("Play/Pause: " + QString(to_string(time(nullptr) - ts).c_str()) + " s");
                if (0 == playStatus)
                    break;
                if (1 == playStatus)
                {
                    time_t tp = time(nullptr);
                    unique_lock<mutex> lk(playCondVarMtx);
                    playCondVar.wait(lk);
                    ts += time(nullptr) - tp;
                }
            }
        });
        t2.detach();
    });

    QPushButton *stopButton = new QPushButton(playDock);
    stopButton->setText("Stop");
    connect(stopButton, &QPushButton::clicked, [&]() { playStatus = 0; playCondVar.notify_all(); });

    playLayout->addWidget(playButton);
    playLayout->addWidget(stopButton);

    QWidget * playWidget = new QWidget();
    playWidget->setLayout(playLayout);
    playDock->setWidget(playWidget);
    addDockWidget(Qt::BottomDockWidgetArea, playDock);
}