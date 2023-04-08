#include "MainWindow.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGroupBox>

#include <iostream>
#include <thread>
#include <condition_variable>
#include <unistd.h>

using namespace std;
using namespace Eigen;

int playStatus;
condition_variable playCondVar;
mutex playCondVarMtx;

extern int FPS;
extern bool realtime_playback;
extern bool is_synchronous;

#ifdef USE_CARLA
extern void play(Scenario & scenario);
#else
void play(Scenario & scenario) {} // dummy play to link successfully
#endif

MainWindow::MainWindow(const string & xodrfile, string objfile, QWidget * parent) : QMainWindow(parent)
{
    m_viewer = new Viewer(m_scenario, xodrfile, objfile);
    setCentralWidget(m_viewer);

    m_treeView = new TreeView(m_scenario.getID());
    QDockWidget *treeDock = new QDockWidget(tr("Paths"), this);
    addDockWidget(Qt::LeftDockWidgetArea, treeDock);
    treeDock->setWidget(m_treeView);

    connect(m_viewer,   &Viewer::signal_addWaypoint, [this](int id)
    {
        m_treeView->slot_addItem(id, "Waypoint", m_scenario.getActiveWaypath()->getID());
        m_scenario.getActiveWaypath()->updateSmoothPath();
        m_scenario.getActiveActor()->updatePose();
        update();
    });
    connect(m_viewer,   &Viewer::signal_select,       [this](int id){ m_treeView->slot_select(id); });
    connect(m_treeView, &TreeView::signal_select,     [this](int id){ m_viewer->slot_select(id); update(); });

    QDockWidget * propsDock = new QDockWidget(tr("Properties"), this);
    addDockWidget(Qt::RightDockWidgetArea, propsDock);
    connect(m_viewer, &Viewer::signal_select, 
    [&, this, propsDock](int id){
        auto item = m_scenario.findSelectable(id);

        if (!item) return;

        if (item->getType() == "Waypoint")
        {
            closeActive();
            m_activeDlg = m_pointProps = new WaypointProps(*dynamic_cast<Waypoint*>(item));
            propsDock->setWidget(m_pointProps);
            m_c.push_back(connect(m_pointProps, &WaypointProps::signal_update, [this](){ m_scenario.getActiveWaypath()->updateSmoothPath(); update(); }));
            m_c.push_back(connect(m_pointProps, &WaypointProps::signal_delete, [this](int id)
            {
                Waypath * activeWaypath = dynamic_cast<Waypath*>(m_scenario.getActiveWaypath());
                deleteItem(id);
                activeWaypath->updateSmoothPath();
                update();
            }));
            m_c.push_back(connect(m_viewer, &Viewer::signal_activeSelectableMovedBy, [this](float dx, float dy, float dz)
            {
                Selectable * s = m_scenario.getSelected();
                if (s && dynamic_cast<Waypoint*>(s))
                {
                    Vector3f pos = dynamic_cast<Waypoint*>(s)->get_pos();
                    m_pointProps->update(pos[0] + dx, pos[1] + dy, pos[2] + dz);
                    m_scenario.getActiveWaypath()->updateSmoothPath();
                    update();
                }
            }));
        }
        else if (item->getType() == "Waypath")
        {
            closeActive();
            m_activeDlg = m_pathProps = new WaypathProps(*dynamic_cast<Waypath*>(item));
            propsDock->setWidget(m_pathProps);
            m_c.push_back(connect(m_pathProps, &WaypathProps::signal_update, [this](){ m_scenario.getActiveActor()->updatePose(); update(); }));
            m_c.push_back(connect(m_pathProps, &WaypathProps::signal_delete, [this](int id)
            { 
                deleteItem(id);
            }));
        }
        else if (item->getType() == "Camera")
        {
            closeActive();
            m_activeDlg = m_camProps = new CameraProps(*dynamic_cast<Camera*>(item));
            propsDock->setWidget(m_camProps);
            m_c.push_back(connect(m_camProps, &CameraProps::signal_update, [this](){ update(); }));
            m_c.push_back(connect(m_camProps, &CameraProps::signal_delete, [this](int id)
            {
                deleteItem(id);
            }));
            m_c.push_back(connect(m_viewer, &Viewer::signal_activeSelectableMovedBy, [this](float dx, float dy, float dz)
            {
                Selectable * s = m_scenario.getSelected();
                if (s && dynamic_cast<Camera*>(s))
                {
                    Camera * cam = dynamic_cast<Camera*>(s);
                    Selectable * parent = cam->getParent();
                    Vector3f v(dx, dy, dz);
                    if (parent && parent->getType() == "Vehicle")
                    {
                        Vehicle * pv = dynamic_cast<Vehicle*>(parent);
                        Vector3f ori = pv->get_ori();
                        Matrix3f m = AngleAxisf(ori[2]*DEG2RAD, Vector3f::UnitZ())*AngleAxisf(ori[1]*DEG2RAD, Vector3f::UnitY())*AngleAxisf(ori[0]*DEG2RAD, Vector3f::UnitX()).toRotationMatrix();
                        v = m.transpose()*v;
                    }
                    Vector3f pos = cam->get_pos();
                    m_camProps->update(pos[0] + v[0], pos[1] + v[1], pos[2] + v[2]);
                    update();
                }
            }));
        }
        else if (item->getType() == "Vehicle")
        {
            closeActive();
            m_activeDlg = m_vehicleProps = new VehicleProps(*dynamic_cast<Vehicle*>(item));
            propsDock->setWidget(m_vehicleProps);
            m_c.push_back(connect(m_vehicleProps, &VehicleProps::signal_delete, [this](int id)
            {
                deleteItem(id);
            }));
            m_c.push_back(connect(m_vehicleProps, &VehicleProps::signal_update, [this](){ update(); })); // color update
            m_c.push_back(connect(m_vehicleProps, &VehicleProps::signal_addWaypath, [this](int id){ m_treeView->slot_addItem(id, "Waypath"); update(); }));
            m_c.push_back(connect(m_vehicleProps, &VehicleProps::signal_addCamera , [this](int id){ m_treeView->slot_addItem(id, "Camera");  update(); }));
        }
        else if (item->getType() == "Walker")
        {
            closeActive();
            m_activeDlg = m_walkerProps = new WalkerProps(*dynamic_cast<Walker*>(item));
            propsDock->setWidget(m_walkerProps);
            m_c.push_back(connect(m_walkerProps, &VehicleProps::signal_delete, [this](int id)
            {
                deleteItem(id);
            }));
            m_c.push_back(connect(m_walkerProps, &VehicleProps::signal_update, [this](){ update(); })); // color update
            m_c.push_back(connect(m_walkerProps, &VehicleProps::signal_addWaypath, [this](int id){ m_treeView->slot_addItem(id, "Waypath"); update(); }));
            m_c.push_back(connect(m_walkerProps, &VehicleProps::signal_addCamera , [this](int id){ m_treeView->slot_addItem(id, "Camera");  update(); }));
        }
        else if (item->getType() == "Scenario")
        {
            closeActive();
            m_activeDlg = m_scenarioProps = new ScenarioProps(*dynamic_cast<Scenario*>(item));
            propsDock->setWidget(m_scenarioProps);
            m_c.push_back(connect(m_scenarioProps, &ScenarioProps::signal_addVehicle, [this](int id){ m_treeView->slot_addItem(id, "Vehicle"); update(); }));
            m_c.push_back(connect(m_scenarioProps, &ScenarioProps::signal_addWalker, [this](int id){ m_treeView->slot_addItem(id, "Walker"); update(); }));
            m_c.push_back(connect(m_scenarioProps, &ScenarioProps::signal_addCamera , [this](int id){ m_treeView->slot_addItem(id, "Camera");  update(); }));
            m_c.push_back(connect(m_scenarioProps, &ScenarioProps::signal_update, [this](){ m_treeView->loadScenario(&m_scenario); update(); }));
            m_c.push_back(connect(m_scenarioProps, &ScenarioProps::signal_clear, [this]()
            {
                m_scenario.clear();
                m_treeView->loadScenario(&m_scenario);
                update();
            }));
        }
        propsDock->setMaximumWidth(200);
        propsDock->setMinimumWidth(200);
    });

    QDockWidget *playDock = new QDockWidget("Animation");
    QHBoxLayout * playLayout = new QHBoxLayout();

    QPushButton *playButton = new QPushButton(playDock);
    playButton->setText("Play");
    QLabel * rosbagImage = new QLabel(); // need to be created in the parent thread, creating in the thread does not update it

    QHBoxLayout * settingsLayout = new QHBoxLayout(this);
    QGroupBox * settings = new QGroupBox();
    QCheckBox * isSync = new QCheckBox("synch mode", settings);
    QCheckBox * isRealtime = new QCheckBox("real time", settings);
    QSpinBox * freq = new QSpinBox(settings);
    isSync->setChecked(true);
    isRealtime->setChecked(true);
    settingsLayout->addWidget(isSync);
    settingsLayout->addWidget(isRealtime);
    settings->setLayout(settingsLayout);
    freq->setRange(1,100);
    freq->setSingleStep(1);
    freq->setValue(30);
    settingsLayout->addWidget(new QLabel("simulation FPS:", settings));
    settingsLayout->addWidget(freq);
    playLayout->addWidget(settings);

    connect(playButton, &QPushButton::clicked, [&, rosbagImage, playButton, isSync, isRealtime, freq]()
    {
        if (0 == playStatus)
        {
            playStatus = 2;
            isSync->setEnabled(false);
            isRealtime->setEnabled(false);
            freq->setEnabled(false);
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
            play(m_scenario);
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
    connect(stopButton, &QPushButton::clicked, [&, isSync, isRealtime, freq]()
    {
        playStatus = 0;
        isSync->setEnabled(true);
        isRealtime->setEnabled(true);
        freq->setEnabled(true);
        playCondVar.notify_all();
        }
    );

    connect(isSync, &QCheckBox::stateChanged, [&](int state){ is_synchronous = state; });
    connect(isRealtime, &QCheckBox::stateChanged, [&](int state){ realtime_playback = state; });
    connect(freq, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [&](int val){ FPS = val; });

    playLayout->addWidget(playButton);
    playLayout->addWidget(stopButton);

    QWidget * playWidget = new QWidget();
    playWidget->setLayout(playLayout);
    playDock->setWidget(playWidget);
    addDockWidget(Qt::BottomDockWidgetArea, playDock);
}