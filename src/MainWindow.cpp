#include "MainWindow.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QCheckBox>
#include <QtGui/QtEvents>

#include <iostream>
#include <thread>
#include <condition_variable>
#include <unistd.h>

using namespace std;
using namespace Eigen;


MainWindow::MainWindow(const string & xodrfile, string objfile, QWidget * parent) : QMainWindow(parent), m_scenario(new Root())
{
    m_client = new CarlaClient(this);
    m_viewer = new Viewer(m_scenario, m_client->get_camTrf(), xodrfile, objfile);

    setCentralWidget(m_viewer);

    m_treeView = new TreeView(m_scenario.getParent());
    QDockWidget *treeDock = new QDockWidget(tr("Actors"), this);
    addDockWidget(Qt::LeftDockWidgetArea, treeDock);
    m_treeView->setMinimumSize(100, 0.5*size().height());
    m_treeView->resize(100, 0.4*size().height());
    treeDock->setWidget(m_treeView);

    connect(m_viewer,   &Viewer::signal_addWaypoint, [this](int id)
    {
        m_treeView->slot_addItem(id);
        m_scenario.getActiveWaypath()->updateSmoothPath();
        update();
    });
    connect(m_viewer,   &Viewer::signal_select,       [this](int id){ m_treeView->slot_select(id); });
    connect(m_treeView, &TreeView::signal_select,     [this](int id){ m_viewer->slot_select(id); update(); });

    QDockWidget * propsDock = new QDockWidget(tr("Properties"), this);
    addDockWidget(Qt::LeftDockWidgetArea, propsDock);
    connect(m_viewer, &Viewer::signal_select, 
    [&, this, propsDock](int id){
        auto item = m_scenario.findSelectable(id);

        if (!item) return;

        if (item->getType() == "Waypoint")
        {
            closeActive();
            m_activeDlg = m_pointProps = new WaypointProps(*dynamic_cast<Waypoint*>(item), m_c);
            propsDock->setWidget(m_pointProps);
            m_c.push_back(connect(m_pointProps, &WaypointProps::signal_update, [this](){ m_scenario.getActiveWaypath()->updateSmoothPath(); update(); }));
            m_c.push_back(connect(m_pointProps, &WaypointProps::signal_delete, [this, item](int id)
            {
                Waypath * activeWaypath = dynamic_cast<Waypath*>(m_scenario.getActiveWaypath());
                deleteItem(item);
                activeWaypath->updateSmoothPath();
                update();
            }));
            m_c.push_back(connect(m_viewer, &Viewer::signal_moveSelectedTo, [this](float x, float y, float z)
            {
                Selectable * s = m_scenario.getSelected();
                m_pointProps->update(x, y, z);
                m_scenario.getActiveWaypath()->updateSmoothPath();
                update();
            }));
        }
        else if (item->getType() == "Waypath")
        {
            closeActive();
            m_activeDlg = m_pathProps = new WaypathProps(*dynamic_cast<Waypath*>(item), m_c);
            propsDock->setWidget(m_pathProps);
            m_c.push_back(connect(m_pathProps, &WaypathProps::signal_delete, [this, item](int id)
            { 
                deleteItem(item);
            }));
        }
        else if (item->getType() == "Camera")
        {
            closeActive();
            m_activeDlg = m_camProps = new CameraProps(*dynamic_cast<Camera*>(item), m_c);
            propsDock->setWidget(m_camProps);
            m_c.push_back(connect(m_camProps, &CameraProps::signal_update, [this](){ update(); }));
            m_c.push_back(connect(m_camProps, &CameraProps::signal_delete, [this, item](int id)
            {
                deleteItem(item);
            }));
            m_c.push_back(connect(m_viewer, &Viewer::signal_moveSelectedTo, [this](float x, float y, float z)
            {
                Camera * cam = dynamic_cast<Camera*>(m_scenario.getSelected());
                assert(cam);
                Vector4f v(x, y, z, 1.0f);
                Actor * parent = dynamic_cast<Actor*>(cam->getParent());
                if (parent)
                {
                    Matrix4f parentTrf; parentTrf.setIdentity();
                    Vector3f ori = parent->get_ori();
                    parentTrf.block(0,0,3,3) = AngleAxisf(ori[2]*DEG2RAD, Vector3f::UnitZ())*AngleAxisf(ori[1]*DEG2RAD, Vector3f::UnitY())*AngleAxisf(ori[0]*DEG2RAD, Vector3f::UnitX()).toRotationMatrix();
                    parentTrf.block(0,3,3,1) = parent->get_pos();
                    v = parentTrf.inverse()*v;
                }
                m_camProps->update(v.x(), v.y(), v.z());
                update();
            }));
        }
        else if (item->getType() == "Vehicle")
        {
            closeActive();
            m_activeDlg = m_vehicleProps = new VehicleProps(*dynamic_cast<Vehicle*>(item), m_c);
            propsDock->setWidget(m_vehicleProps);
            m_c.push_back(connect(m_vehicleProps, &VehicleProps::signal_delete, [this, item](int id)
            {
                deleteItem(item);
            }));
            m_c.push_back(connect(m_vehicleProps, &VehicleProps::signal_update, [this](){ update(); })); // color update
            m_c.push_back(connect(m_vehicleProps, &VehicleProps::signal_addWaypath, [this](int id){ m_treeView->slot_addItem(id); update(); }));
            m_c.push_back(connect(m_vehicleProps, &VehicleProps::signal_addCamera , [this](int id){ m_treeView->slot_addItem(id);  update(); }));
            m_c.push_back(connect(m_vehicleProps, &VehicleProps::signal_uncheckEgo, [this]()
            {
                for (auto && c : m_scenario.children())
                {
                    Vehicle * v = dynamic_cast<Vehicle*>(c);
                    if (v)
                        v->set_isEgo(false);
                }
            }));
        }
        else if (item->getType() == "Walker")
        {
            closeActive();
            m_activeDlg = m_walkerProps = new WalkerProps(*dynamic_cast<Walker*>(item), m_c);
            propsDock->setWidget(m_walkerProps);
            m_c.push_back(connect(m_walkerProps, &VehicleProps::signal_delete, [this, item](int id)
            {
                deleteItem(item);
            }));
            m_c.push_back(connect(m_walkerProps, &VehicleProps::signal_update, [this](){ update(); })); // color update
            m_c.push_back(connect(m_walkerProps, &VehicleProps::signal_addWaypath, [this](int id){ m_treeView->slot_addItem(id); update(); }));
            m_c.push_back(connect(m_walkerProps, &VehicleProps::signal_addCamera , [this](int id){ m_treeView->slot_addItem(id);  update(); }));
        }
        else if (item->getType() == "Scenario")
        {
            closeActive();
            m_activeDlg = m_scenarioProps = new ScenarioProps(*dynamic_cast<Scenario*>(item), m_c);
            propsDock->setWidget(m_scenarioProps);
            m_c.push_back(connect(m_scenarioProps, &ScenarioProps::signal_addVehicle, [this](int id){ m_treeView->slot_addItem(id); update(); }));
            m_c.push_back(connect(m_scenarioProps, &ScenarioProps::signal_addWalker, [this](int id) { m_treeView->slot_addItem(id); update(); }));
            m_c.push_back(connect(m_scenarioProps, &ScenarioProps::signal_addCamera , [this](int id){ m_treeView->slot_addItem(id);  update(); }));
            m_c.push_back(connect(m_scenarioProps, &ScenarioProps::signal_update, [this](QString scenarioFileName)
            {
                m_treeView->slot_addItem(m_scenario.getID());
                setWindowTitle(QString("ODRSM - ") + scenarioFileName);
                update();
            }));
            m_c.push_back(connect(m_scenarioProps, &ScenarioProps::signal_clear, [this]()
            {
                m_scenario.clear();
                m_treeView->slot_addItem(m_scenario.getID());
                update();
            }));
        }
        propsDock->setMaximumWidth(200);    
    });
    m_treeView->slot_addItem(m_scenario.getID());

    QDockWidget *playDock = new QDockWidget("Animation");
    QVBoxLayout * playLayout = new QVBoxLayout();

    QPushButton *playButton = new QPushButton(playDock);
    playButton->setText("Play");
    QLabel * rosbagImage = new QLabel(); // need to be created in the parent thread, creating in the thread does not update it

    QCheckBox * isSync = new QCheckBox("synch mode", playDock);
    QCheckBox * isRealtime = new QCheckBox("real time", playDock);
    QCheckBox * useCarla = new QCheckBox("use carla", playDock);
    QSpinBox * freq = new QSpinBox(playDock);
    playLayout->addWidget(isSync);
    playLayout->addWidget(isRealtime);
    playLayout->addWidget(freq);
    playLayout->addWidget(useCarla);
    playDock->setMaximumWidth(200);

    connect(playButton, &QPushButton::clicked, [&, rosbagImage, playButton, isSync, isRealtime, freq, useCarla, treeDock, propsDock]()
    {
        if (Client::STOP == m_client->get_playStatus())
        {
            m_client->set_playStatus(Client::PLAY);
            isSync->setEnabled(false);
            isRealtime->setEnabled(false);
            freq->setEnabled(false);
#ifdef USE_CARLA
            useCarla->setEnabled(false);
#endif
            treeDock->setEnabled(false);
            propsDock->setEnabled(false);
        }
        else if (Client::PAUSE == m_client->get_playStatus())
        {
            m_client->set_playStatus(Client::PLAY);
            m_client->get_playCondVar().notify_all();
            return;
        }
        else if (Client::PLAY == m_client->get_playStatus())
        {
            m_client->set_playStatus(Client::PAUSE);
            m_client->get_playCondVar().notify_all();
            return;
        }

        std::thread t1([this, useCarla]()
        {
            if (useCarla->isChecked())
                m_client->play(m_scenario);
            else
                m_client->playDummy(m_scenario);
        });
        t1.detach();

        std::thread t2([&, playButton]()
        {
            if (Client::PLAY == m_client->get_playStatus()) // we wait for the first tick (since scene loading/actors spawning can take time)
            {
                unique_lock<mutex> lk(m_client->get_playCondVarMtx());
                m_client->get_playCondVar().wait(lk);
            }
            time_t ts = time(nullptr);
            while (true)
            {
                usleep(100000);
                playButton->setText("Play/Pause: " + QString(to_string(time(nullptr) - ts).c_str()) + " s");
                if (Client::STOP == m_client->get_playStatus())
                    break;
                if (Client::PAUSE == m_client->get_playStatus())
                {
                    time_t tp = time(nullptr);
                    unique_lock<mutex> lk(m_client->get_playCondVarMtx());
                    m_client->get_playCondVar().wait(lk);
                    ts += time(nullptr) - tp;
                }
            }
        });
        t2.detach();
    });

    QPushButton *stopButton = new QPushButton(playDock);
    stopButton->setText("Stop");
    connect(stopButton, &QPushButton::clicked, [&, isSync, isRealtime, freq, useCarla, treeDock, propsDock]()
    {
        m_client->set_playStatus(Client::STOP);
        isSync->setEnabled(true);
        isRealtime->setEnabled(true);
        freq->setEnabled(true);
#ifdef USE_CARLA
        useCarla->setEnabled(true);
#endif
        m_client->get_playCondVar().notify_all();
        treeDock->setEnabled(true);   
        propsDock->setEnabled(true);
    });

    connect(isSync, &QCheckBox::stateChanged, [&, isRealtime ](int state)
    { 
        m_client->set_isSynchronous(state);
        isRealtime->setChecked(false);
        isRealtime->setEnabled(state);
    });
    connect(isRealtime, &QCheckBox::stateChanged, [&](int state){ m_client->set_realtimePlayback(state); });
    connect(freq, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [this](int val){ m_client->set_FPS(val); });
    connect(useCarla, &QCheckBox::stateChanged, [isSync, isRealtime, this](int state)
    {
        isSync->setEnabled(state);
        isRealtime->setEnabled(state);
    });

    isSync->setChecked(true);
    isRealtime->setChecked(m_client->get_realtimePlayback());
    freq->setRange(1,100);
    freq->setSingleStep(1);
    freq->setValue(30);
#ifdef USE_CARLA
    useCarla->setChecked(true);
#else
    useCarla->setChecked(false);
    useCarla->setEnabled(false);
#endif

    playLayout->addWidget(playButton);
    playLayout->addWidget(stopButton);
    playLayout->addStretch(1);

    QWidget * playWidget = new QWidget();
    playWidget->setLayout(playLayout);
    playDock->setWidget(playWidget);
    addDockWidget(Qt::LeftDockWidgetArea, playDock);
}

void MainWindow::keyPressEvent(QKeyEvent * e)
{
    if (e->key() == Qt::Key_Z && e->modifiers() == Qt::ControlModifier)
    {
        m_scenario.undo();
        m_treeView->slot_addItem(m_scenario.getID());
        update();
    }
    if (e->key() == Qt::Key_Y && e->modifiers() == Qt::ControlModifier)
    {
        m_scenario.redo();
        m_treeView->slot_addItem(m_scenario.getID());
        update();
    }
    if (e->key() == Qt::Key_Delete && m_scenario.getSelected())
    {
        if (m_scenario.getSelected() != &m_scenario)
        {
            Waypath * activeWaypath = dynamic_cast<Waypath*>(m_scenario.getActiveWaypath());
            if (activeWaypath)
            {
                deleteItem(m_scenario.getSelected());
                activeWaypath->updateSmoothPath();
                update();
            }
            else
                deleteItem(m_scenario.getSelected());
        }
    }

    QMainWindow::keyPressEvent(e);
}