#include "MainWindow.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QPushButton>

// ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>


#include <iostream>
#include <thread>
#include <condition_variable>

using namespace std;

extern int play(Scenario & scenario);
extern int playStatus;
extern condition_variable playCondVar;
extern mutex playCondVarMtx;

MainWindow::MainWindow(const string & xodrfile, string objfile, QWidget * parent) : QMainWindow(parent)
, m_viewer(nullptr)
, m_treeView(nullptr)
, m_pointProps(nullptr)
, m_pathProps(nullptr)
, m_vehicleProps(nullptr)
, m_scenarioProps(nullptr)
{
    m_viewer = new Viewer(xodrfile, objfile);
    setCentralWidget(m_viewer);

    m_treeView = new TreeView(m_viewer->getScenario());
    QDockWidget *treeDock = new QDockWidget(tr("Paths"), this);
    addDockWidget(Qt::LeftDockWidgetArea, treeDock);
    treeDock->setWidget(m_treeView);

    connect(m_viewer,   SIGNAL(signal_addWaypoint(int)),    m_treeView,     SLOT(slot_addWaypoint(int)));
    connect(m_viewer,   SIGNAL(signal_select(int)),         m_treeView,     SLOT(slot_select(int)));
    connect(m_treeView, SIGNAL(signal_select(int)),         m_viewer,       SLOT(slot_select(int)));

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
            connect(m_pathProps, &WaypathProps::signal_delWaypoint, [this](int id){ m_treeView->slot_delItem(id); m_viewer->update(); });
            connect(m_pathProps, &WaypathProps::signal_updateSmoothPath, [this](){ m_viewer->update(); });
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
            connect(m_vehicleProps, &VehicleProps::signal_addWaypath, [this](int id){ m_treeView->slot_addWaypath(id); update(); });
            connect(m_vehicleProps, &VehicleProps::signal_delWaypath, [this](int id){ m_treeView->slot_delItem(id); update(); });
            connect(m_vehicleProps, &VehicleProps::signal_update, [this](){ update(); }); // color update
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
            connect(m_scenarioProps, &ScenarioProps::signal_addVehicle, [this](int id){ m_treeView->slot_addVehicle(id); update(); });
            connect(m_scenarioProps, &ScenarioProps::signal_delVehicle, [this](int id){ m_treeView->slot_delItem(id); update(); });
            connect(m_scenarioProps, &ScenarioProps::signal_updateOnScenarioLoad, [this](){ m_treeView->loadScenario(); update(); });
        }
        propsDock->setMaximumWidth(200);
        propsDock->setMinimumWidth(200);
    }
    );

    QDockWidget *playDock = new QDockWidget("Animation");
    QHBoxLayout * playLayout = new QHBoxLayout();

    QPushButton *playButton = new QPushButton(playDock);
    playButton->setText("Play");
    m_rosbagImage = new QLabel();

    connect(playButton, &QPushButton::pressed, [&, playButton]()
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

        std::thread t3([&, this]()
        {
            string bagFileName = m_viewer->getScenario().getRosbagFile();
            if (access(bagFileName.c_str(), F_OK) != 0)
            {
                cout << "Rosbag file does not exist, skipping playback of the rosbag file.";
                return;
            }
            cout << "Rosbag file: " << bagFileName << endl;

            if (2 == playStatus) // we wait for the first tick (since scene loading/actors spawning can take time)
            {
                unique_lock<mutex> lk(playCondVarMtx);
                playCondVar.wait(lk);
            }

            if (m_viewer->getScenario().getRosbagOffset() < 0)
            {
                // KB: the negative offset will cause hanging!!!
                // negative offset should be handled in a different way
                cout << "Negative time offset for playback of rosbag files is now not supported!!!" << endl;
            }
            else usleep(m_viewer->getScenario().getRosbagOffset()*1e6);

            rosbag::Bag bag;
            bag.open(bagFileName);
            auto &&messages = rosbag::View(bag);
            m_rosbagImage->resize(800,600);
            m_rosbagImage->show();

            for (rosbag::MessageInstance const &msg : messages)
            {
                if (0 == playStatus)
                    break;
                const std::string msg_topic = msg.getTopic();
                if (msg_topic == m_viewer->getScenario().getRosbagTopic())
                {
                    sensor_msgs::CompressedImage::ConstPtr comp_img_msg = msg.instantiate<sensor_msgs::CompressedImage>();

                    QPixmap backBuffer;
                    if (comp_img_msg->format.find("jpeg") != std::string::npos || comp_img_msg->format.find("jpg") != std::string::npos)
                        backBuffer.loadFromData(&comp_img_msg->data[0], comp_img_msg->data.size(), "JPG");
                    else if (comp_img_msg->format.find("png") != std::string::npos)
                        backBuffer.loadFromData(&comp_img_msg->data[0], comp_img_msg->data.size(), "PNG");
                    else continue;

                    m_rosbagImage->setPixmap(backBuffer.scaled(m_rosbagImage->size(), Qt::KeepAspectRatio));
                    m_rosbagImage->update();
                    unique_lock<mutex> lk(playCondVarMtx);
                    playCondVar.wait(lk);
                }
            }
        });
        t3.detach();
    });

    QPushButton *stopButton = new QPushButton(playDock);
    stopButton->setText("Stop");
    connect(stopButton, &QPushButton::pressed, [&]() { playStatus = 0; playCondVar.notify_all(); });
    //connect(&m_scenario, &Scenario::signal_update, [this](){m_viewer->update();} );

    playLayout->addWidget(playButton);
    playLayout->addWidget(stopButton);

    QWidget * playWidget = new QWidget();
    playWidget->setLayout(playLayout);
    playDock->setWidget(playWidget);
    addDockWidget(Qt::BottomDockWidgetArea, playDock);
}