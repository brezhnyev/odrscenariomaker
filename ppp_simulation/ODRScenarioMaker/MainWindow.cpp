#include "MainWindow.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QPushButton>

#include <iostream>

using namespace std;

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent), m_pointProps(nullptr), m_pathProps(nullptr), m_actorProps(nullptr)
{
    m_viewer = new Viewer(m_scenario);
    setCentralWidget(m_viewer);

    m_treeView = new TreeView(m_scenario);
    QDockWidget *treeDock = new QDockWidget(tr("Paths"), this);
    addDockWidget(Qt::LeftDockWidgetArea, treeDock);
    treeDock->setWidget(m_treeView);

    connect(m_viewer, SIGNAL(signal_addActor(int)), m_treeView, SLOT(slot_addActor(int)));
    connect(m_viewer, SIGNAL(signal_addWaypath(int)), m_treeView, SLOT(slot_addWaypath(int)));
    connect(m_viewer, SIGNAL(signal_addWaypoint(int)), m_treeView, SLOT(slot_addWaypoint(int)));
    // connect(m_viewer, SIGNAL(signal_delActor(int)), m_treeView, SLOT(slot_delItem(int)));
    // connect(m_viewer, SIGNAL(signal_delWaypath(int)), m_treeView, SLOT(slot_delItem(int)));
    // connect(m_viewer, SIGNAL(signal_delWaypoint(int)), m_treeView, SLOT(slot_delItem(int)));
    connect(m_viewer, SIGNAL(signal_select(int)), m_treeView, SLOT(slot_select(int)));
    connect(m_treeView, SIGNAL(signal_select(int)), m_viewer, SLOT(slot_select(int)));

    QDockWidget * propsDock = new QDockWidget(tr("Properties"), this);
    addDockWidget(Qt::RightDockWidgetArea, propsDock);
    connect(m_viewer, &Viewer::signal_select, 
    [&, this, propsDock](int id){
        auto item = m_scenario.getChild(id);

        if (!item) return;

        if (dynamic_cast<Waypoint*>(item))
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
        else if (dynamic_cast<Waypath*>(item))
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
        else if (dynamic_cast<Actor*>(item))
        {
            if (m_actorProps)
            {
                m_actorProps->close();
                delete m_actorProps;
            }
            m_actorProps = new ActorProps(*dynamic_cast<Actor*>(item));
            propsDock->setWidget(m_actorProps);
            connect(m_actorProps, &ActorProps::signal_addWaypath, [this](int id){m_viewer->update(); });
            connect(m_actorProps, &ActorProps::signal_addWaypath, [this](int id){m_treeView->slot_addWaypath(id); });
        }
    }
    );

    QDockWidget *playDock = new QDockWidget("Animation");
    QHBoxLayout * playLayout = new QHBoxLayout();

    QPushButton *playButton = new QPushButton(playDock);
    playButton->setText("Play");
    connect(playButton, SIGNAL(pressed()), m_viewer, SLOT(slot_play()));

    QPushButton *stopButton = new QPushButton(playDock);
    stopButton->setText("Stop");
    connect(stopButton, SIGNAL(pressed()), m_viewer, SLOT(slot_stop()));

    playLayout->addWidget(playButton);
    playLayout->addWidget(stopButton);

    QWidget * playWidget = new QWidget();
    playWidget->setLayout(playLayout);
    playDock->setWidget(playWidget);
    addDockWidget(Qt::BottomDockWidgetArea, playDock);
}