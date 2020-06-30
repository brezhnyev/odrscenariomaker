#include "MainWindow.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QPushButton>

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent), m_wpProps(nullptr)
{
    m_viewer = new Viewer(m_scenario);
    setCentralWidget(m_viewer);

    m_treeView = new TreeView(m_scenario);
    QDockWidget *treeDock = new QDockWidget(tr("Paths"), this);
    addDockWidget(Qt::LeftDockWidgetArea, treeDock);
    treeDock->setWidget(m_treeView);

    connect(m_viewer, SIGNAL(signal_addWaypath(int)), m_treeView, SLOT(slot_addWaypath(int)));
    connect(m_viewer, SIGNAL(signal_addWaypoint(int)), m_treeView, SLOT(slot_addWaypoint(int)));
    connect(m_viewer, SIGNAL(signal_delWaypath(int)), m_treeView, SLOT(slot_delWaypath(int)));
    connect(m_viewer, SIGNAL(signal_delWaypoint(int)), m_treeView, SLOT(slot_delWaypoint(int)));
    connect(m_viewer, SIGNAL(signal_select(int)), m_treeView, SLOT(slot_select(int)));
    connect(m_treeView, SIGNAL(signal_select(int)), m_viewer, SLOT(slot_select(int)));

    QDockWidget * propsDock = new QDockWidget(tr("Waypoint props"), this);
    addDockWidget(Qt::RightDockWidgetArea, propsDock);
    connect(m_viewer, &Viewer::signal_select, 
    [&, this, propsDock](int id){
        auto wp = m_scenario.getActiveWaypoint();
        if (!wp) return;
        delete m_wpProps;
        m_wpProps = new WaypointProps(*wp);
        propsDock->setWidget(m_wpProps);
        connect(m_wpProps, &WaypointProps::update, [this](){m_viewer->update(); });
    }
    );

    QDockWidget *playDock = new QDockWidget();
    QPushButton *playButton = new QPushButton(playDock);
    playButton->setText("Play");
    playDock->setWidget(playButton);
    addDockWidget(Qt::BottomDockWidgetArea, playDock);
    connect(playButton, SIGNAL(pressed()), m_viewer, SLOT(slot_play()));
}