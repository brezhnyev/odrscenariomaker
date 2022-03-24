#include "treeview.h"
#include "treemodel.h"

#include "Viewer.h"
#include "WaypointProps.h"
#include "WaypathProps.h"
#include "ActorProps.h"
#include "ScenarioProps.h"
#include "IPC.h"

#include <QtWidgets/QMainWindow>

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(const std::string & xodrfile, QWidget * parent = nullptr);
    void update()
    {
        m_viewer->update();
    }

private:
    Viewer         *m_viewer;
    TreeView       *m_treeView;
    WaypointProps  *m_pointProps;
    WaypathProps   *m_pathProps;
    VehicleProps   *m_vehicleProps;
    ScenarioProps  *m_scenarioProps;
};