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
    MainWindow(QWidget * parent = nullptr);

private:
    Viewer         *m_viewer;
    TreeView       *m_treeView;
    Scenario        m_scenario;
    WaypointProps  *m_pointProps;
    WaypathProps   *m_pathProps;
    VehicleProps   *m_vehicleProps;
    ScenarioProps  *m_scenarioProps;
    IPC             m_IPC;
};