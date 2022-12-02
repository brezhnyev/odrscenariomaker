#include "treeview.h"
#include "treemodel.h"

#include "Viewer.h"
#include "WaypointProps.h"
#include "WaypathProps.h"
#include "CameraProps.h"
#include "ActorProps.h"
#include "ScenarioProps.h"
#include "IPC.h"

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QLabel>


class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(const std::string & xodrfile, std::string obj = "", QWidget * parent = nullptr);
    void update()
    {
        m_treeView->update();
        m_viewer->update();
    }
    void deleteItem(int id)
    {
        m_scenario.deleteSelectable(id);
        m_treeView->slot_delItem(id);
        update();
    }

private:
    Scenario        m_scenario;
    Viewer         *m_viewer{nullptr};
    TreeView       *m_treeView{nullptr};
    WaypointProps  *m_pointProps{nullptr};
    WaypathProps   *m_pathProps{nullptr};
    CameraProps    *m_camProps{nullptr};
    VehicleProps   *m_vehicleProps{nullptr};
    ScenarioProps  *m_scenarioProps{nullptr};
};