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

#include <list>


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
    void closeActive()
    {
        if (!m_activeDlg)
            return;
        m_activeDlg->close();
        for (auto && c : m_c)
            disconnect(c);
        m_c.clear();
        delete m_activeDlg;
        m_activeDlg = nullptr;
    }

private:
    Scenario        m_scenario;
    Viewer         *m_viewer{nullptr};
    TreeView       *m_treeView{nullptr};
    WaypointProps  *m_pointProps{nullptr};
    WaypathProps   *m_pathProps{nullptr};
    CameraProps    *m_camProps{nullptr};
    VehicleProps   *m_vehicleProps{nullptr};
    WalkerProps    *m_walkerProps{nullptr};
    ScenarioProps  *m_scenarioProps{nullptr};
    QWidget        *m_activeDlg{nullptr};
    std::list<QMetaObject::Connection> m_c;
};