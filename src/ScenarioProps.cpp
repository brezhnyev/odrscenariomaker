#include "ScenarioProps.h"
#include "Vehicle.h"
#include "Camera.h"
#include "Walker.h"
#include "Waypoint.h"
#include "Waypath.h"
#include "exporters/xosc/components.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTextEdit>
#include <QMessageBox>

#include <fstream>
#include <string>
#include <sstream>
#include <ctime>
#include <iomanip>

using namespace std;

#define MINMAXPOS 1000000


ScenarioProps::ScenarioProps(Scenario & scenario, std::list<QMetaObject::Connection> & cons) : m_scenario(scenario)
{
    QVBoxLayout * mainLayout = new QVBoxLayout();
    QGroupBox * general = new QGroupBox();
    QVBoxLayout * generalLayout = new QVBoxLayout();
    generalLayout->addWidget(new QLabel("Carla Town name:"));
    QLineEdit * townName = new QLineEdit();
    generalLayout->addWidget(townName);
    townName->setText(m_scenario.get_townName().c_str());
    cons.push_back(connect(townName, &QLineEdit::textChanged, [this](const QString & text){ m_scenario.set_townName(text.toStdString()); }));
    QPushButton * loadScenario = new QPushButton("Load Scenario", this);
    generalLayout->addWidget(loadScenario);
    QPushButton * saveScenario = new QPushButton("Save Scenario", this);
    generalLayout->addWidget(saveScenario);
    general->setLayout(generalLayout);
    mainLayout->addWidget(general);
    setLayout(mainLayout);

    QPushButton * addVehicle = new QPushButton("Add vehicle", this);
    mainLayout->addWidget(addVehicle);
    cons.push_back(connect(addVehicle, &QPushButton::clicked, [this]()
    { 
        int id = (new Vehicle(&m_scenario))->getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Vehicle: index not found!");
            return;
        }
        emit signal_addVehicle(id);
    }));

    QPushButton * addWalker = new QPushButton("Add walker", this);
    mainLayout->addWidget(addWalker);
    cons.push_back(connect(addWalker, &QPushButton::clicked, [this]()
    { 
        int id = (new Walker(&m_scenario))->getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Walker: index not found!");
            return;
        }
        emit signal_addWalker(id);
    }));

    QPushButton * addCamera = new QPushButton("Add Camera", this);
    mainLayout->addWidget(addCamera);
    cons.push_back(connect(addCamera, &QPushButton::clicked, [this]()
    {
        int id = (new Camera(&m_scenario))->getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Camera: index not found!");
            return;
        }
        emit signal_addCamera(id);
    }));

    QPushButton * clearScenario = new QPushButton("Clear Scenario", this);
    clearScenario->setStyleSheet("background-color: red");
    mainLayout->addWidget(clearScenario);

    mainLayout->addStretch(1);

    cons.push_back(connect(clearScenario, &QPushButton::clicked, [this]()
    {
        signal_clear();
    }));

    cons.push_back(connect(loadScenario, &QPushButton::clicked, [this, townName]()
    {
        QString name = QFileDialog::getOpenFileName(this, tr("Open Scenario"), "/home", tr("Scenarios (*.yaml)"), nullptr, QFileDialog::DontUseNativeDialog);
        if (name.isEmpty()) return;
        
        ifstream ifs(name.toStdString());
        m_scenario.clear();
        stringstream ssf; ssf << ifs.rdbuf();
        YAML::Node root = YAML::Load(ssf.str());
        m_scenario.from_yaml(root);

        ifs.close();

        m_scenario.set_scenarioFileName(name.toStdString());
        emit signal_update(name);
    }));

    cons.push_back(connect(saveScenario, &QPushButton::clicked, [this]()
    {
        YAML::Node root;
        m_scenario.to_yaml(root);
        string scenarioFileName = m_scenario.get_scenarioFileName();
        string path = scenarioFileName.empty() ? "/home/scenario.yaml" : scenarioFileName;
        QString name = QFileDialog::getSaveFileName(this, tr("Save Scenario"), path.c_str(), tr("Scenarios (*.yaml)"), nullptr,  QFileDialog::DontUseNativeDialog);
        ofstream ofs(name.toStdString());
        ofs << root << endl;
        ofs.close();

        // export trajectories (aorta specific code)
        ofstream ofs_waypath(name.toStdString()+"_waypaths.yaml");
        ofs_waypath << "trajectory:" << endl;
        for (auto && it : m_scenario.children())
        {
            Vehicle * v = dynamic_cast<Vehicle*>(it.second);
            if (v)
            {
                ofs_waypath << "\t" << v->get_name() << ": ";
                for (auto && it : v->children())
                {
                    Waypath * w = dynamic_cast<Waypath*>(it.second);
                    if (w)
                    {
                        ofs_waypath << "[";
                        for (auto && it : w->children())
                        {
                            Waypoint * p = dynamic_cast<Waypoint*>(it.second);
                            auto pos = p->get_pos();
                            ofs_waypath << pos[0] << "," << pos[1] << "," << (pos[2] + 0.5*v->get_bbox()[2]) << ", "; 
                        }
                        ofs_waypath << "]" << "\n";
                    }
                }
            }
        }
        ofs_waypath.close();

        // export initial positions (aorta specific code):
        ofstream ofs_initpos(name.toStdString()+"_init_poses.json");
        ofs_initpos << "{" << endl;
        size_t counter = 1;
        auto writePoses = [&](string infoType, bool isStart = true)
        {
            string indent = "\t";
            if (infoType == "emergency_positions")
            {
                indent += "\t";
                if (isStart)
                    ofs_initpos << indent << "\"start\":{" << endl;
                else
                    ofs_initpos << indent << "\"end\":{" << endl;
            }
            indent += "\t";
            for (auto && it : m_scenario.children())
            {
                Vehicle * v = dynamic_cast<Vehicle*>(it.second);
                if (v && (infoType == "car_positions" || infoType == "emergency_positions"))
                {
                    string name = v->get_name();
                    if (infoType == "car_positions" && name.find("ambulance") != string::npos)
                        continue;
                    if (infoType == "emergency_positions" && name.find("ambulance") == string::npos)
                        continue;

                    for (auto && it : v->children())
                    {
                        Waypath * w = dynamic_cast<Waypath*>(it.second);
                        if (w)
                        {
                            if (w->children().empty())
                                break;
                            ofs_initpos << indent << "\"" << counter << "\"" << ":";
                            ofs_initpos << "{";
                            auto pos = isStart ? w->getStartingPosition() : w->getEndingPosition();
                            ofs_initpos << "\"x\":" << pos[0] << ", ";
                            ofs_initpos << "\"y\":" << pos[1] << ", ";
                            ofs_initpos << "\"z\":" << (pos[2] + 0.5*v->get_bbox()[2]) << ", ";
                            auto dir = isStart ? w->getStartingDirection() : w->getEndingDirection();
                            ofs_initpos << "\"roll\":" << 0 << ", ";
                            ofs_initpos << "\"pitch\":" << asin(dir[2]/dir.norm())*RAD2DEG << ", ";
                            ofs_initpos << "\"yaw\":" << atan2(dir[1], dir[0])*RAD2DEG;
                            ofs_initpos << "}," << endl;
                            ++counter;
                        }
                    }
                }
                Camera * c = dynamic_cast<Camera*>(it.second);
                if (infoType == "edge_camera" && c)
                {
                    ofs_initpos << indent;
                    auto pos = c->get_pos();
                    ofs_initpos << "{\"x\":" << pos[0] << ", ";
                    ofs_initpos << "\"y\":"  << pos[1] << ", ";
                    ofs_initpos << "\"z\":" <<  pos[2] << ", ";
                    auto ori = c->get_ori();
                    ofs_initpos << "\"roll\":" << ori[0] << ", ";
                    ofs_initpos << "\"pitch\":" << ori[1] << ", ";
                    ofs_initpos << "\"yaw\":" << ori[2] << "},";
                    ofs_initpos << endl;
                }
            }
            if (infoType == "emergency_positions")
            {
                indent = indent.substr(0, indent.size()-1);
                ofs_initpos << indent << "}," << endl;
            }
        };
        ofs_initpos << "\t" << "\"car_positions\":{" << endl;
        writePoses("car_positions");
        ofs_initpos << "\t}," << endl;
        counter = 1;
        ofs_initpos << "\t" << "\"emergency_positions\":{" << endl;
        writePoses("emergency_positions", true);
        writePoses("emergency_positions", false);
        ofs_initpos << "\t}," << endl;
        ofs_initpos << "\t" << "\"edge_camera\":\n\t[" << endl;
        writePoses("edge_camera");
        ofs_initpos << "\t]" << endl;
        ofs_initpos << "}";
        ofs_initpos.close();

        // XOSC exporter:
        auto fillplaceholder = [](string & text, const string & placeholder, const auto & value)
        {
            size_t pos = text.find(placeholder);
            if (pos != string::npos)
            {
                stringstream ss;
                ss << fixed << value;
                if (placeholder.find("=\"") != string::npos)
                    text.insert(pos+placeholder.size(), ss.str());
                else
                    text.insert(pos, ss.str());
            }
        };

        ofstream ofs_xosc(name.toStdString()+".xosc");
        string xosc(xosc_template);
        fillplaceholder(xosc, "LogicFile filepath=\"", m_scenario.get_townName());
        time_t unixt = time(nullptr);
        char timeString [256];
        std::strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%S", std::gmtime(&unixt));
        fillplaceholder(xosc, "date=\"", string(timeString));
        fillplaceholder(xosc, "description=\"", "Usecase_"+ string(timeString));
        vector<string> objectNames;
        for (auto && child : m_scenario.children())
        {
            if (child.second->getType() == "Vehicle")
            {
                Vehicle * vehicle = dynamic_cast<Vehicle*>(child.second);
                string xosc_vehicle(xosc_template_vehicle);
                string scenarioObjectName = vehicle->get_isEgo() ? "hero" : "Vehicle_" + to_string(vehicle->getID());
                objectNames.push_back(scenarioObjectName);
                fillplaceholder(xosc_vehicle, "ScenarioObject name=\"", scenarioObjectName);
                fillplaceholder(xosc_vehicle, "Vehicle name=\"", vehicle->get_name());
                fillplaceholder(xosc_vehicle, "z=\"", vehicle->get_bbox().z());
                fillplaceholder(xosc_vehicle, "width=\"", 2*vehicle->get_bbox().x());
                fillplaceholder(xosc_vehicle, "length=\"", 2*vehicle->get_bbox().y());
                fillplaceholder(xosc_vehicle, "height=\"", 2*vehicle->get_bbox().z());
                string typevalue = vehicle->get_isEgo() ? "ego_vehicle" : "simulation";
                fillplaceholder(xosc_vehicle, "value=\"", typevalue);
                // maxSteering, wheelDiameter also possible if we keep this info in Vehicle in future
                fillplaceholder(xosc, "  </Entities>", xosc_vehicle);
            }
            if (child.second->getType() == "Walker")
            {
                Walker * walker = dynamic_cast<Walker*>(child.second);
                string xosc_walker(xosc_template_pedestrian);
                string scenarioObjectName = "Ped_" + to_string(walker->getID());
                objectNames.push_back(scenarioObjectName);
                fillplaceholder(xosc_walker, "ScenarioObject name=\"", scenarioObjectName);
                fillplaceholder(xosc_walker, "Pedestrian model=\"", walker->get_name());
                fillplaceholder(xosc_walker, "z=\"", walker->get_bbox().z());
                fillplaceholder(xosc_walker, "width=\"", 2*walker->get_bbox().x());
                fillplaceholder(xosc_walker, "length=\"", 2*walker->get_bbox().y());
                fillplaceholder(xosc_walker, "height=\"", 2*walker->get_bbox().z());
                fillplaceholder(xosc, "  </Entities>", xosc_walker);
            }
        }
        string xosc_global_action(xosc_template_global_action);
        fillplaceholder(xosc_global_action, "dateTime=\"", string(timeString));
        fillplaceholder(xosc, "      </Actions>", xosc_global_action);
        auto onit = objectNames.begin(); // onit == object names iterator
        for (auto && child : m_scenario.children())
        {
            if (child.second->getType() == "Vehicle" || child.second->getType() == "Walker")
            {
                Actor * actor = dynamic_cast<Actor*>(child.second);
                string xosc_action_member(xosc_template_action_member);
                fillplaceholder(xosc_action_member, "Private entityRef=\"", *onit);
                auto waypath = actor->getFirstWaypath();
                if (waypath && !waypath->children().empty())
                {
                    auto pos = waypath->getStartingPosition();
                    fillplaceholder(xosc_action_member, " x=\"", pos.x());
                    fillplaceholder(xosc_action_member, " y=\"", pos.y());
                    fillplaceholder(xosc_action_member, " z=\"", pos.z());
                    auto dir = waypath->getStartingDirection();
                    fillplaceholder(xosc_action_member, " r=\"", 0);
                    fillplaceholder(xosc_action_member, " p=\"", atan2(dir.z(), hypot(dir.x(), dir.y())));
                    fillplaceholder(xosc_action_member, " h=\"", atan2(dir.y(), dir.x()));
                    // fillplaceholder(xosc_action_member, "<AbsoluteTargetSpeed value=\"", wpoint->get_speed());
                }
                fillplaceholder(xosc, "      </Actions>", xosc_action_member);
                ++onit;
            }
        }
        onit = objectNames.begin(); // onit == object names iterator
        for (auto && child : m_scenario.children())
        {
            string xosc_story(xosc_template_story);
            string xosc_waypath_event(xosc_template_waypath_event);
            if (child.second->getType() == "Vehicle" || child.second->getType() == "Walker")
            {
                fillplaceholder(xosc_story, "name=\"", "Story " + *onit);
                Actor * actor = dynamic_cast<Actor*>(child.second);
                for (auto && child : actor->children())
                {
                    if (child.second->getType() == "Waypath")
                    {
                        Waypath * waypath = dynamic_cast<Waypath*>(child.second);
                        for (auto && child : waypath->children())
                        {
                            if (child.second->getType() == "Waypoint")
                            {
                                Waypoint * waypoint = dynamic_cast<Waypoint*>(child.second);
                                string xosc_waypoint(xosc_template_waypoint);
                                auto pos = waypoint->get_pos();
                                fillplaceholder(xosc_waypoint, " x=\"", pos.x());
                                fillplaceholder(xosc_waypoint, " y=\"", pos.y());
                                fillplaceholder(xosc_waypoint, " z=\"", pos.z());
                                fillplaceholder(xosc_waypath_event, "                      </Route>", xosc_waypoint);
                                string xosc_speed_event(xosc_template_speed_event);
                                fillplaceholder(xosc_speed_event, " entityRef=\"", *onit);
                                fillplaceholder(xosc_speed_event, " x=\"", pos.x());
                                fillplaceholder(xosc_speed_event, " y=\"", pos.y());
                                fillplaceholder(xosc_speed_event, " z=\"", pos.z() + actor->get_bbox().z());
                                fillplaceholder(xosc_speed_event, "AbsoluteTargetSpeed value=\"", waypoint->get_speed());
                                fillplaceholder(xosc_speed_event, "Event name=\"", "Actor_speed_at_" + to_string(waypoint->getID()));
                                fillplaceholder(xosc_speed_event, "Action name=\"", "Actor_speed_at_" + to_string(waypoint->getID()));
                                fillplaceholder(xosc_story, "        </Maneuver>", xosc_speed_event);
                            }
                        }
                    }
                }
            }
            fillplaceholder(xosc_story, "        </Maneuver>", xosc_waypath_event);
            fillplaceholder(xosc_story, " entityRef=\"", *onit);
            fillplaceholder(xosc, "  <StopTrigger/>", xosc_story);
            ++onit;
        }
        ofs_xosc << xosc << endl;
        ofs_xosc.close();
    }));
}
