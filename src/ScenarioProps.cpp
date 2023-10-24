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


ScenarioProps::ScenarioProps(Scenario & scenario) : m_scenario(scenario)
{
    QVBoxLayout * mainLayout = new QVBoxLayout();
    mainLayout->addWidget(new QLabel(QString(scenario.getType().c_str()) + " ID: " + QString::number(scenario.getID()), this));

    QGroupBox * general = new QGroupBox();
    QVBoxLayout * generalLayout = new QVBoxLayout();
    generalLayout->addWidget(new QLabel("Carla Town name:"));
    QLineEdit * townName = new QLineEdit();
    generalLayout->addWidget(townName);
    townName->setText(m_scenario.getTownName().c_str());
    connect(townName, &QLineEdit::textChanged, [this](const QString & text){ m_scenario.setTownName(text.toStdString()); });
    QPushButton * loadScenario = new QPushButton("Load Scenario", this);
    generalLayout->addWidget(loadScenario);
    QPushButton * saveScenario = new QPushButton("Save Scenario", this);
    generalLayout->addWidget(saveScenario);
    general->setLayout(generalLayout);
    mainLayout->addWidget(general);
    setLayout(mainLayout);

    QPushButton * addVehicle = new QPushButton("Add vehicle", this);
    mainLayout->addWidget(addVehicle);
    connect(addVehicle, &QPushButton::clicked, [this]()
    { 
        int id = (new Vehicle(&m_scenario))->getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Vehicle: index not found!");
            return;
        }
        emit signal_addVehicle(id);
    });

    QPushButton * addWalker = new QPushButton("Add walker", this);
    mainLayout->addWidget(addWalker);
    connect(addWalker, &QPushButton::clicked, [this]()
    { 
        int id = (new Walker(&m_scenario))->getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Walker: index not found!");
            return;
        }
        emit signal_addWalker(id);
    });

    QPushButton * addCamera = new QPushButton("Add Camera", this);
    mainLayout->addWidget(addCamera);
    connect(addCamera, &QPushButton::clicked, [this]()
    {
        int id = (new Camera(&m_scenario))->getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Camera: index not found!");
            return;
        }
        emit signal_addCamera(id);
    });

    QPushButton * clearScenario = new QPushButton("Clear Scenario", this);
    mainLayout->addWidget(clearScenario);

    // Add drop-down list to select Ego:
    mainLayout->addWidget(new QLabel("Set Ego by Vehicle ID:"));
    QStringList ls;
    ls << "No";
    for (auto && c : m_scenario.children())
    {
        if (c.second->getType() == "Vehicle")
        {
            Vehicle * vehicle = dynamic_cast<Vehicle*>(c.second);
            QString entry((to_string(vehicle->getID())).c_str());
            ls << entry;
        }
    }
    QComboBox * vehiclesList = new QComboBox(this);
    vehiclesList->addItems(ls);
    mainLayout->addWidget(vehiclesList);

    for (auto & c : m_scenario.children())
    {
        if (c.second->getType() == "Vehicle")
        {
            Vehicle * v = dynamic_cast<Vehicle*>(c.second);
            if (v->get_isEgo())
                vehiclesList->setCurrentText(to_string(v->getID()).c_str());
        }
    }

    connect(vehiclesList, &QComboBox::currentTextChanged, [this](const QString & qentry){
        string entry = qentry.toStdString();
        for (auto & c : m_scenario.children())
        {
            if (c.second->getType() == "Vehicle")
            {
                Vehicle * v = dynamic_cast<Vehicle*>(c.second);
                v->set_isEgo(false);
                if (qentry != "No" && qentry.toInt() == v->getID())
                    v->set_isEgo(true);
            }
        }
    });

    mainLayout->addStretch(1);

    connect(clearScenario, &QPushButton::clicked, [this]()
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "No undo option", "The scenario will be lost. Proceed?");
        if (reply == QMessageBox::Yes)
            signal_clear();
    });

    connect(loadScenario, &QPushButton::clicked, [this, townName](){
        QString name = QFileDialog::getOpenFileName(this, tr("Open Scenario"), "/home", tr("Scenarios (*.yaml)"));
        if (name.isEmpty()) return;
        
        ifstream ifs(name.toStdString());
        m_scenario.clear();
        stringstream ssf; ssf << ifs.rdbuf();
        YAML::Node root = YAML::Load(ssf.str());
        m_scenario.from_yaml(root);

        ifs.close();
        emit signal_update();
    });

    connect(saveScenario, &QPushButton::clicked, [this](){
        YAML::Node root;
        m_scenario.to_yaml(root);
        QString name = QFileDialog::getSaveFileName(this, tr("Save Scenario"), "/home/scenario.yaml", tr("Scenarios (*.yaml)"));
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
                text.insert(pos+placeholder.size(), ss.str());
            }
        };

        ofstream ofs_xosc(name.toStdString()+".xosc");
        string xosc_header(xosc_template_header);
        fillplaceholder(xosc_header, "LogicFile filepath=\"", m_scenario.getTownName());
        time_t unixt = time(nullptr);
        char timeString [256];
        std::strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%S", std::gmtime(&unixt));
        fillplaceholder(xosc_header, "date=\"", string(timeString));
        fillplaceholder(xosc_header, "description=\"", "Usecase_"+ string(timeString));
        ofs_xosc << xosc_header << endl;
        ofs_xosc << xosc_template_start_entities << endl;
        vector<string> objectNames;
        for (auto && it : m_scenario.children())
        {
            if (it.second->getType() == "Vehicle")
            {
                Vehicle * vehicle = dynamic_cast<Vehicle*>(it.second);
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
                ofs_xosc << xosc_vehicle << endl;
            }
            if (it.second->getType() == "Walker")
            {
                Walker * walker = dynamic_cast<Walker*>(it.second);
                string xosc_walker(xosc_template_pedestrian);
                string scenarioObjectName = "Ped_" + to_string(walker->getID());
                objectNames.push_back(scenarioObjectName);
                fillplaceholder(xosc_walker, "ScenarioObject name=\"", scenarioObjectName);
                fillplaceholder(xosc_walker, "Pedestrian model=\"", walker->get_name());
                fillplaceholder(xosc_walker, "z=\"", walker->get_bbox().z());
                fillplaceholder(xosc_walker, "width=\"", 2*walker->get_bbox().x());
                fillplaceholder(xosc_walker, "length=\"", 2*walker->get_bbox().y());
                fillplaceholder(xosc_walker, "height=\"", 2*walker->get_bbox().z());
                ofs_xosc << xosc_walker << endl;
            }
        }
        ofs_xosc << xosc_template_end_entities << endl;
        string xosc_storyboard(xosc_template_start_storyboard);
        fillplaceholder(xosc_storyboard, "dateTime=\"", string(timeString));
        ofs_xosc << xosc_storyboard << endl;
        auto onit = objectNames.begin(); // onit == object names iterator
        for (auto && it : m_scenario.children())
        {
            Vehicle * v = dynamic_cast<Vehicle*>(it.second);
            string xosc_action_member(xosc_template_action_member);
            if (it.second->getType() == "Vehicle" || it.second->getType() == "Walker")
            {
                Actor * actor = dynamic_cast<Actor*>(it.second);
                fillplaceholder(xosc_action_member, "Private entityRef=\"", *onit);
                auto waypath = actor->getFirstWaypath();
                if (waypath)
                {
                    auto pos = waypath->getStartingPosition();
                    fillplaceholder(xosc_action_member, "x=\"", pos.x());
                    fillplaceholder(xosc_action_member, "y=\"", pos.y());
                    fillplaceholder(xosc_action_member, "z=\"", pos.z());
                    auto dir = waypath->getStartingDirection();
                    fillplaceholder(xosc_action_member, "r=\"", 0);
                    fillplaceholder(xosc_action_member, "p=\"", atan2(dir.z(), hypot(dir.x(), dir.y())));
                    fillplaceholder(xosc_action_member, "h=\"", atan2(dir.y(), dir.x()));
                }
                Waypoint * wpoint = actor->getFirstWaypoint();
                if (wpoint)
                {
                    fillplaceholder(xosc_action_member, "<AbsoluteTargetSpeed value=\"", wpoint->get_speed());
                }
                ofs_xosc << xosc_action_member << endl;
                ++onit;
            }
        }
        ofs_xosc << xosc_template_end_init_storyboard << endl;
        ofs_xosc << xosc_template_story << endl;
        ofs_xosc << xosc_template_footer << endl;
        ofs_xosc.close();
    });
}
