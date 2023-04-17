#include "ScenarioProps.h"
#include "Vehicle.h"
#include "Camera.h"
#include "Serializer.h"
#include "Walker.h"
#include "Waypoint.h"
#include "Waypath.h"

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
    connect(clearScenario, &QPushButton::clicked, [this]()
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this, "No undo option", "The scenario will be lost. Proceed?");
        if (reply == QMessageBox::Yes)
            signal_clear();
    });


    // addd rosbag file
    QGroupBox * rosGroup = new QGroupBox(this);
    QVBoxLayout * rosLayout = new QVBoxLayout();
    rosGroup->setLayout(rosLayout);

    rosLayout->addWidget(new QLabel("Rosbag file:", rosGroup));
    QLineEdit * rosBagfile = new QLineEdit(rosGroup);
    rosBagfile->setText(m_scenario.getRosbagFile().c_str());
    connect(rosBagfile, &QLineEdit::textChanged, [this](const QString & text){ m_scenario.setRosbagFile(text.toStdString()); });
    rosLayout->addWidget(rosBagfile);

    rosLayout->addWidget(new QLabel("Rosbag topic:", rosGroup));
    QTextEdit * rosTopics = new QTextEdit(rosGroup);
    for (auto && topic : m_scenario.getRosbagTopics())
        rosTopics->append(topic.c_str());
    connect(rosTopics, &QTextEdit::textChanged, [this, rosTopics]()
    {
        stringstream ss(rosTopics->toPlainText().toStdString());
        vector<string> topics; string line;
        while (ss >> line) topics.push_back(line);
        m_scenario.setRosbagTopics(topics);
    });
    rosLayout->addWidget(rosTopics);

rosLayout->addWidget(new QLabel("Rosbag playback offset:"));
QLineEdit * rosTimeOffset = new QLineEdit(rosGroup);
    rosTimeOffset->setText(QString::number(m_scenario.getRosbagOffset()));
    connect(rosTimeOffset, &QLineEdit::textChanged, [this](const QString & text){ m_scenario.setRosbagOffset(text.toFloat()); });
    rosLayout->addWidget(rosTimeOffset);
    mainLayout->addWidget(rosGroup);

    connect(loadScenario, &QPushButton::clicked, [this, townName, rosBagfile, rosTopics, rosTimeOffset](){
        QString name = QFileDialog::getOpenFileName(this, tr("Open Scenario"), "/home", tr("Scenarios (*.yaml)"));
        if (name.isEmpty()) return;
        
        ifstream ifs(name.toStdString());
        m_scenario.clear();
        stringstream ssf; ssf << ifs.rdbuf();
        Serializer::deserialize_yaml(m_scenario, ssf.str());

        ifs.close();
        emit signal_update();
    });

    connect(saveScenario, &QPushButton::clicked, [this](){
        string contents = Serializer::serialize_yaml(&m_scenario);
        QString name = QFileDialog::getSaveFileName(this, tr("Save Scenario"), "/home/scenario.yaml", tr("Scenarios (*.yaml)"));
        ofstream ofs(name.toStdString());
        ofs << contents << endl;
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
                            ofs_waypath << pos[0] << "," << -pos[1] << "," << (pos[2] + 0.5*v->get_bbox()[2]) << ", "; 
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
    });
}
