#include "ScenarioProps.h"
#include "Vehicle.h"
#include "Serializer.h"

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


    QPushButton * addVehicle = new QPushButton("Add vehicle", this);
    mainLayout->addWidget(addVehicle);

    setLayout(mainLayout);

    connect(addVehicle, &QPushButton::pressed, [this]()
    { 
        int id = m_scenario.addChild(new Vehicle());
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Vehicle: index not found!");
            return;
        }
        emit signal_addVehicle(id);
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

    connect(loadScenario, &QPushButton::pressed, [this, townName, rosBagfile, rosTopics, rosTimeOffset](){
        QString name = QFileDialog::getOpenFileName(this, tr("Open Scenario"), "/home", tr("Scenarios (*.yaml)"));
        if (name.isEmpty()) return;
        
        ifstream ifs(name.toStdString());
        m_scenario.clear();
        rosTopics->clear();
        stringstream ssf; ssf << ifs.rdbuf();
        m_scenario = Serializer::deserialize_yaml(ssf.str());
        emit signal_update();

        townName->setText(m_scenario.getTownName().c_str());
        rosBagfile->setText(m_scenario.getRosbagFile().c_str());
        for (auto && topic : m_scenario.getRosbagTopics())
            rosTopics->append(topic.c_str());
        rosTimeOffset->setText(QString::number(m_scenario.getRosbagOffset()));
        ifs.close();
    });

    connect(saveScenario, &QPushButton::pressed, [this](){
        string contents = Serializer::serialize_yaml(&m_scenario);
        QString name = QFileDialog::getSaveFileName(this, tr("Save Scenario"), "/home/scenario.yaml", tr("Scenarios (*.yaml)"));
        ofstream ofs(name.toStdString());
        ofs << contents << endl;
        ofstream ofs_waypath(name.toStdString()+"_waypaths.yaml");
        for (auto && it : m_scenario.children())
        {
            Vehicle * v = dynamic_cast<Vehicle*>(it.second);
            if (v)
            {
                ofs_waypath << v->getName() << " ";
                for (auto && it : v->children())
                {
                    Waypath * w = dynamic_cast<Waypath*>(it.second);
                    if (w)
                    {
                        ofs_waypath << "[";
                        for (auto && it : w->children())
                        {
                            Waypoint * p = dynamic_cast<Waypoint*>(it.second);
                            auto pos = p->getPosition();
                            ofs_waypath << pos[0] << "," << -pos[1] << "," << pos[2] << ", "; 
                        }
                        ofs_waypath << "]" << "\n";
                    }
                }
            }
        }
        ofs.close();
    });
}
