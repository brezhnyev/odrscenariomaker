#include "Viewer.h"
#include "Vehicle.h"

#include <QGLViewer/vec.h>
#include <QtWidgets/QMessageBox>

#include <iostream>
#include <sstream>


#include <stdio.h> 

namespace net
{
    #include <sys/socket.h>
    #include <arpa/inet.h> 
} 
#include <unistd.h> 
#include <string.h>
#define PORT 12345 

using namespace std;
using namespace qglviewer;
using namespace Eigen;


Viewer::Viewer(Scenario & scenario) : m_scenario(scenario), m_commThread(nullptr)
{
    using namespace net;

    // cout << "Version: " << glGetString(GL_VERSION) << endl; // KB: causes cout stop!
    // cout << "GLSL: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
    // cout.flush();
    camera()->setSceneRadius(200);
    camera()->fitSphere(Vec(0, 0, 0), 200);
    camera()->setZNearCoefficient(0.01);
}

void Viewer::init()
{
    m_scenario.init();
    int id = m_scenario.addVehicle();
    emit signal_addVehicle(id); // must be done over gui later
}

void Viewer::draw()
{
    m_scenario.draw();
}

void Viewer::drawWithNames()
{
    m_scenario.drawWithNames();
}

void Viewer::postSelection(const QPoint &point)
{
    qglviewer::Vec orig, dir;
    // Compute orig and dir, used to draw a representation of the intersecting
    // line
    camera()->convertClickToLine(point, orig, dir); // orig == camera position

    // Find the selectedPoint coordinates, using camera()->pointUnderPixel().
    bool found;
    Vec sp = camera()->pointUnderPixel(point, found);

    if (selectedName() == -1) return;

    if (selectedName() == 1) // Canvas
    {
        int id = m_scenario.addWaypoint(Vector3f(sp.x, sp.y, sp.z));
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Add/activate waypath in Actor!");
            return;
        }
        emit signal_addWaypoint(id);
    }
    else
    {
        int id = selectedName();
        m_scenario.select(id); // In graphics we can only select waypoint
        emit signal_select(id);
        // KB: no need to call update, since this function is a virtual and the update is called after it from OGLViewer
    }
}

void Viewer::slot_select(int id)
{
    m_scenario.select(id); // in the tree we can select both waypoint and waypath
    emit signal_select(id);
    update();
}

void Viewer::slot_play()
{
    Waypath & wp = *m_scenario.getActiveWaypath();
    if (wp.children().size() < 2)
    {
        cout << "At least two waypoints needed" << endl;
        return;
    }
    string command = "./client/client \"" + wp.serialize() + "\" &";
    system(command.c_str());

    usleep(2e6);

    if (m_commThread && !m_commThread->joinable()) delete m_commThread;

    m_commThread = new std::thread(
        [&, this]()
        {
            listenForResponse();
        }
    );
    m_commThread->detach();
}

void Viewer::listenForResponse()
{
    using namespace net;

    m_sock = 0;
    static struct sockaddr_in serv_addr;

    if ((m_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    { 
        printf("\n Socket creation error \n");
        return; 
    } 
   
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)
    { 
        printf("\nInvalid address/ Address not supported \n");
        return; 
    }

    if (net::connect(m_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    { 
        printf("\nConnection Failed \n");
        return;
    }

    while (true)
    {
        using namespace net;
        char buffer[1024] = {0};

        int valread = read( m_sock , buffer, 1024);
        if (valread == 1 && buffer[0] == '*') break;
        if (!valread) return;
        stringstream ss(buffer);
        float x; ss >> x;
        float y; ss >> y;
        float z; ss >> z;
        float yaw; ss >> yaw;
        Actor * actor = m_scenario.getActiveActor();
        if (actor)
            actor->setTrf(Vector3f(x,-y,z), -yaw);
        update();
    }

    shutdown(m_sock, net::SHUT_RDWR);
}

void Viewer::slot_stop()
{
    shutdown(m_sock, net::SHUT_RDWR);
}