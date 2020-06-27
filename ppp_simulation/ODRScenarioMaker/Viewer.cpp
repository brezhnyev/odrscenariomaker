#include "Viewer.h"
#include "Vehicle.h"

#include <QGLViewer/vec.h>
#include <QtWidgets/QMessageBox>

#include <iostream>
#include <sstream>
#include <thread>


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


Viewer::Viewer(Scenario & scenario) : m_canvas("../data/Town02.jpg", QRect(-27, 92, 239, 237)), m_scenario(scenario)
{
    using namespace net;

    cout << "Version: " << glGetString(GL_VERSION) << endl;
    cout << "GLSL: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
    cout.flush();
    camera()->setSceneRadius(200);
    camera()->fitSphere(Vec(0, 0, 0), 200);
    camera()->setZNearCoefficient(0.01);
}

void Viewer::init()
{
    m_canvas.init();
    int id = m_scenario.addWaypath();
    emit signal_addWaypath(id); // must be done over gui later
}

void Viewer::draw()
{
    m_canvas.draw();
    m_scenario.draw();
    m_vehicle.draw();
}

void Viewer::drawWithNames()
{
    m_canvas.drawWithNames();
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

    if (selectedName() == 0) // put a new waypoint
    {
        int id = m_scenario.addWaypoint(Vector3f(sp.x, sp.y, sp.z));
        emit signal_addWaypoint(id);
    }
    else
    {
        int id = selectedName();
        m_scenario.select(id); // In graphics we can only select waypoint
        emit signal_select(id);
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
    string command = "./client/client \"" + wp.serialize() + "\" &";
    system(command.c_str());

    usleep(2e6);

    std::thread * t = new std::thread(
        [&, this]()
        {
            listenForResponse();
            delete t;
        }
    );

    t->detach();
}

void Viewer::listenForResponse()
{
    using namespace net;

    int sock = 0;
    static struct sockaddr_in serv_addr;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
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

    if (net::connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("\nConnection Failed \n"); 
        return; 
    }

    while (true)
    {
        using namespace net;
        char buffer[1024] = {0}; 

        int valread = read( sock , buffer, 1024); 
        if (valread == 1 && buffer[0] == '*') break;
        stringstream ss(buffer);
        float x; ss >> x;
        float y; ss >> y;
        float z; ss >> z;
        float yaw; ss >> yaw;
        m_vehicle.setPosYaw(Vector3f(x,-y,z), -yaw);
        update();
    }

    shutdown(sock, net::SHUT_RDWR);
}