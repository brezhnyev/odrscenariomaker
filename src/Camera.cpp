#include "Camera.h"

#include <GL/gl.h>

using namespace Eigen;

#define DEG2RAD M_PI/180

static const float box[][12] = {
    0,0,0, +1,-1,-1, +1,+1,-1, 0,0,0,
    0,0,0, +1,-1,+1, +1,+1,+1, 0,0,0,
    0,0,0, +1,-1,-1, +1,-1,+1, 0,0,0,
    0,0,0, +1,+1,-1, +1,+1,+1, 0,0,0,
    0,0,0, 0,0,0, 0,0,0, 0,0,0,
    +1,-1,-1, +1,-1,+1, +1,+1,+1, +1,+1,-1
};

static const float frustum[][6] = {
    0,0,0, +1,-1,-1,
    0,0,0, +1,-1,+1,
    0,0,0, +1,+1,-1,
    0,0,0, +1,+1,+1,
};

void Camera::drawGeometry() const
{
    glDisable(GL_TEXTURE_2D);

    float currentColor[4];
    glGetFloatv(GL_CURRENT_COLOR, currentColor);

    glPushMatrix(); // parent
    Actor * parent = dynamic_cast<Actor*>(m_parent);
    if (parent)
    {
        Eigen::Vector3f pos = parent->get_pos();
        glTranslatef(pos[0], pos[1], pos[2]);
        Eigen::Vector3f ori = parent->get_ori();
        glRotatef(ori[2],0,0,1);
        glRotatef(0,ori[1],0,1);
        glRotatef(0,0,ori[0],1);
    }

    glPushMatrix();
    Eigen::Matrix4f m; m.setIdentity();
    m.block(0,0,3,3) = AngleAxisf(m_ori[2]*DEG2RAD, Vector3f::UnitZ())*AngleAxisf(m_ori[1]*DEG2RAD, Vector3f::UnitY())*AngleAxisf(m_ori[0]*DEG2RAD, Vector3f::UnitX()).toRotationMatrix();
    m.block(0,3,3,1) = Vector3f(m_pos.x(), m_pos.y(), m_pos.z());
    glMultMatrixf(m.data());

    glColor3f(0,1,0);

    // a bit too complicated method to re-compute the frustum depending on width, height and FOV:
    // our frustum (see box and frustum arrays) rays are built at 45 degrees (corresponds to FOV=90 degree)
    Vector3f v = AngleAxisf(0.5f*m_FOV*DEG2RAD - M_PI_4, Vector3f::UnitZ()).toRotationMatrix()*Vector3f(1,1,1);
    float fx = v[0];
    float fy = v[1];
    float fz = fy*m_height/m_width; // also take the aspect into consideration

    if (m_selected)
    {
        glPushMatrix();
        glScalef(30.0f, 30.0f, 30.0f);
        for (auto && c : frustum)
        {
            glBegin(GL_LINES);
            glVertex3f(c[0]*fx, c[1]*fy, c[2]*fz);
            glVertex3f(c[3]*fx, c[4]*fy, c[5]*fz);
            glEnd();
        }
        glBegin(GL_LINE_LOOP);
        glVertex3f(box[5][0]*fx, box[5][1]*fy, box[5][2]*fz);
        glVertex3f(box[5][3]*fx, box[5][4]*fy, box[5][5]*fz);
        glVertex3f(box[5][6]*fx, box[5][7]*fy, box[5][8]*fz);
        glVertex3f(box[5][9]*fx, box[5][10]*fy,box[5][11]*fz);
        glEnd();
        glPopMatrix();
    }

    for (auto && c : box)
    {
        glBegin(GL_QUADS);
        glVertex3f(c[0]*fx, c[1]*fy, c[2]*fz);
        glVertex3f(c[3]*fx, c[4]*fy, c[5]*fz);
        glVertex3f(c[6]*fx, c[7]*fy, c[8]*fz);
        glVertex3f(c[9]*fx, c[10]*fy, c[11]*fz);
        glEnd();
    }

    glPopMatrix();
    glPopMatrix(); // parent transform (if any)

    glColor4fv(currentColor);
}

void Camera::draw() const
{
    drawGeometry();
    this->Selectable::draw();
}

void Camera::drawWithNames() const
{
    glPushName(m_id);
    drawGeometry();
    glPopName();
    for (auto && child : m_children) child.second->drawWithNames();
}

void Camera::to_yaml(YAML::Node & parent)
{
    YAML::Node node;
    node["type"] = getType();
    YAML::Node location;
    location["x"] = m_pos.x();
    location["y"] = m_pos.y();
    location["z"] = m_pos.z();
    node["location"] = location;
    YAML::Node orientation;
    orientation["roll"] = m_ori.x();
    orientation["pitch"] = m_ori.y();
    orientation["yaw"] = m_ori.z();
    node["orientation"] = orientation;
    YAML::Node camprops;
    camprops["width"] = m_width;
    camprops["height"] = m_height;
    camprops["fov"] = m_FOV;
    node["camprops"] = camprops;
    parent.push_back(node);
}

void Camera::from_yaml(const YAML::Node & node)
{
    auto location = node["location"];
    float x = location["x"].as<float>();
    float y = location["y"].as<float>();
    float z = location["z"].as<float>();
    m_pos = Eigen::Vector3f(x,y,z);
    auto orientation = node["orientation"];
    float roll = orientation["roll"].as<float>();
    float pitch = orientation["pitch"].as<float>();
    float yaw = orientation["yaw"].as<float>();
    m_ori = Eigen::Vector3f(roll,pitch,yaw);
    auto camprops = node["camprops"];
    m_width = camprops["width"].as<float>();
    m_height = camprops["height"].as<float>();
    m_FOV = camprops["fov"].as<float>();
}