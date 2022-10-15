#include "Actor.h"
#include "scenario.h"

#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace Eigen;

Actor::~Actor() {} // pure virtual destructor is not allowed to be inline, so place it here

void Actor::updatePose()
{
    if (m_children.empty())
        return;
    Waypath * firstWaypath = static_cast<Waypath*>(m_children.begin()->second);
    setPos(firstWaypath->getStartingPosition());
    Vector3f dir = firstWaypath->getStartingDirection();
    float pitch = asin(dir[2]/dir.norm())*RAD2DEG;
    float yaw = atan2(dir[1], dir[0])*RAD2DEG;
    setOri(Vector3f(0,pitch,yaw));
}