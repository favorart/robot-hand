#include "StdAfx.h"

#include "Robo.h"
#include "RoboEdges.h"
#include "RoboPhysicsStatus.h"
#include "Hand.h"
#include "Tank.h"

using namespace Robo;
using namespace Robo::Mobile;
using namespace Robo::NewHand;
//-------------------------------------------------------------------------------
Robo::EnvEdges::EnvEdges(const RoboPhysics &robo, distance_t backpath, distance_t damping)
    : robo_(robo), backpath_ratio_(backpath /*/100*/), damping_(damping)
{}
//-------------------------------------------------------------------------------
Robo::distance_t Robo::EnvEdges::interaction(joint_t joint) const
{
    if (!robo_.status->somethingMoving()) // || robo_.moveEnd())
        collision = false;

    if (!containE(robo_.getEnvCond(), ENV::EDGES))// || !robo_.status->jointMovingOn(joint))
        return 1.;

    if (isCollision(joint))
    {
        collision = true;

        const muscle_t mo = robo_.muscleByJoint(joint, true);
        const muscle_t mc = robo_.muscleByJoint(joint, false);

        robo_.status->edgesDampLasts(mo, damping_);
        robo_.status->edgesDampLasts(mc, damping_);

        return -backpath_ratio_;
    }
    return 1.;
}

//-------------------------------------------------------------------------------
Robo::EnvEdgesTank::EnvEdgesTank(const RoboPhysics &robo, distance_t backpath, distance_t damping)
    : EnvEdges(robo, backpath, damping), tank_(static_cast<const Mobile::Tank&>(robo))//, backpath_angle_(backpath_ratio_ * M_PI)
{}
//-------------------------------------------------------------------------------
bool Robo::EnvEdgesTank::checkBorders(joint_t) const
{
    const Point& cpL{ tank_.curPos(0) };
    const Point& cpR{ tank_.curPos(1) };
    //const auto W = tank_.params.trackWidth / 2;
    const auto H = tank_.params.trackHeight;

    Point normal{ (cpL - cpR).orto() };
    normal *= H / normal.norm2();

    Point corners[]{ cpL + normal, cpL - normal ,
                     cpR + normal, cpR - normal };
    for (auto &c : corners)
        if (c.x < borders_[0] || c.y < borders_[0] ||
            c.x > borders_[1] || c.y > borders_[1])
        {
            CDEBUG("  curPos {" << borders_[1] << " < " << c << " < " << borders_[0] << "}");
            return true;
        }
    return false;
}
//-------------------------------------------------------------------------------
Robo::distance_t Robo::EnvEdgesTank::interaction(joint_t joint) const
{
    return EnvEdges::interaction(joint);

    //for (muscle_t m = 0; m < tank_.musclesCount(); ++m)
    //tank_.status->damping(joint, damping_);

    //return -((/*rotate*/) ? backpath_angle_ : backpath_ratio_);
    //return -backpath_ratio_;
}

//-------------------------------------------------------------------------------
Robo::EnvEdgesHand::EnvEdgesHand(const RoboPhysics &robo, distance_t backpath, distance_t damping)
    : EnvEdges(robo, backpath, damping), hand_(static_cast<const NewHand::Hand&>(robo_))
{}
//-------------------------------------------------------------------------------
bool Robo::EnvEdgesHand::isCollision(joint_t joint) const
{
    return (checkFullOpened(joint) || checkFullClosed(joint) || checkBorders(joint));
}
//-------------------------------------------------------------------------------
bool Robo::EnvEdgesHand::checkFullOpened(joint_t joint) const
{
    const muscle_t mo = robo_.muscleByJoint(joint, true);
    //const muscle_t mc = hand_.muscleByJoint(joint, false);
    if (robo_.status->movingOnFrame(mo) && (hand_.angles[joint] > (hand_.maxAngleJoint(joint) - RoboI::minFrameMove)))
    {
        //CDEBUG("  mo[" << mo << "]=" << robo_.status->movingOnFrame(mo) << " mc[" << mc << "]=" << robo_.status->movingOnFrame(mc));
        CDEBUG("  angles[" << joint << "]=" << hand_.angles[joint] << " > " << (hand_.maxAngleJoint(joint) - RoboI::minFrameMove));
        return true;
    }
    return false;
}
//-------------------------------------------------------------------------------
bool Robo::EnvEdgesHand::checkFullClosed(joint_t joint) const
{
    //const muscle_t mo = RoboI::muscleByJoint(joint, true);
    const muscle_t mc = robo_.muscleByJoint(joint, false);
    if (robo_.status->movingOnFrame(mc) && (hand_.angles[joint] < RoboI::minFrameMove))
    {
        //CDEBUG("  mo[" << mo << "]=" << robo_.status->movingOnFrame(mo) << " mc[" << mc << "]=" << robo_.status->movingOnFrame(mc));
        CDEBUG("  angles[" << joint << "]=" << hand_.angles[joint] << " < " << RoboI::minFrameMove);
        return true;
    }
    return false;
}
//-------------------------------------------------------------------------------
bool Robo::EnvEdgesHand::checkBorders(joint_t joint) const
{
    for (joint_t j = 0; j <= joint; ++j)
    {
        Point c_max = hand_.curPos(j) + hand_.params.jointRadius;
        Point c_min = hand_.curPos(j) - hand_.params.jointRadius;

        if (c_max.x > borders_[1] || c_max.y > borders_[1])
        {
            CDEBUG("  curPos[" << j << "] {" << borders_[1] << " < " << c_max << " < " << borders_[0] << "}");
            return true;
        }
        if (c_min.x < borders_[0] || c_min.y < borders_[0])
        {
            CDEBUG("  curPos[" << j << "] {" << borders_[1] << " < " << c_min << " < " << borders_[0] << "}");
            return true;
        }
    }
    return false;
}
//-------------------------------------------------------------------------------
Robo::distance_t Robo::EnvEdgesHand::interaction(joint_t joint) const
{
    if (EnvEdges::interaction(joint) == 1.)
        return 1.;
    const auto mo = robo_.muscleByJoint(joint, true);
    return -/*distance_t(robo_.muscleMaxLasts(mo)) /*/ backpath_ratio_;
}

//-------------------------------------------------------------------------------
void Robo::RoboPhysics::Status::edgesDampLasts(muscle_t m, distance_t damp)
{
    if (!musclesMove[m])
        return;
    frames_t& lastsDrive = (movingOn(m) ? lastsMove[m] : lastsStop[m]);
    //auto  lastsDriveNew = std::min(lastsDrive - 1, lasts[m]);
    //CDEBUG("  lastsMove[" << m << "]=" << lastsDrive << " - " << lastsDriveNew << " =" << (lastsDrive - lastsDriveNew));
    lastsDrive -= std::min(lastsDrive - 1, lasts[m]);
    lasts[m] = 0; // TURN-OFF muscle if collision
    prevFrame[m] /= damp;
}

