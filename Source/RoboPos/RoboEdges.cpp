#include "StdAfx.h"

#include "Robo.h"
#include "RoboEdges.h"
#include "RoboPhysicsStatus.h"
#include "Hand.h"
#include "Tank.h"


using namespace Robo::Mobile;
using namespace Robo::NewHand;

bool Robo::EnvEdgesTank::border() const
{
    const Point& cpL{ tank_.currPos(0) };
    const Point& cpR{ tank_.currPos(1) };
    //const auto W = tank_.params.trackWidth / 2;
    const auto H = tank_.params.trackHeight;

    Point normal{ (cpL - cpR).orto() };
    normal *= H / normal.norm2();

    Point corners[]{ cpL + normal, cpL - normal ,
                     cpR + normal, cpR - normal };
    for (auto &c : corners)
        if (c.x < borders_[0] || c.y < borders_[0] || 
            c.x > borders_[1] || c.y > borders_[1])
            return true;
    return false;
}

void Robo::EnvEdgesTank::interaction(bool used, const Point &center, const Point &moved, double tan_angle)
{
    if (!used || tank_.moveEnd())
        return;
    
    const bool movesL = (tank_.status->musclesMove[Tank::LTrackFrw] > 0 || tank_.status->musclesMove[Tank::LTrackBck] > 0);
    const bool movesR = (tank_.status->musclesMove[Tank::RTrackFrw] > 0 || tank_.status->musclesMove[Tank::RTrackBck] > 0);

    if (!(collision = isCollision((movesL || movesR), 0)))
        return;

    if (fabs(tan_angle) > 0./*eps?*/)
    {
        tank_.currPos(Tank::LTrack).rotate_radians(center, -std::atan(tan_angle * backpath_angle_));
        tank_.currPos(Tank::RTrack).rotate_radians(center, +std::atan(tan_angle * backpath_angle_));
    }
    else
    {
        tank_.currPos(Tank::LTrack) -= moved * backpath_ratio_;
        tank_.currPos(Tank::RTrack) -= moved * backpath_ratio_;
    }
    tank_.currPos(Tank::Center) = (tank_.currPos(Tank::LTrack) + tank_.currPos(Tank::RTrack)) / 2.; // mid point

    for (muscle_t m = 0 ; m < tank_.muscles; ++m)
    {
        auto &lasts = (tank_.status->lastsMove[m] > 0) ? tank_.status->lastsMove : tank_.status->lastsStop;
        lasts[m] -= tank_.status->lasts[m];
        tank_.status->lasts[m] -= tank_.status->lasts[m];
        tank_.status->prevFrame[m] /= damping_;
    }
}


bool Robo::EnvEdgesHand::full_opened(joint_t joint) const
{ return (hand_.angles[joint] > (hand_.params.maxAngles[joint] - RoboI::minFrameMove)); }

bool Robo::EnvEdgesHand::full_closed(joint_t joint) const
{ return (hand_.angles[joint] < RoboI::minFrameMove); }

bool Robo::EnvEdgesHand::border(joint_t joint) const
{
    for (joint_t j = 0; j <= joint; ++j)
        if ((hand_.currPos(j).x + hand_.params.jointRadius) > borders_[1] ||
            (hand_.currPos(j).y + hand_.params.jointRadius) > borders_[1] ||
            (hand_.currPos(j).x - hand_.params.jointRadius) < borders_[0] ||
            (hand_.currPos(j).y - hand_.params.jointRadius) < borders_[0])
            return true;
    return false;
}

void Robo::EnvEdgesHand::interaction(bool used, const Point&, const Point&, double)
{
    if (!used || hand_.moveEnd())
        return;

    collision = false;
    for (joint_t joint = 0; joint < hand_.jointsCount(); ++joint)
    {
        const double mAn = (hand_.maxJointAngle(joint));
        const muscle_t mo = hand_.muscleByJoint(joint, true);
        const muscle_t mc = hand_.muscleByJoint(joint, false);
        const bool moves = (hand_.status->musclesMove[mo] > 0 || hand_.status->musclesMove[mc] > 0);
         
        if (!isCollision(joint, moves))
            continue;
        collision = true;

        if (hand_.status->lastsMove[mo] > 0 || hand_.status->lastsStop[mo] > 0) hand_.muscleDriveFrame(mo);
        if (hand_.status->lastsMove[mc] > 0 || hand_.status->lastsStop[mc] > 0) hand_.muscleDriveFrame(mc);

        distance_t offset;
        offset = /*-*/(hand_.status->shifts[mc] - hand_.status->shifts[mo]) * (hand_.muscleMaxLasts(mc) / backpath_ratio_);
        offset = (0.0 > (hand_.angles[joint] + offset)) ? (0.0 - hand_.angles[joint]) : offset;
        offset = (mAn < (hand_.angles[joint] + offset)) ? (mAn - hand_.angles[joint]) : offset;

        hand_.jointMove(joint, offset);

        for (muscle_t m : { mo, mc })
        {
            auto &lasts = (hand_.status->lastsMove[m] > 0) ? hand_.status->lastsMove : hand_.status->lastsStop;
            lasts[m] -= hand_.status->lasts[m];
            hand_.status->lasts[m] -= hand_.status->lasts[m];
            hand_.status->prevFrame[m] /= damping_;
        }        
    }
}
