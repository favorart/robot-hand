﻿#include "StdAfx.h"

#include "Robo.h"
#include "RoboEdges.h"
#include "Hand.h"
#include "Tank.h"


using namespace Robo::Mobile;
using namespace Robo::NewHand;

bool Robo::EnvEdgesTank::border() const
{
    const Point& cpL{ tank_.status.curPos[0] };
    const Point& cpR{ tank_.status.curPos[1] };
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

void Robo::EnvEdgesTank::interaction(const Point &center, const Point &moved, double tan_angle)
{
    if (!used_ || tank_.moveEnd())
        return;

    const bool moves0 = (tank_.status.musclesMove[0] > 0 || tank_.status.musclesMove[1] > 0);
    const bool moves1 = (tank_.status.musclesMove[2] > 0 || tank_.status.musclesMove[3] > 0);

    if ((moves0 || moves1) && border())
    {
        if (fabs(tan_angle) > 0)
        {
            const joint_t l_track = (tank_.params.jointsUsed[0] == Tank::Joint::LTrack) ? 0 : 1;
            const joint_t r_track = (tank_.params.jointsUsed[0] == Tank::Joint::RTrack) ? 0 : 1;
            Point &cpL{ tank_.status.curPos[l_track] };
            Point &cpR{ tank_.status.curPos[r_track] };
            cpL.rotate_radians(center, -std::atan(tan_angle * backpath_angle_));
            cpR.rotate_radians(center, +std::atan(tan_angle * backpath_angle_));
        }
        else
        {
            tank_.status.curPos[0] -= moved * backpath_ratio_;
            tank_.status.curPos[1] -= moved * backpath_ratio_;
        }

        const joint_t jcenter = joint_t(Tank::Joint::Center);
        tank_.status.curPos[jcenter] = (tank_.status.curPos[0] + tank_.status.curPos[1]) / 2.; // mid point

        for (muscle_t m = 0 ; m < tank_.muscles; ++m)
        {
            auto &lasts = (tank_.status.lastsMove[m] > 0) ? tank_.status.lastsMove : tank_.status.lastsStop;
            lasts[m] -= tank_.status.lasts[m];
            tank_.status.lasts[m] -= tank_.status.lasts[m];
            tank_.status.prevFrame[m] /= damping_;
        }
    }
}


bool Robo::EnvEdgesHand::full_opened(joint_t joint) const
{ return (hand_.angles[joint] > (hand_.params.maxAngles[joint] - RoboI::minFrameMove)); }

bool Robo::EnvEdgesHand::full_closed(joint_t joint) const
{ return (hand_.angles[joint] < RoboI::minFrameMove); }

bool Robo::EnvEdgesHand::border(joint_t joint) const
{
    for (joint_t j = 0; j <= joint; ++j)
        if ((hand_.status.curPos[j].x + hand_.params.jointRadius) > borders_[1] ||
            (hand_.status.curPos[j].y + hand_.params.jointRadius) > borders_[1] ||
            (hand_.status.curPos[j].x - hand_.params.jointRadius) < borders_[0] ||
            (hand_.status.curPos[j].y - hand_.params.jointRadius) < borders_[0])
            return true;
    return false;
}

void Robo::EnvEdgesHand::interaction()
{
    if (!used_ || hand_.moveEnd())
        return;

    for (joint_t joint = 0; joint < hand_.jointsCount(); ++joint)
    {
        const double mAn = (hand_.maxJointAngle(joint));
        const muscle_t mo = hand_.muscleByJoint(joint, true);
        const muscle_t mc = hand_.muscleByJoint(joint, false);
        const bool moves = (hand_.status.musclesMove[mo] > 0 || hand_.status.musclesMove[mc] > 0);
         
        if ( !(moves && ( full_opened(joint) || full_closed(joint) || border(joint) )) )
            continue;
        
        if (hand_.status.lastsMove[mo] > 0 || hand_.status.lastsStop[mo] > 0) hand_.muscleDriveFrame(mo);
        if (hand_.status.lastsMove[mc] > 0 || hand_.status.lastsStop[mc] > 0) hand_.muscleDriveFrame(mc);

        distance_t offset;
        offset = /*-*/(hand_.status.shifts[mc] - hand_.status.shifts[mo]) * (hand_.muscleMaxLasts(mc) / backpath_ratio_);
        offset = (0.0 > (hand_.angles[joint] + offset)) ? (0.0 - hand_.angles[joint]) : offset;
        offset = (mAn < (hand_.angles[joint] + offset)) ? (mAn - hand_.angles[joint]) : offset;

        hand_.jointMove(joint, offset);

        for (muscle_t m : { mo, mc })
        {
            auto &lasts = (hand_.status.lastsMove[m] > 0) ? hand_.status.lastsMove : hand_.status.lastsStop;
            lasts[m] -= hand_.status.lasts[m];
            hand_.status.lasts[m] -= hand_.status.lasts[m];
            hand_.status.prevFrame[m] /= damping_;
        }        
    }
}
