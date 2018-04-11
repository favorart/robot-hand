#include "StdAfx.h"

#include "Robo.h"
#include "RoboEdges.h"
#include "Hand.h"
#include "Tank.h"


using namespace Robo::Mobile;
using namespace Robo::NewHand;
bool Robo::EnvEdgesTank::interaction(RoboI &robo, const Point &vecBodyVelocity)
{
    auto &tank = dynamic_cast<Tank&>(robo);

    if (state || vecBodyVelocity.norm2() >= Tank::minFrameMove)
    {
        Point &cpL{ tank.status.curPos[Tank::Joint::LTrack] };
        Point &cpR{ tank.status.curPos[Tank::Joint::RTrack] };
         
        const Point LEdge{ std::min(cpL.x, cpR.x) - tank.params.trackHeight,
                           std::min(cpL.y, cpR.y) - tank.params.trackWidth / 2 };
        const Point REdge{ std::max(cpL.x, cpR.x) + tank.params.trackHeight,
                           std::max(cpL.y, cpR.y) + tank.params.trackWidth / 2 };

        if (state || LEdge < LBorder || REdge > RBorder)
        {
            if (!state) //(oscillate.norm2() < Tank::minFrameMove)
                oscillate = -(vecBodyVelocity / 2.);// -vecBodyVelocity / 20.);
            else
                oscillate = -(oscillate / 2.);// -oscillate / 20.);
            state = true;

            tcout << _T("oscillate=") << oscillate << std::endl;

            ////for (muscle_t m = 0; m < robo->musclesCount(); ++m)
            //for (joint_t j = 0; j < tank.jointsCount(); ++j)
            //    tank.status.curPos[tank.J(j)] -= oscillate;
            cpL -= (oscillate);
            cpR -= (oscillate);

            tcout << cpL << ' ' << cpR << std::endl;

            if (oscillate.norm2() < Tank::minFrameMove)
            {
                state = false;
                oscillate = { 0., 0. };
                
                for (auto &muscle : tank.params.musclesUsed)
                    tank.status.shifts[muscle] = 0.;
                //return false; // full stop
            }
            return true; // interact         
        }
    }
    return false; // not interact
}


bool Robo::EnvEdgesHand::interaction(RoboI &robo, const Point &vecBodyVelocity)
{
    auto &hand = dynamic_cast<Hand&>(robo);

    // if (vecBodyVelocity > 0 && (maxAngle == 100 || maxAngle == 0))
    // if (curPos < LBorder || curPos > RBorder)

    if (state || vecBodyVelocity.norm2() >= Hand::minFrameMove)
    {
        //Point LEdge, REdge;
        std::array<bool, Hand::JointsMaxCount> inters;
        //Hand::Joint min_inter = Hand::Joint::JInvalid;
        joint_t min_inter = Robo::JInvalid;

        joint_t j = Robo::JInvalid;
        for (auto joint : hand.params.jointsUsed)
        {
            if ((hand.status.curPos[joint].x - hand.params.jointRadius) > RBorder.x) inters[joint] = true;
            if ((hand.status.curPos[joint].y - hand.params.jointRadius) > RBorder.y) inters[joint] = true;
            if ((hand.status.curPos[joint].x + hand.params.jointRadius) < LBorder.x) inters[joint] = true;
            if ((hand.status.curPos[joint].y + hand.params.jointRadius) < LBorder.y) inters[joint] = true;

            if (hand.status.angles[joint] == 0 || hand.status.angles[joint] == hand.physics.jointsMaxAngles[joint])
                inters[joint] = true;

            if (inters[joint])
            {
                state = true;
                if (min_inter > j)
                    min_inter = j;
            }
            ++j;
        }

        j = min_inter;
        if (min_inter != Hand::Joint::JInvalid || state)
        {
            const Point oscillate = vecBodyVelocity / 2.;
            for (auto joint : hand.params.jointsUsed)
            {
                // сочленение получившее импульс через другое сочленение имеет отклонение /2
                hand.status.curPos[joint] += oscillate;
                // (2. * j); if (j > 1) --j;
            }

            if (oscillate.norm2() >= Tank::minFrameMove)
                return true;

            state = false;
            for (auto &muscle : hand.params.musclesUsed)
                hand.status.angles[muscle] = 0.;
        }
    }
    return false;
}
