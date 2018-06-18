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
        Point &cpL{ tank.status.curPos[0] };
        Point &cpR{ tank.status.curPos[1] };
         
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

            //for (muscle_t m = 0; m < robo.musclesCount(); ++m)
            //for (joint_t j = 0; j < robo.jointsCount(); ++j)
            //    tank.status.curPos[j] -= oscillate;
            cpL -= (oscillate);
            cpR -= (oscillate);

            tcout << cpL << ' ' << cpR << std::endl;

            if (oscillate.norm2() < RoboI::minFrameMove)
            {
                state = false;
                oscillate = { 0., 0. };

                for (muscle_t m = 0; m < robo.musclesCount(); ++m)
                    tank.status.shifts[m] = 0.;
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
        std::array<bool, Hand::joints> inters;
        //Hand::Joint min_inter = Hand::Joint::JInvalid;
        joint_t min_inter = Robo::JInvalid;

        for (joint_t joint = 0; joint < robo.jointsCount(); ++joint)
        {
            if ((hand.status.curPos[joint].x - hand.params.jointRadius) > RBorder.x) inters[joint] = true;
            if ((hand.status.curPos[joint].y - hand.params.jointRadius) > RBorder.y) inters[joint] = true;
            if ((hand.status.curPos[joint].x + hand.params.jointRadius) < LBorder.x) inters[joint] = true;
            if ((hand.status.curPos[joint].y + hand.params.jointRadius) < LBorder.y) inters[joint] = true;

            if (hand.angles[joint] == 0 || hand.angles[joint] == hand.params.maxAngles[joint])
                inters[joint] = true;

            if (inters[joint])
            {
                state = true;
                if (min_inter > joint)
                    min_inter = joint;
            }
        }

        joint_t joint = min_inter;
        if (min_inter != Robo::JInvalid || state)
        {
            const Point oscillate = vecBodyVelocity / 2.;
            for (joint_t j = 0; j < robo.jointsCount(); ++j)
            {
                // сочленение получившее импульс через другое сочленение имеет отклонение /2
                hand.status.curPos[j] += oscillate;
                // (2. * j); if (j > 1) --j;
            }

            if (oscillate.norm2() >= RoboI::minFrameMove)
                return true;

            state = false;
            for (muscle_t m = 0; m < robo.musclesCount(); ++m)
                hand.angles[m] = 0.;
        }
    }
    return false;
}
