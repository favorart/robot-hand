#include "StdAfx.h"

#include "RoboPosMDirections_Old.h"
#ifdef MDIR_OLD
//#include "WindowHeader.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"


using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------
RoboPos::DirectionPredictor::MainDirection::MainDirection(IN Robo::muscle_t m, IN Robo::RoboI &robo) : muscle(m)
{
    robo.move(Robo::Control{ { m, 0, robo.muscleMaxLasts(m) } });
    const Point &front = robo.trajectory().front().spec();
    for (const auto &state : robo.trajectory())
        shifts.emplace_back(state.spec().x - front.x, state.spec().y - front.y);
}

//------------------------------------------
RoboPos::DirectionPredictor::DirectionPredictor(IN Robo::RoboI &robo)
{
    robo.reset();
    _base_pos = robo.position();

    //auto oldRarity = robo.getVisitedRarity();
    //robo.setVisitedRarity(1);

    _directions.resize(robo.musclesCount());
    for (Robo::joint_t j = 0; j < robo.jointsCount(); ++j)
    {
        auto mo = Robo::RoboI::muscleByJoint(j, true);
        auto mc = Robo::RoboI::muscleByJoint(j, true);

        robo.setJoints({ { j, 100. } });
        _directions[mo] = MainDirection(mo, robo);

        robo.setJoints({ { j, 0. } });
        _directions[mc] = MainDirection(mc, robo);
    }
    //robo.setVisitedRarity(oldRarity);
}

//------------------------------------------
Point RoboPos::DirectionPredictor::predict(IN Robo::Control controls)
{
    Point pred_end;
    for (auto &c : controls)
    {
        const auto &d = _directions[c.muscle];
        if (c.lasts < d.shifts.size())
        {
            pred_end.x += d.shifts[c.lasts].x;
            pred_end.y += d.shifts[c.lasts].y;
        }
    }
    return pred_end;
}
#endif // MDIR_OLD
