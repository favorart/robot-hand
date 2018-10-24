#include "StdAfx.h"

#include "RoboPosMDirections_Old.h"
#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"

#ifdef MDIR_OLD
using namespace RoboPos;
using namespace RoboMoves;

//------------------------------------------
RoboPos::DirectionPredictor::MainDirection::MainDirection(IN Robo::muscle_t m, IN Robo::RoboI &robo) : muscle(m)
{
    Robo::Trajectory trajectory;
    robo.move(Robo::Control{ { m, 0, robo.muscleMaxLast(m) } }, trajectory);

    Point &front = trajectory.front();
    for (auto pt : trajectory)
        shifts.push_back(Point{ pt.x - front.x, pt.y - front.y });
}

//------------------------------------------
RoboPos::DirectionPredictor::DirectionPredictor(IN Robo::RoboI &robo)
{
    robo.reset();
    _base_pos = robo.position();

    auto oldRarity = robo.getVisitedRarity();
    robo.setVisitedRarity(1);

    _directions.resize(robo.musclesCount());
    for (Robo::joint_t j = 0; j < robo.jointsCount(); ++j)
    {
        Robo::muscle_t muscle;

        robo.setJoints({ { j, 100. } });
        muscle = Robo::RoboI::muscleByJoint(j, true);
        _directions[muscle] = MainDirection(muscle, robo);

        robo.reset();

        robo.setJoints({ { j, 0. } });
        muscle = Robo::RoboI::muscleByJoint(j, false);
        _directions[muscle] = MainDirection(muscle, robo);

        robo.reset();
    }
    //---------------------------------
    robo.setVisitedRarity(oldRarity);
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
