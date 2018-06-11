#pragma once

#include "Point.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboPosTour.h"

namespace RoboPos
{
class TourEvo : public RoboPos::TourI
{
    const Robo::frames_t min_lasts = _lasts_step_increment_init; // 70;
    const Robo::frames_t big_lasts = std::min(min_lasts * 1000, Robo::musclesMaxLasts(_robo));
    const double side = 0.001;

    const TargetI &_t;
    double _reached_dist = 0.01;
    double _oppo_penalty = 1.;

    double reached_less(double dist) const { return (dist - _reached_dist / 2); }

public:
    TourEvo(RoboMoves::Store &store, Robo::RoboI &robo, const TargetI &target) :
        TourI(store, robo), _t(target)
    {}

    void setParams(double reached_dist, double oppo_penalty)
    {
        _reached_dist = reached_dist;
        _oppo_penalty = oppo_penalty;
    }

    bool runNestedForMuscle(Robo::joint_t, Robo::Control&, Point&);
    bool runNestedForMuscleSteps(Robo::joint_t, Robo::Control&, Point&);
};
}
