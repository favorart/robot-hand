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
    double _reached_dist;

    double reached_less(double dist) const { return (dist - _reached_dist / 2); }

public:
    TourEvo(RoboMoves::Store &store, Robo::RoboI &robo, RoboPos::borders_t borders,
            const TargetI &target, double reached_dist = 0.01, double oppo_penalty = 1.) :
        TourI(store, robo, borders), _t(target), _reached_dist(reached_dist)
    {}

    bool runNestedForMuscle(Robo::joint_t, Robo::Control&, Point&);
};
}
