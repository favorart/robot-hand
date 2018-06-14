#pragma once

#include "Point.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboPosTour.h"

namespace RoboPos
{
class TourEvo : public RoboPos::TourI
{
protected:
    //const Robo::frames_t big_lasts = std::min(min_lasts * 1000, Robo::musclesMaxLasts(_robo));
    const double side = 0.001;
    
    Point _base_pos{};

    const TargetI &_t;
    double _reached_dist = 0.01;
    double _oppo_penalty = 1.;

    Robo::frames_t minLasts();

    double reached_less(double dist) const { return (dist - _reached_dist / 2); }

    bool runNestedReset(IN const Robo::Control&, IN OUT Point &hit);

    bool runNestedPreMove(IN const Robo::Control&, IN Robo::frames_t max_frames, OUT Point &hit);

    bool runNestedStop(IN const Robo::bitset_t &muscles, IN bool stop);
    bool runNestedStop(IN const Robo::Control &controls, IN bool stop);

    bool runNestedStep(IN const Robo::bitset_t &muscles, IN Robo::frames_t lasts, OUT Point &hit);
    bool runNestedStep(IN const Robo::Control&, OUT Point &hit);

    bool runNestedForMuscle(Robo::joint_t, Robo::Control&, Point &robo_hit);
    bool runNestedForMuscleSteps(Robo::joint_t, Robo::Control&, Point&robo_hit);

public:
    TourEvo(RoboMoves::Store &store, Robo::RoboI &robo, const TargetI &target) :
        TourI(store, robo), _t(target)
    {}

    void setParams(double reached_dist, double oppo_penalty)
    {
        _reached_dist = reached_dist;
        _oppo_penalty = oppo_penalty;
    }
};
}
