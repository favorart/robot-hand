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
    const TargetI &_t;
    /*const*/ double _reached_dist = 0.000005;
    /*const*/ double _oppo_penalty = 0.5;
    /*const*/ double _prev_dist_add = Robo::RoboI::minFrameMove;
    /*const*/ Point _base_pos{};
    
    const Robo::muscle_t _n_muscles = 0;
    const Robo::muscle_t _n_acts = 0;
    const Robo::frames_t _lasts_max = 10000;
    const Robo::frames_t _lasts_init = 1;
    /*const*/ Robo::frames_t _lasts_step = 1;
    /*const*/ Robo::frames_t _max_ncontrols = 8;
    /*const*/ Robo::frames_t _step_back = 55;

    Robo::frames_t minLasts();

    bool runNestedReset(const Robo::Control&, Robo::muscle_t muscles, Robo::frames_t frame, Robo::frames_t lasts, const Point &aim, IN OUT Point &hit);
    bool runNestedPreMove(const Robo::Control&, Robo::muscle_t muscles, Robo::frames_t frame, const Point &aim, Point &hit);
    bool runNestedStop(const Robo::bitset_t &muscles, bool stop);
    //bool runNestedStop(const Robo::Control&, bool stop);
    //bool runNestedStep(const Robo::bitset_t &muscles, Robo::frames_t lasts, OUT Point &hit);
    //bool runNestedStep(const Robo::Control&, OUT Point &hit);
    //bool runNestedForMuscleSteps(Robo::joint_t, Robo::Control&, Point&hit);
    bool runNestedForMuscle(Robo::joint_t, Robo::Control&, Point &hit);

    const RoboMoves::Record* appendMarker(const Robo::Control &controls, const Robo::Control &new_controls, const Point &aim);
    bool containMarker(const Robo::Control&, Robo::muscle_t muscles, Robo::frames_t frame, const Point &aim);

public:
    TourEvo(RoboMoves::Store&, Robo::RoboI&, const TargetI&);
    void setParams(double reached_dist, double oppo_penalty)
    {
        _reached_dist = reached_dist;
        _oppo_penalty = oppo_penalty;
    }
};

class TourEvoSteps : public TourEvo
{
protected:
    bool runNestedForStep(const Robo::RoboI::bitwise &muscles, Point &robo_hit);
    bool runNestedForMuscle(Robo::joint_t, Robo::Control&, Point &robo_hit);
    bool compansateOverHit(Robo::Control&, const Point &goal);

public:
    TourEvoSteps(RoboMoves::Store&, Robo::RoboI&, const TargetI&);
};

/* Случайные движения 
   с отжигом
   с запоминаением..
*/


}
