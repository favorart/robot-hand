#pragma once

#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
//#include "Old\RoboPosMDirections.h"
#include "Old\RoboPosMDirections_Old.h"


namespace RoboPos {
class Approx;
struct Counters;

class Tour
{
protected:
    std::shared_ptr<Robo::RoboI> _robo{};
    std::shared_ptr<RoboMoves::Store> _store{};
    std::shared_ptr<Counters> _counters{};

    Point _base_pos{};
    size_t _complexity = 0;

    double _step_distance{ 0. };
    Robo::frames_t _lasts_step_increment{ 0 };

    Robo::frames_t _lasts_step_increment_thick = 20;
    Robo::frames_t _lasts_step_increment_init = 1;

    Robo::frames_t _lasts_step_braking_init = 10; // 30
    Robo::frames_t _lasts_step_braking_incr = 2;

    bool _b_distance{}, _b_target{}, _b_braking{}, _b_predict{}, _b_checking{};

public:
    Tour(IN RoboMoves::Store *store, IN Robo::RoboI *robo);
    virtual ~Tour() {}
    virtual bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high) = 0;
    virtual bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos) = 0;

    void run(bool distance, bool target, bool braking, bool predict, bool checking,
             double step_distance, Robo::frames_t lasts_step_increment);
};

class TourNoRecursion : public Tour
{
    struct BorderLasts { Robo::frames_t min_lasts, max_lasts; };
    using Borders = std::vector<BorderLasts>;
    Borders _borders;

    RecTarget &_target;
    size_t  _max_nested = 0;
    Approx &_approx;

    /// !!!
    std::vector<Robo::Actuator> _breakings_controls = {}; ///< массив управлений-торможений для каждого сочленения
    std::shared_ptr<RoboPos::DirectionPredictor> pDP;

    bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high);
    bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos);

public:
    TourNoRecursion(IN RoboMoves::Store *store, IN Robo::RoboI *robo, IN RecTarget &target, Approx &approx) :
        Tour(store, robo), _target(target),
        _max_nested(_robo->jointsCount()),
        _breakings_controls(_max_nested), /// ???
        pDP{ std::make_shared<RoboPos::DirectionPredictor>(*_robo) },
        _approx(approx)
    {
        _lasts_step_increment_thick = 20;
        _lasts_step_increment_init = 25;
        _lasts_step_braking_incr = 5;
        _lasts_step_braking_init = 5;
        defineTargetBorders(0.05);
    }

    void specifyBordersByRecord(const RoboMoves::Record &rec);
    void defineTargetBorders(Robo::distance_t side);
};

}
