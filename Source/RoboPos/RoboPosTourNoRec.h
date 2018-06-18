#pragma once

#include "Robo.h"
#include "RoboPos.h"
//#include "Old\RoboPosMDirections.h"
#include "Old\RoboPosMDirections_Old.h"


namespace RoboPos
{

class Approx;

class Tour
{
protected:
    RoboMoves::Store &_store;
    Robo::RoboI      &_robo;
    Point             _base_pos{};
    Counters          _counters{};
    size_t            _complexity = 0;

    double _step_distance{ 0. };
    Robo::frames_t _lasts_step_increment{ 0 };

    Robo::frames_t _lasts_step_increment_thick = 20;
    Robo::frames_t _lasts_step_increment_init = 1;

    Robo::frames_t _lasts_step_braking_init = 10; // 30
    Robo::frames_t _lasts_step_braking_incr = 2;

    bool _b_distance{}, _b_target{}, _b_braking{}, _b_predict{}, _b_checking{};

public:
    Tour(IN RoboMoves::Store &store, IN Robo::RoboI &robo) :
        _store(store), _robo(robo)
    {}

    void  run(bool distance, bool target, bool braking, bool predict, bool checking,
              double step_distance, Robo::frames_t lasts_step_increment)
    {
        _b_distance = distance;
        _b_target = target;
        _b_braking = braking;
        _b_predict = predict;
        _b_checking = checking;
        // ----------------------------------------------------
        _step_distance = step_distance;
        _lasts_step_increment = lasts_step_increment;
        // ----------------------------------------------------
        _complexity = 0;
        _counters.clear();
        // ----------------------------------------------------
        _robo.reset();
        _base_pos = _robo.position();
        // ----------------------------------------------------
        try
        {
            if (_step_distance < DBL_EPSILON || _lasts_step_increment == 0)
                throw std::runtime_error{ "Increment Params aren't set" };

            Point useless;
            runNestedForMuscle(0, Robo::Control{}, useless);
        }
        catch (boost::thread_interrupted&)
        {
            CINFO("WorkingThread interrupted");
            //return;
        }
        catch (const std::exception &e)
        {
            CERROR(e.what());
            //return;
        }
        // ----------------------------------------------------
        if (_b_checking) { _counters.print(); }
        tcout << _T("\nStep: ") << (_step_distance / 0.0028) << _T("mm.");
        tcout << _T("\nComplexity: ") << _complexity;
        tcout << _T("  minutes:") << double(_complexity) / 60. << std::endl;
    }

    virtual bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high) = 0;
    virtual bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos) = 0;
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
    TourNoRecursion(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target, Approx &approx) :
        Tour(store, robo), _target(target),
        _max_nested(_robo.jointsCount()),
        _breakings_controls(_max_nested), /// ???
        pDP{ std::make_shared<RoboPos::DirectionPredictor>(_robo) },
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
