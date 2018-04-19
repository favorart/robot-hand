#pragma once

#include "Robo.h"
#include "RoboPos.h"

#include "Old\RoboPosMDirections.h"
#include "Old\RoboPosMDirections_Old.h"

namespace RoboPos {
//------------------------------------------------------------------------------
class Tour
{
protected:
    Robo::RoboI &_robo;
    Point _base_pos;
    RoboMoves::Store &_store;
    RecTarget &_target;
    borders_t &_borders;
    RoboPos::MainDirections _md;
    Counters _counters;
    size_t _complexity = 0;

    // --- flags ---
    bool _b_distance; ///<
    bool _b_target;   ///< обход всей мишени целиком
    bool _b_braking;  ///< использование торможения мускулом
    bool _b_predict;  ///<
    bool _b_checking; ///< проверка предсказаний основных направлений

    // --- distance --- 
    double _target_distance;
    double _step_distance;

    Robo::frames_t _lasts_step_increment;
    Robo::frames_t _lasts_step_increment_thick = 20;
    Robo::frames_t _lasts_step_increment_init = 1;

    Robo::frames_t _lasts_step_braking_init = 10; // 30
    Robo::frames_t _lasts_step_braking_incr = 2;
        
    /// Descrete tour around all over the workspace
    /// \param[in]   joint          focus of moving on the next joint
    /// \param[in]   controls       result control to robo-move
    /// \param[out]  robo_pos_high  best _hit_ using the next joint
    virtual bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high) = 0;
    /// Straight move by robot with chosen muscles
    /// \param[in]   controls       result control to robo-move
    /// \param[out]  robo_pos       best _hit_ using the next joint
    virtual bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos) = 0;

public:
    Tour(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target, IN borders_t &borders /*, IN const tstring &config=_T("")*/) :
        _store(store), _robo(robo), _target(target), _borders(borders),
        _md(MainDirectionsFactory(robo)),
        _target_distance(boost_distance(_target.min(), _target.max()) / 1.5)
    {
        _robo.reset();
        _base_pos = _robo.position();
    }
    //------------------------------------------------------------------------------
    virtual void run(bool distance, bool target, bool braking, bool predict, bool checking,
                     borders_t &borders, double step_distance, Robo::frames_t lasts_step_increment)
    {
        _b_target = target;
        _b_braking = braking;
        _b_distance = distance;
        _b_predict = predict;
        _b_checking = checking;

        _borders = borders;
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
        tcout << _T("\nStep: ") << (step_distance / 0.0028) << _T("mm.");
        tcout << _T("\nComplexity: ") << _complexity;
        tcout << _T("  minutes:") << double(_complexity) / 60. << std::endl;
    }
};

//------------------------------------------------------------------------------
class TourWorkSpace : public Tour
{
    size_t _max_nested = 0;
    size_t _breakings_controls_actives = 0;
    std::vector<Robo::Actuator> _breakings_controls = {}; ///< массив управлений-торможений для каждого сочленения

    bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high);
    bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos);

    void exactsBreakings(IN Robo::joint_t joint, IN const Robo::Control &controls);
    void appendBreakings(IN Robo::joint_t joint, IN const Robo::Actuator &a);
    void removeBreakings(IN Robo::joint_t joint);
    void  cleanBreakings(IN Robo::joint_t joint);

public:
    TourWorkSpace(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target, IN borders_t &borders) :
        Tour(store, robo, target, borders),
        _max_nested(robo.jointsCount()),
        _breakings_controls(_max_nested),
        _breakings_controls_actives(0)
    {}
    //------------------------------------------------------------------------------
    void  run(bool distance, bool target, bool braking, bool predict, bool checking,
              borders_t &borders, double step_distance, Robo::frames_t lasts_step_increment)
    {
        Tour::run(distance, target, braking, predict, checking, borders, step_distance, lasts_step_increment);
    }
};

//------------------------------------------------------------------------------
class TourNoRecursion : public Tour
{
    size_t _max_nested = 0;
    
    /// !!!
    std::vector<Robo::Actuator> _breakings_controls = {};
    std::shared_ptr<RoboPos::DirectionPredictor> pDP;

    bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high);
    bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos);

public:
    TourNoRecursion(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target, IN borders_t &borders) :
        Tour(store, robo, target, borders),
        _max_nested(_robo.jointsCount()),
        _breakings_controls(_max_nested), /// ???
        pDP{ std::make_shared<RoboPos::DirectionPredictor>(_robo) }
    {
        _lasts_step_increment_thick = 20;
        _lasts_step_increment_init = 25;
        _lasts_step_braking_incr = 5;
        _lasts_step_braking_init = 5;
    }
    //------------------------------------------------------------------------------
    void  run(bool distance, bool target, bool braking, bool predict, bool checking,
              borders_t &borders, double step_distance, Robo::frames_t lasts_step_increment)
    {
        Tour::run(distance, target, braking, predict, checking, borders, step_distance, lasts_step_increment);
    }
};

}
//------------------------------------------------------------------------------