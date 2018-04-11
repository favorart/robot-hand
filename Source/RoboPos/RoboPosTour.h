#pragma once

#include "Robo.h"
#include "RoboPos.h"
#include "RoboPosMDirections.h"
namespace RoboPos {

//------------------------------------------------------------------------------
class Tour
{
protected:
    Robo::RoboI &_robo;
    Point _base_pos;
    RoboMoves::Store &_store;
    RecTarget &_target;
    borders_t _borders;
    RoboPos::NewHand::MainDirections _md;
    Counters _counters;
    size_t _complexity = 0U;
    bool _end = false;

    // --- flags ---
    bool  _b_distance, _b_target, _b_braking, _b_checking;

    // --- distance --- 
    double _target_distance;
    //Point _pd_shift{ 0.15, -0.05 };

    double               _step_distance;
    Robo::frames_t _lasts_step_increment;
    Robo::frames_t _lasts_step_increment_thick = 20U;
    Robo::frames_t _lasts_step_initiate = 25U;
    Robo::frames_t _lasts_step_braking = 5U;

public:
    Tour(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target/*, IN const tstring &config=_T("")*/) :
        _store(store), _robo(robo), _target(target),
        _md(NewHand::MainDirectionsFactory(robo)),
        _target_distance((_target.max().x - _target.min().x) * 2)
    {
        _robo.reset();
        _base_pos = _robo.position();
    }

    bool end() const { return _end; }
    //------------------------------------------------------------------------------
    virtual void  run(IN bool distance, IN bool target, IN bool braking, IN bool checking,
                      IN double step_distance, IN Robo::frames_t lasts_step_increment,
                      IN bool verbose = false)
    {
        _b_target = target;
        _b_braking = braking;
        _b_distance = distance;
        _b_checking = checking;
        // ----------------------------------------------------
        _step_distance = step_distance;
        _lasts_step_increment = lasts_step_increment;
        // ----------------------------------------------------
        _complexity = 0U;
        _counters.clear();
    }
    //------------------------------------------------------------------------------
    virtual void  STAGE_1(IN bool verbose) = 0;
    virtual void  STAGE_2(IN bool verbose) = 0;
};

//------------------------------------------------------------------------------
class TourWorkSpace : public Tour
{
    size_t _max_nested;
    std::vector<Robo::Actuator> _arr_controlings; ///< brakings

    /// Descrete tour around all over the workspace
    /// \param[in]   joint          focus of moving on the next joint
    /// \param[in]   controls       result control to robo-move
    /// \param[out]  robo_pos_high  best _hit_ using the next joint
    bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high);
    /// Straight move by robot with chosen muscles
    /// \param[in]   controls       result control to robo-move
    /// \param[out]  robo_pos       best _hit_ using the next joint
    bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos);

public:
    TourWorkSpace(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target) :
        Tour(store, robo, target),
        _max_nested(robo.jointsCount()),
        _arr_controlings(_max_nested)
    {}
    //------------------------------------------------------------------------------
    void  run(IN bool distance, IN bool target, IN bool braking, IN bool checking,
              IN double step_distance, IN Robo::frames_t lasts_step_increment, IN bool verbose = false)
    {
        Tour::run(distance, target, braking, checking, step_distance, lasts_step_increment, verbose);
        // ----------------------------------------------------
        try
        {
            // log_fout.open ("out.txt");
            // log_fout.precision (4);
            Point useless;
            runNestedForMuscle(0, Robo::Control{}, useless);
            // log_fout.close ();
        }
        catch (boost::thread_interrupted&)
        {
            /* tcout << _T("WorkingThread interrupted") << std::endl; */
            return;
        } // end catch
        
        if (verbose)
        {
            if (checking) { _counters.print(); }
            tcout << _T("\nStep: ") << (step_distance / 0.0028) << _T("mm.");
            tcout << _T("\nComplexity: ") << _complexity;
            tcout << _T("  minutes:") << double(_complexity) / 60. << std::endl;
        }
    }
    //------------------------------------------------------------------------------
    void  STAGE_1(IN bool verbose)
    {
        /* грубое покрытие всего рабочего пространства */
        defineRobotBorders(_robo, 70U/*25U*/, _borders);
        /* mm :
             (target.max - target.min) = 300 mm
             1
             0.84 <--> 300 mm
             x    <-->   1 mm   ==> 0.0028
        */

        run(/* distance */  true,
            /* target   */ false,
            /* braking  */  true,
            /* checking */ false,
            0.07/*0.1*/, 5U/*1U*/,
            verbose);
    }
    void  STAGE_2(IN bool verbose)
    {
        /* Покрытие всей мишени не слишком плотно */
        defineTargetBorders(_target, _store, /* side */ 0.05, _borders); // indices_

        run(/* distance */ false,
            /* target   */ true,
            /* braking  */ true,
            /* checking */ false,
            0.015/*0.02*/, 2U/*3U*/,
            verbose);
    }
};

//------------------------------------------------------------------------------
class TourNoRecursion : public Tour
{
    size_t _max_nested;

    bool  runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &pos_high);
    bool  runNestedMove(IN Robo::Control &controls, OUT Point &robo_pos);

public:
    TourNoRecursion(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target) :
        Tour(store, robo, target), _max_nested(_robo.jointsCount())
    {}
    void  run(IN bool distance, IN bool target, IN bool braking, IN bool checking,
              IN double step_distance, IN Robo::frames_t lasts_step_increment, IN bool verbose = false)
    {
        Tour::run(distance, target, braking, checking, step_distance, lasts_step_increment, verbose);
        // ----------------------------------------------------
        try
        {
            Point useless;
            runNestedForMuscle(0, Robo::Control{}, useless);
        }
        catch (boost::thread_interrupted&)
        {
            tcout << _T("WorkingThread interrupted") << std::endl;
            return;
        }
        // ----------------------------------------------------
        if (verbose)
        {
            tcout << _T("\nStep: ") << int(step_distance / 0.0028 + 1);
            tcout << _T("\nComplexity: ") << _complexity << std::endl;
        }
    }

    /* грубое покрытие всего рабочего пространства */
    void  STAGE_1(IN bool verbose)
    {
        defineRobotBorders(_robo, 70U, _borders); // indices_

        run(/* distance */  true,
            /* target   */ false,
            /* braking  */  true,
            /* checking */ false,
            0.03, 3U,
            verbose);
    }
    /* Покрытие всей мишени не слишком плотно */
    void  STAGE_2(IN bool verbose)
    {
        defineTargetBorders(_target, _store, /* side */ 0.05, _borders); // indices_

        run(/* distance */ false,
            /* target   */ true,
            /* braking  */ true,
            /* checking */ false,
            0.015, 2U,
            verbose);
    }
};

}
//------------------------------------------------------------------------------