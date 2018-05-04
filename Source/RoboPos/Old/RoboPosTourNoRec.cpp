﻿#include "StdAfx.h"
#include "Robo.h"
#include "RoboPos.h"
#include "RoboPosTour.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
using namespace Robo::Mobile;
using namespace Robo::NewHand;
//------------------------------------------------------------------------------

//DirectionPredictor  *pDP;
//Point  predict_shift{ 0.15, -0.05 };
////--------------------------------------------------------
//double                 step_distance;
//frames_t   _lasts_step_increment;
//frames_t   _lasts_step_increment_thick = 20U;
//frames_t   _lasts_step_initiate = 25U;
//frames_t   _lasts_step_braking = 5U;

//------------------------------------------------------------------------------
bool TourNoRecursion::runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high)
{
    bool  target_contain = true;
    //------------------------------------------
    frames_t start_i = 0U;
    // frames_t last_lasts = 0U;
    frames_t lasts_step; // = _lasts_step_increment_init;
    
    //------------------------------------------
    bool  was_on_target = false;
    //------------------------------------------
    Point curr_pos = _base_pos, prev_pos = _base_pos;
    //------------------------------------------
    int     high = 0;
    Point   max_hand_pos_high{ 0., 0. };
    //------------------------------------------
    Actuator control_i;
    //------------------------------------------
    for (auto muscle_i : { RoboI::muscleByJoint(joint, true),
                           RoboI::muscleByJoint(joint, false) })
    {
        control_i.muscle = muscle_i;
        //------------------------------------------
        auto &board = _borders[muscle_i];
        if (board.min_lasts == 0 && board.min_lasts >= board.max_lasts)
        {}
        {
            lasts_step = _lasts_step_increment;
            frames_t _lasts_i_max = _robo.muscleMaxLast(muscle_i);
            //------------------------------------------
            start_i = 0U;
            target_contain = true;
            //------------------------------------------
            frames_t last_i = 0U;
            for (last_i = board.min_lasts;
                (last_i <  _lasts_i_max) && (target_contain || !was_on_target);
                 last_i += lasts_step)
            {
                control_i.lasts = last_i;

                if ((last_i > board.max_lasts - start_i) &&
                    (!_b_target || !target_contain ||
                     prev_pos.x > _target.max().x || prev_pos.y < _target.min().y))
                {
                    break;
                }
                //------------------------------------------
                if (0U <= joint && (joint + 1u) < _max_nested)
                {
                    //===============================================================
                    target_contain = runNestedForMuscle(joint + 1, controls + control_i, curr_pos) || !_b_target;
                    //===============================================================
                }
                else
                {
                    if (_b_braking)
                    {
                        auto it = controls.begin();
                        for (auto ji = 0; ji <= joint; ++ji, ++it)
                            if (_breakings_controls[ji].lasts)
                                _breakings_controls[ji].start = it->lasts + 1U;
                    }
                    //===============================================================
                    target_contain = runNestedMove(controls + control_i, curr_pos) || !_b_target;
                    //===============================================================              
                } // end else (insert)

                  //------------------------------------------
                if (_b_target && (target_contain && !was_on_target))
                {
                    board.min_lasts = last_i;
                    was_on_target = true;
                }
                //------------------------------------------
                {
                    max_hand_pos_high.x += curr_pos.x;
                    max_hand_pos_high.y += curr_pos.y;
                    ++high;
                }
                //------------------------------------------
                double  d = boost_distance(prev_pos, curr_pos);
                //------------------------------------------
                if (d > _step_distance)
                {
                    if (lasts_step >= 2 * _lasts_step_increment)
                        lasts_step -= _lasts_step_increment;
                    else if (lasts_step > _lasts_step_increment)
                        lasts_step -= _lasts_step_increment;
                    else if ((_lasts_step_increment > 1) && (lasts_step > _lasts_step_increment / 2))
                        lasts_step -= _lasts_step_increment / 2;
                    else if (_b_braking && (_lasts_step_increment == 1U))
                    {
                        auto it = controls.begin();
                        for (auto ji = 0; ji <= joint; ++ji, ++it)
                        {
                            auto opposite_muscle = RoboI::muscleOpposite(it->muscle);
                            auto prev_last = _breakings_controls[ji].lasts;
                            _breakings_controls[ji] = { opposite_muscle, it->lasts + 1, (prev_last ? prev_last : 30) + _lasts_step_braking_incr };
                        }
                    }
                }
                else if (d < _step_distance)
                {
                    if (_breakings_controls[0].lasts)
                    {
                        if (_breakings_controls[0].lasts > _lasts_step_braking_incr)
                            for (auto ji = 0; ji <= joint; ++ji)
                                _breakings_controls[ji].lasts -= _lasts_step_braking_incr;
                    }
                    else
                    { lasts_step += _lasts_step_increment; }
                }
                //------------------------------------------
                prev_pos = curr_pos;
                //------------------------------------------
            } // end for (last)
              //------------------------------------------
            if (_b_target) { board.max_lasts = last_i; }
            //------------------------------------------
            for (auto ji = 0; ji <= joint; ++ji)
            { _breakings_controls[ji].lasts = 0U; }
            //------------------------------------------
            if (_b_target)
            {
                start_i = 0U;
                target_contain = true;
                //------------------------------------------
                // frames_t _lasts_step_prev = _lasts_step;
                lasts_step = _lasts_step_increment_thick; // _lasts_step_initiate;
                
                //------------------------------------------
                for (last_i = board.min_lasts;
                    (last_i - lasts_step) < _lasts_i_max && target_contain;
                     last_i -= lasts_step)
                {
                    control_i.lasts = last_i;
                    //------------------------------------------
                    if (0u <= joint && (joint + 1u) < _max_nested)
                    {
                        //===============================================================
                        target_contain = runNestedForMuscle(joint + 1, controls + control_i, curr_pos) || !_b_target;
                        //===============================================================
                    }
                    else
                    {
                        //===============================================================
                        target_contain = runNestedMove(controls + control_i, curr_pos) || !_b_target;
                        //===============================================================
                    } // end (insert)
                      //------------------------------------------
                    double  d = boost_distance(prev_pos, curr_pos);
                    //------------------------------------------
                    if (d > _step_distance)
                    {
                        if (lasts_step > _lasts_step_increment_thick)
                            lasts_step -= _lasts_step_increment_thick;
                    }
                    else if (d < _step_distance)
                    { lasts_step += _lasts_step_increment_thick; }
                    //-----------------------------
                    prev_pos = curr_pos;
                } // end for (_lasts)
                  //------------------------------------------
                  // _lasts_step = _lasts_step_prev;
            } // if (_target)
        } // end if (border)
    } // end for (muscle)
    //------------------------------------------
    if (high)
    {
        robo_pos_high = Point{ max_hand_pos_high.x / high,
                               max_hand_pos_high.y / high };
    }
    //------------------------------------------
    return true;
}


bool TourNoRecursion::runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos)
{
    // const Record &rec = _store.ClothestPoint (hand_position, 0.1);
    // if ( boost_distance (rec.hit, hand_position) < step_distance )
    // {
    //   hand_position = rec.hit;
    //   return _target.contain (hand_position);
    // }

    Point end = _base_pos;
    //----------------------------------------------
    Control controling = controls + _breakings_controls;
    //----------------------------------------------
    if (_b_target || _b_distance)
    {
        end = pDP->predict(controls);
        //----------------------------------------------
        // if ( _b_distance && _b_target )
        // { end.x += predict_shift.x;
        //   end.y += predict_shift.y;
        // }
        //----------------------------------------------
        if (_b_checking)
        { _counters.fill(_robo, _target, controling, end); }
    }
    //----------------------------------------------
    if ((!_b_target || _target.contain(end)) && (!_b_distance || boost_distance(_target.center(), end) < _target_distance))
    {
        /* ближе к мишени */
        Trajectory trajectory;
        //----------------------------------------------
        _robo.reset();
        /* двигаем рукой */
        _robo.move(controling, trajectory);
        ++_complexity;
        robo_pos = _robo.position();
        //----------------------------------------------
        Record  rec(robo_pos, _base_pos, robo_pos, controling, trajectory);
        _store.insert(rec);
        //----------------------------------------------
    }
    else if (!_b_target) { robo_pos = end; }
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return _target.contain(robo_pos);
}
