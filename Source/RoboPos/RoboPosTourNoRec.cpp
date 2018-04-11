#include "StdAfx.h"
#include "Robo.h"
#include "RoboPos.h"
#include "RoboPosTour.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
using namespace Robo::Mobile;
using namespace Robo::NewHand;
//------------------------------------------------------------------------------
bool TourNoRecursion::runNestedForMuscle(IN joint_t joint, IN Control &controls, OUT Point &pos_high)
{
    bool  target_contain = true;
    //------------------------------------------
    frames_t start_i = 0U;
    // frames_t last_lasts = 0U;
    frames_t lasts_step; // = lasts_step_initiate;

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
        //------------------------------------------
        auto &board = _borders[muscle_i];
        if (board.first > 0
            && board.first < board.second)
        {
            lasts_step = _lasts_step_increment;
            frames_t lasts_i_max = _robo.muscleMaxLast(muscle_i);
            //------------------------------------------
            start_i = 0U;
            target_contain = true;
            //------------------------------------------
            frames_t last_i = 0U;
            for (last_i = board.first;
                (last_i < lasts_i_max) && (target_contain || !was_on_target) /* &&
                (last_i <= board.second - start_i || target_contain) */;
                 last_i += lasts_step)
            {
                if ((last_i > board.second - start_i) && ((!_b_target || !target_contain) ||
                     prev_pos.x > _target.max().x || prev_pos.y < _target.min().y))
                {
                    break;
                }
                //------------------------------------------
                if (0 <= joint && (joint + 1U) < _max_nested)
                {
                    //===============================================================
                    target_contain = runNestedForMuscle(joint + 1, controls + control_i, curr_pos) || !_b_target;
                    //===============================================================
                }
                else
                {
                    // if ( b_braking )
                    // { auto it = controls.begin ();
                    //   for ( size_t ji = 0; ji <= joint; ++ji, ++it )
                    //     if ( arr_controlings[ji].last )
                    //       arr_controlings[ji].start = it->last + 1U;
                    // }
                    //===============================================================
                    target_contain = runNestedMove(controls + control_i, curr_pos) || !_b_target;
                    //===============================================================              
                } // end else (insert)

                  //------------------------------------------
                if (_b_target && (target_contain && !was_on_target))
                {
                    board.first = last_i;
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
                    // else if ( b_braking && (lasts_step_increment == 1U) )
                    // {
                    //   auto it = controls.begin ();
                    //   for ( size_t ji = 0; ji <= joint; ++ji, ++it )
                    //   {
                    //     auto opposite_muscle = muscleOpposite (it->muscle);
                    //     auto prev_last = arr_controlings[ji].last;
                    //     arr_controlings[ji] = (Control (opposite_muscle, it->last + 1,
                    //                                           (prev_last ? prev_last : 30) +
                    //                                           lasts_step_braking));
                    //   }
                    // }
                }
                else if (d < _step_distance)
                {
                    // if ( arr_controlings[0].last )
                    // {
                    //   if ( arr_controlings[0].last > lasts_step_braking )
                    //     for ( size_t ji = 0; ji <= joint; ++ji )
                    //       arr_controlings[ji].last -= lasts_step_braking;
                    // }
                    // else
                    { lasts_step += _lasts_step_increment; }
                }
                //------------------------------------------
                prev_pos = curr_pos;
                //------------------------------------------
            } // end for (last)
              //------------------------------------------
            if (_b_target) { board.second = last_i; }
            //------------------------------------------
            // for ( size_t ji = 0; ji <= joint; ++ji )
            // { arr_controlings[ji].last = 0U; }
            //------------------------------------------
            if (_b_target)
            {
                start_i = 0U;
                target_contain = true;
                //------------------------------------------
                // frames_t lasts_step_prev = lasts_step;
                lasts_step = _lasts_step_increment_thick; // lasts_step_initiate;
                //------------------------------------------
                for (last_i = board.first;
                    (last_i - lasts_step) < lasts_i_max && target_contain;
                     last_i -= lasts_step)
                {
                    control_i.last = last_i;
                    //------------------------------------------
                    if (0U <= joint && (joint + 1U) < _max_nested)
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
                } // end for (lasts)
                  //------------------------------------------
                  // lasts_step = lasts_step_prev;
            } // if (target)
        } // end if (border)
    } // end for (muscle)
      //------------------------------------------
    if ( /*hand_pos_high &&*/ high)
    {
        pos_high = Point{ max_hand_pos_high.x / high,
            max_hand_pos_high.y / high };
    }
    //------------------------------------------
    return true;
}

//------------------------------------------------------------------------------
bool TourNoRecursion::runNestedMove(IN Control &controls, OUT Point &robo_pos)
{
    // const Record &rec = store.ClothestPoint (robo_pos, 0.1);
    // if ( boost_distance (rec.hit, robo_pos) < step_distance )
    // {
    //   robo_pos = rec.hit;
    //   return target.contain (robo_pos);
    // }

    bool target_contain = true;
    Point predEnd = _base_pos;
    //----------------------------------------------
    // bool  has_braking = b_braking
    //   && boost::algorithm::any_of (arr_controlings,
    //                                [](const Control &item)
    //                                { return  item.last; });
    //----------------------------------------------
    // Сontrol  new_controling;
    // if ( has_braking )
    // {
    //   for ( const Control &c : controls )
    //   { new_controling.push_back (c); }
    //   /* записываем все разрывы */
    //   for ( auto &c : arr_controlings )
    //   { new_controling.push_back (c); }
    //   new_controling.sort ();
    // }
    // Сontrol  &controling = (has_braking) ? new_controling : controls;
    //----------------------------------------------
    if (_b_target || _b_distance)
    {
        for (const auto &a : controls)
        { predEnd = _md.predict({ a }); }
        //----------------------------------------------
        // if ( b_distance && b_target )
        // { end.x += predict_shift.x;
        //   end.y += predict_shift.y;
        // }
    }
    //----------------------------------------------
    if ((!_b_target || _target.contain(predEnd)) &&
        (!_b_distance || boost_distance(_target.center(), predEnd) < _target_distance))
    {
        /* ближе к мишени */
        Trajectory trajectory;
        //----------------------------------------------
        _robo.reset();
        /* двигаем рукой */
        _robo.move(controls, trajectory);
        // ++complexity; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //----------------------------------------------
        robo_pos = _robo.position();
        target_contain = _target.contain(robo_pos);
        //----------------------------------------------
        Record rec(robo_pos, _base_pos, robo_pos, controls, trajectory);
        _store.insert(rec);
        //----------------------------------------------
    } /* end if */
    else
    {
        if (!_b_target)
        { robo_pos = predEnd; }
        target_contain = false;
    }
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return  target_contain;
}

