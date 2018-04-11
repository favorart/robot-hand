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
bool TourWorkSpace::runNestedForMuscle(IN joint_t joint, IN Control &controls, OUT Point &robo_pos_high /* std::vector<Point> *robo_pos_highs */)
{
    bool  target_contain = true;
    //------------------------------------------
    frames_t start_i = 0U;
    // frames_t last_lasts = 0U;
    frames_t lasts_step; // = lasts_step_initiate;
    //------------------------------------------
    bool was_on_target = false;
    //------------------------------------------
    Point curr_pos = _base_pos, prev_pos = _base_pos;
    //------------------------------------------
    int   high = 0;
    Point max_robo_pos_high{ 0., 0. };
    //------------------------------------------
    //std::vector<Point>  curr_positions;
    //std::vector<Point>  prev_positions;
    //
    //auto m = _roboI::muscleByJoint(joint, true);
    //curr_positions.reserve(2 * _robo.muscleMaxLast(m));
    //prev_positions.reserve(2 * _robo.muscleMaxLast(m));
    //------------------------------------------
    Actuator control_i;
    //------------------------------------------
    for (auto muscle_i : { RoboI::muscleByJoint(joint, true),
                           RoboI::muscleByJoint(joint, false) })
    {
        control_i.muscle = muscle_i;
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
                (last_i <= board.second - start_i || _target_contain) */;
                 last_i += lasts_step)
            {
                control_i.last = last_i;
                //------------------------------------------
                if ((last_i > board.second - start_i) && ((!_b_target || !target_contain) ||
                    (prev_pos.x > _target.max().x) || (prev_pos.y < _target.min().y)))
                {
                    break;
                }
                //------------------------------------------
                //double _d;
                //------------------------------------------
                if (0U <= joint && (joint + 1U) < _max_nested)
                {
                    //===============================================================
                    target_contain = runNestedForMuscle(joint + 1, controls + control_i, curr_pos) || !_b_target;
                    //===============================================================
                    //if (!prev_positions.empty())
                    //{
                    //    _d = 0.;
                    //    size_t count = 0U;
                    //
                    //    auto itc = curr_positions.begin();
                    //    auto itp = prev_positions.begin();
                    //    for (; itc != curr_positions.end() && itp != prev_positions.end(); ++itc, ++itp)
                    //    {
                    //        d += boost_distance(*itc, *itp);
                    //        ++count;
                    //    }
                    //    _d /= (count);
                    //    // _d *= 2.8;
                    //}
                    //else
                    //{ _d = _step_distance; }
                    //std::swap(curr_positions, prev_positions);
                    //curr_positions.clear();
                }
                else
                {
                    if (_b_braking)
                    {
                        auto it = controls.begin();
                        for (size_t ji = 0; ji <= joint; ++ji, ++it)
                            if (_arr_controlings[ji].last)
                                _arr_controlings[ji].start = it->last + 1U;
                    }
                    //===============================================================
                    target_contain = runNestedMove(controls + control_i, curr_pos) || !_b_target;
                    //===============================================================
                    //if (robo_pos_highs)
                    //{ robo_pos_highs->push_back(curr_pos); }
                    //_d = boost_distance(prev_pos, curr_pos);
                } // end else (insert)

                //------------------------------------------
                if (_b_target && (target_contain && !was_on_target))
                {
                    board.first = last_i;
                    was_on_target = true;
                }
                //------------------------------------------
                {
                    max_robo_pos_high.x += curr_pos.x;
                    max_robo_pos_high.y += curr_pos.y;
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
                        for (joint_t ji = 0; ji <= joint; ++ji, ++it)
                        {
                            auto opposite_muscle = RoboI::muscleOpposite(it->muscle);
                            auto prev_last = _arr_controlings[ji].last;
                            _arr_controlings[ji] = { opposite_muscle, it->last + 1, (prev_last ? prev_last : 30) + _lasts_step_braking };
                        }
                    }
                }
                else if (d < _step_distance)
                {
                    if (_arr_controlings[0].last)
                    {
                        if (_arr_controlings[0].last > _lasts_step_braking)
                            for (joint_t ji = 0; ji <= joint; ++ji)
                                _arr_controlings[ji].last -= _lasts_step_braking;
                    }
                    else
                    { lasts_step += _lasts_step_increment; }
                }
                //------------------------------------------
                prev_pos = curr_pos;
                //------------------------------------------
            } // end for (last)
            //------------------------------------------
            if (_b_target) { board.second = last_i; }
            //------------------------------------------
            for (size_t ji = 0; ji <= joint; ++ji)
            { _arr_controlings[ji].last = 0U; }
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
            } // if (_target)
        } // end if (border)
    } // end for (muscle)
    //------------------------------------------
    if ( /*robo_pos_high &&*/ high)
    {
        robo_pos_high = Point{ max_robo_pos_high.x / high,
                               max_robo_pos_high.y / high };
    }
    //------------------------------------------
    return true;
}

//------------------------------------------------------------------------------
bool TourWorkSpace::runNestedMove(IN const Control &controls, OUT Point &robo_pos)
{
    // const Record &rec = store.ClothestPoint (_robo_position, 0.1);
    // if ( boost_distance (rec.hit, _robo_position) < _step_distance )
    // {
    //   _robo_position = rec.hit;
    //   return _target.contain (_robo_position);
    // }

    bool  _target_contain = true;
    Point predEnd = _base_pos;
    //----------------------------------------------
    bool  has_braking = _b_braking && ba::any_of(_arr_controlings, [](const Control &item) { return item[0].last; });
    //----------------------------------------------
    Control controling = has_braking ? controls : controls + _arr_controlings; // already sorted
    //----------------------------------------------
    if (_b_target || _b_distance)
    {
        for (const auto &a : controls)
        { predEnd = _md.predict({ a }); }
        //----------------------------------------------
        // if ( b_distance && _b_target )
        // { end.x += predict_shift.x;
        //   end.y += predict_shift.y;
        // }
        //----------------------------------------------
        if (_b_checking)
        { _counters.fill(_robo, _target, controling, predEnd); }
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
        _robo.move(controling, trajectory);
        ++_complexity;
        //----------------------------------------------
        robo_pos = _robo.position();
        _target_contain = _target.contain(robo_pos);
        //----------------------------------------------
        Record rec(robo_pos, _base_pos, robo_pos, controling, trajectory);
        _store.insert(rec);
        //----------------------------------------------
    } /* end if */
    else
    {
        if (!_b_target)
        { robo_pos = predEnd; }
        _target_contain = false;
    }
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return _target_contain;
}

