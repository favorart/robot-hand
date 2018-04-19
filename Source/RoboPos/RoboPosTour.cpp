#include "StdAfx.h"
#include "Robo.h"
#include "RoboPos.h"
#include "RoboPosTour.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
using namespace Robo::Mobile;
using namespace Robo::NewHand;

frames_t glob_lasts_step;
//------------------------------------------------------------------------------
void TourWorkSpace::appendBreakings(IN joint_t joint, IN const Actuator &a)
{
    if (_b_braking)
    {
        // добавляем торможения, если исчерпали изменения длительности
        if (_breakings_controls[joint].muscle != MInvalid && _breakings_controls[joint].last > 0)
        {
            // увеличиваем длительность
            _breakings_controls[joint].last += _lasts_step_braking_incr;
        }
        else
        {
            //CDEBUG("add breaks on last_step=" << glob_lasts_step << 
            //       " incr=" << _lasts_step_increment << " for joint=" << joint);
            // добавляем торможения
            auto opposite_muscle = RoboI::muscleOpposite(a.muscle);
            _breakings_controls[joint] = { opposite_muscle,
                                          /*start=*/(a.start + a.last + 1),
                                          _lasts_step_braking_init };
            _breakings_controls_actives++;
        }
        //auto it = controls.begin();
        //for (joint_t ji = 0; ji <= joint; ++ji, ++it)
        //{
        //    if (_brakings_controls[ji].muscle != it->muscle)
        //        CWARN("_brakings_controls[ji].muscle=" <<
        //              _brakings_controls[ji].muscle << " != " <<
        //              "Opposite(controls[ji].muscle)=" <<
        //              RoboI::muscleOpposite(it->muscle));
        //
        //    if (_brakings_controls[ji].muscle != MInvalid && _brakings_controls[ji].last > 0)
        //    {
        //        // увеличиваем длительность
        //        _brakings_controls[ji].last += _lasts_step_braking;
        //    }
        //    else
        //    {
        //        CDEBUG("add breaks " << _lasts_step_increment);
        //        // добавляем торможения
        //        auto opposite_muscle = RoboI::muscleOpposite(it->muscle);
        //        _brakings_controls[ji] = { opposite_muscle, /*start=*/it->last + 1, _lasts_step_braking_init };
        //        _brakings_controls_actives++;
        //    }
        //}
    }
}

//------------------------------------------------------------------------------
void TourWorkSpace::removeBreakings(IN joint_t joint)
{
    if (_b_braking)
    {
        if (_breakings_controls[joint].last > _lasts_step_braking_incr)
            _breakings_controls[joint].last -= _lasts_step_braking_incr;
        else
        {
            _breakings_controls[joint] = { MInvalid, 0, 0 };
            _breakings_controls_actives--;
        }
        //for (joint_t ji = 0; ji <= joint; ++ji)
        //    if (_brakings_controls[ji].last > _lasts_step_braking_incr)
        //    {
        //        _brakings_controls[ji].last -= _lasts_step_braking_incr;
        //    }
        //    else
        //    {
        //        _brakings_controls[ji] = { MInvalid, 0, 0 };
        //        _brakings_controls_actives--;
        //    }
    }
}

//------------------------------------------------------------------------------
void TourWorkSpace::cleanBreakings(IN joint_t joint)
{
    // удалить торможения, задействованные в данной вложенной процедуре
    _breakings_controls[joint] = { MInvalid, 0, 0 };
    //for (size_t ji = 0; ji <= joint; ++ji)
    //    _breakings_controls[ji].last = 0;
    //------------------------------------------
}

//------------------------------------------------------------------------------
void TourWorkSpace::exactsBreakings(IN joint_t joint, IN const Control &controls)
{
    if (_b_braking && _breakings_controls_actives > 0)
    {
        auto it = controls.begin();
        for (joint_t ji = 0; ji <= joint; ++ji, ++it)
        {
            if (_breakings_controls[ji].muscle != MInvalid &&
                _breakings_controls[ji].muscle != RoboI::muscleOpposite(it->muscle))
                CWARN("_brakings_controls[ji].muscle=" <<
                      _breakings_controls[ji].muscle << " != " <<
                      "Opposite(controls[ji].muscle)=" <<
                      RoboI::muscleOpposite(it->muscle));
            // просто уточняем начало торможений, если что-то изменилось
            if (_breakings_controls[ji].muscle != MInvalid && _breakings_controls[ji].last > 0)
                _breakings_controls[ji].start = (it->start + it->last + 1);
        }
    }
}

//------------------------------------------------------------------------------
bool TourWorkSpace::runNestedForMuscle(IN joint_t joint, IN Control &controls, OUT Point &robo_pos_high)
{
    // стартруем разными сочленениями последовательно
    const frames_t start_i = controls.size() ? (controls[-1].last + controls[-1].start) : 0;
    // стартруем разными сочленениями одновременно
    //const frames_t start_i = 0;
    //------------------------------------------
    Point curr_pos = _base_pos, prev_pos = _base_pos;
    //------------------------------------------
    // заметание пространства проходит рядами,
    // сдвигаясь всё дальше от начальной точки
    int high = 0; // число положений захвата робота в вышестоящем ряду
    Point max_robo_pos_high{ 0., 0. }; // сумма положений робота в вышестоящем ряду
    // robo_pos_high - среднее положение в вышестоящем ряду = (max_robo_pos_high / high)

    //------------------------------------------
    // в случае, если сохранять не среднее, а весь вышестоящий ряд
    //
    //std::vector<Point> *robo_pos_highs
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
        bool was_on_target = false; // положение захвата ещё не достигло мишени в этом ряду
        bool target_contain = true; // положение захвата достигло мишени и остаётся на ней
        
        //------------------------------------------
        auto &board = _borders[muscle_i];
        if (board.min_lasts == 0 && board.min_lasts >= board.max_lasts)
        {
            CWARN("Empty borders of muscle=" << muscle_i);
            return false;
        }

        {
            //------------------------------------------
            frames_t lasts_step = _lasts_step_increment_init; // адаптивный шаг длительности
            frames_t lasts_i_max = _robo.muscleMaxLast(muscle_i);
            //------------------------------------------
            frames_t last_i = 0;
            for (last_i = board.min_lasts; last_i < lasts_i_max && (target_contain || !was_on_target); last_i += lasts_step)
            {
                control_i.last = last_i;
                //------------------------------------------
                if ((last_i > board.max_lasts) && (!_b_target || !target_contain ||
                    (prev_pos.x > _target.max().x) || (prev_pos.y < _target.min().y)))
                {
                    break;
                }
                //------------------------------------------
                //double _d;
                //------------------------------------------
                if ((joint + 1U) < _max_nested)
                {
                    //===============================================================
                    target_contain = runNestedForMuscle(joint + 1, controls + control_i, curr_pos) || !_b_target;
                    //===============================================================

                    //------------------------------------------
                    // в случае, если сохранять не среднее, а весь вышестоящий ряд
                    //
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
                    exactsBreakings(joint, controls + control_i);
                    //===============================================================
                    target_contain = runNestedMove(controls + control_i, curr_pos) || !_b_target;
                    //===============================================================

                    //------------------------------------------
                    // в случае, если сохранять не среднее, а весь вышестоящий ряд
                    //
                    //if (robo_pos_highs)
                    //{ robo_pos_highs->push_back(curr_pos); }
                    //_d = boost_distance(prev_pos, curr_pos);
                } // end else (insert)

                //------------------------------------------
                if (_b_target && target_contain && !was_on_target)
                {
                    board.min_lasts = last_i;
                    was_on_target = true;
                }
                //------------------------------------------
                {
                    // сохраняем полученное положение робота
                    // в сумму для получения среднего, которым пользуется
                    // функция выше по реккурсии
                    max_robo_pos_high.x += curr_pos.x;
                    max_robo_pos_high.y += curr_pos.y;
                    ++high;
                }
                //------------------------------------------
                double d = boost_distance(prev_pos, curr_pos);
                //------------------------------------------
                if (d > _step_distance)
                {
                    // адаптивный шаг изменения длительности
                    if (lasts_step > _lasts_step_increment)
                        lasts_step -= _lasts_step_increment;
                    else if (_lasts_step_increment > 1 && lasts_step > _lasts_step_increment / 2)
                        lasts_step -= _lasts_step_increment / 2;
                    else if (_b_braking) // ?? && _lasts_step_increment == 1)
                    {
                        glob_lasts_step = lasts_step; /// DELETE
                        // если нельзя сохранить одинаковый промежуток уменьшением длительности
                        // подключаем торможения противоположным двигателем
                        appendBreakings(joint, control_i);
                    }
                }
                else if (d < _step_distance)
                {
                    if (_b_braking && _breakings_controls_actives > 0 && 
                        _breakings_controls[joint].last > 0 /* !!! */)
                    {
                        // сначала по возможности отключаем торможения
                        removeBreakings(joint);
                    }
                    else
                    {
                        // затем увеличиваем длительность
                        lasts_step += _lasts_step_increment;
                    }
                }
                //------------------------------------------
                prev_pos = curr_pos;
                //------------------------------------------
            } // end for (lasts)

            //-----------------------------
            cleanBreakings(joint);
            //-----------------------------

            if (_b_target)
            {
                //target_contain = true;
                board.max_lasts = last_i;
                //------------------------------------------
                lasts_step = _lasts_step_increment_thick; // lasts_step_initiate;
                //------------------------------------------
                for (last_i = board.min_lasts; (last_i - lasts_step) < lasts_i_max && (target_contain); last_i -= lasts_step)
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
                    double d = boost_distance(prev_pos, curr_pos);
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
                } // end for (_b_target lasts)
            } // end if (_b_target)
        }
    } // end for (muscle)
    //------------------------------------------
    if (high)
    {
        robo_pos_high = Point{ max_robo_pos_high.x / high,
                               max_robo_pos_high.y / high };
    }
    //------------------------------------------
    return true;
}

#include "WindowData.h"
//class MyWindowData;
std::list<Point> MyWindowData::predicts = {};
std::list<Point> MyWindowData::reals = {};

void RoboPos::Counters::fill(Robo::RoboI &robo, RecTarget &target, Robo::Control &controls, const Point &pred)
{
    Robo::Trajectory trajectory;
    //------------------------------------------
    robo.reset();
    robo.move(controls, trajectory);
    //------------------------------------------
    bool model = target.contain(pred);
    bool real = target.contain(robo.position());
    //------------------------------------------
    incr(model, real);
    avg_miss += boost_distance(pred, robo.position());

    if (model != real)
    {
        MyWindowData::predicts.push_back(pred);
        MyWindowData::reals.push_back(robo.position());
    }
    //------------------------------------------
    robo.reset();
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

    Point pred_end = _base_pos;
    //----------------------------------------------
    // (_b_braking && _breakings_controls_actives)
    Control controling = controls + _breakings_controls;
    //----------------------------------------------
    ///if (_b_target || _b_distance)
    ///{
    ///    //for (const auto &a : controls)
    ///    //    pred_end += _md.predict(a.muscle, a.last);
    ///    pred_end = _md.predict(controls);
    ///
    ///    if (_b_checking)
    ///        _counters.fill(_robo, _target, controling, pred_end);
    ///}
    /////----------------------------------------------
    ///if ((!_b_target || _target.contain(pred_end)) &&
    ///    (!_b_distance || boost_distance(_target.center(), pred_end) <= _target_distance))
    {
        /* ближе к мишени */
        Trajectory trajectory;
        //----------------------------------------------
        /* двигаем рукой */
        _robo.move(controling, trajectory);
        ++_complexity;
        robo_pos = _robo.position();
        //----------------------------------------------
        Record rec(robo_pos, _base_pos, robo_pos, controling, trajectory);
        _store.insert(rec);
        //----------------------------------------------
        _robo.reset();
    }
    ///else robo_pos = pred_end;
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return _target.contain(robo_pos);
}

