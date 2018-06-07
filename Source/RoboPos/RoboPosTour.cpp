#include "StdAfx.h"
#include "Robo.h"
#include "RoboPos.h"
#include "RoboPosTour.h"
#include "RoboPosApprox.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
using namespace Robo::Mobile;
using namespace Robo::NewHand;

//------------------------------------------------------------------------------
TourI::JointsNumerator TourI::forward = [](joint_t njoints, joint_t joint, bool first = false) {
    if (first) return 0;
    return ((joint == (njoints - 1)) ? 0 : (joint + 1));
};
TourI::JointsNumerator TourI::reverse = [](joint_t njoints, joint_t joint, bool first = false) {
    if (first) return (njoints - 1);
    return ((joint == 0) ? (njoints - 1) : (joint - 1));
};
double TourI::divToMeters = 0.0028;
double TourI::divToMinutes = 60.;

//------------------------------------------------------------------------------
void TourI::appendBreakings(IN joint_t joint, IN const Actuator &a)
{
    if (_b_braking)
    {
        // добавляем торможения, если исчерпали изменения длительности
        if (_breakings_controls[joint].muscle != MInvalid && _breakings_controls[joint].lasts > 0)
        {
            // увеличиваем длительность
            _breakings_controls[joint].lasts += _lasts_step_braking_incr;
        }
        else
        {
            //CDEBUG("add breaks on last_step=" << glob_lasts_step << 
            //       " incr=" << _lasts_step_increment << " for joint=" << joint);
            // добавляем торможения
            auto opposite_muscle = RoboI::muscleOpposite(a.muscle);
            _breakings_controls[joint] = { opposite_muscle,
                                          /*start=*/(a.start + a.lasts + 1),
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
void TourI::removeBreakings(IN joint_t joint)
{
    if (_b_braking)
    {
        if (_breakings_controls[joint].lasts > _lasts_step_braking_incr)
            _breakings_controls[joint].lasts -= _lasts_step_braking_incr;
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
void TourI::cleanBreakings(IN joint_t joint)
{
    // удалить торможения, задействованные в данной вложенной процедуре
    _breakings_controls[joint] = { MInvalid, 0, 0 };
    //for (size_t ji = 0; ji <= joint; ++ji)
    //    _breakings_controls[ji].last = 0;
    //------------------------------------------
}

//------------------------------------------------------------------------------
void TourI::exactsBreakings(IN joint_t joint, IN const Control &controls)
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
            if (_breakings_controls[ji].muscle != MInvalid && _breakings_controls[ji].lasts > 0)
                _breakings_controls[ji].start = (it->start + it->lasts + 1);
        }
    }
}

//------------------------------------------------------------------------------
void TourI::run()
{
    _complexity = 0;
    _counters.clear();
    _max_nested = _robo.jointsCount();
    // ----------------------------------------------------
    _robo.reset();
    _base_pos = _robo.position();
    // ----------------------------------------------------
    try
    {
        if (_step_distance < DBL_EPSILON || _lasts_step_increment == 0)
            throw std::runtime_error{ "Increment Params aren't set" };

        Point useless;
        runNestedForMuscle(_next_joint(_robo.jointsCount(), 0, true), Robo::Control{}, useless);
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
    tcout << _T("\nStep: ") << (_step_distance / TourI::divToMeters) << _T("mm.");
    tcout << _T("\nComplexity: ") << complexity();
    tcout << _T("  minutes:") << double(complexity()) / TourI::divToMinutes << std::endl;
}

//------------------------------------------------------------------------------
bool TourI::runNestedMove(IN const Control &controls, OUT Point &robo_pos)
{
    Trajectory trajectory;
    Control controling = controls + _breakings_controls;
    //----------------------------------------------
    _robo.move(controling, trajectory);
    ++_complexity;
    robo_pos = _robo.position();

    Record rec(robo_pos, _base_pos, robo_pos, controling, trajectory);
    _store.insert(rec);

    _robo.reset();
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return true;
}



frames_t glob_lasts_step;
//------------------------------------------------------------------------------
bool TourWorkSpace::runNestedForMuscle(IN joint_t joint, IN Control &controls, OUT Point &robo_pos_high)
{
    CDEBUG("runNested " << joint/* << " last_i " << last_i*/);
    --_max_nested;
    // стартруем разными сочленениями последовательно
    //const frames_t start_i = controls.size() ? (controls[-1].lasts + controls[-1].start) : 0;
    // стартруем разными сочленениями одновременно
    const frames_t start_i = 0;
    //------------------------------------------
    Point curr_pos = _base_pos, prev_pos = _base_pos;
    //------------------------------------------
    // заметание пространства проходит рядами,
    // сдвигаясь всё дальше от начальной точки
    int high = 0; // число положений захвата робота в вышестоящем ряду
    Point max_robo_pos_high{ 0., 0. }; // сумма положений робота в вышестоящем ряду

    Actuator control_i;
    //------------------------------------------
    for (auto muscle_i : { RoboI::muscleByJoint(joint, true),
                           RoboI::muscleByJoint(joint, false) })
    {
        control_i.muscle = muscle_i;
        //------------------------------------------
        // адаптивный шаг длительности
        frames_t lasts_step = _lasts_step_increment_init;
        frames_t lasts_i_max = _robo.muscleMaxLast(muscle_i);
        //------------------------------------------
        for (frames_t last_i = lasts_step; last_i < lasts_i_max; last_i += lasts_step)
        {
            control_i.lasts = last_i;
            //------------------------------------------
            if (_max_nested)
            {
                auto nj = _next_joint(_robo.jointsCount(), joint, false);
                //===============================================================
                if (!runNestedForMuscle(nj, controls + control_i, curr_pos))
                    return false;
                //===============================================================
            }
            else
            {
                exactsBreakings(joint, controls + control_i);
                //===============================================================
                runNestedMove(controls + control_i, curr_pos);
                //===============================================================
            }
            //------------------------------------------
            if (high)
            {
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
                        // если нельзя сохранить одинаковый промежуток уменьшением длительности
                        // подключаем торможения противоположным двигателем
                        appendBreakings(joint, control_i);
                    }
                }
                else if (d < _step_distance)
                {
                    if (_b_braking && _breakings_controls_actives > 0 &&
                        _breakings_controls[joint].lasts > 0 /* !!! */)
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
            }
            //------------------------------------------
            {
                // сохраняем полученное положение робота
                // в сумму для получения среднего, которым пользуется
                // функция выше по реккурсии
                max_robo_pos_high.x += curr_pos.x;
                max_robo_pos_high.y += curr_pos.y;
                ++high;
                //CDEBUG("curr_pos " << curr_pos);
            }
            //------------------------------------------
            prev_pos = curr_pos;
            //------------------------------------------
        } // end for (lasts)

        //-----------------------------
        cleanBreakings(joint);
        //-----------------------------
    } // end for (muscle)
    //------------------------------------------
    if (high)
    {
        robo_pos_high = Point{ max_robo_pos_high.x / high,
                               max_robo_pos_high.y / high };
    }
    //------------------------------------------
    ++_max_nested;
    return true;
}


//------------------------------------------------------------------------------
TourTarget::TourTarget(IN RoboMoves::Store &store,
                       IN Robo::RoboI &robo,
                       IN Approx &approx,
                       IN TargetI &target,
                       //IN TargetContain &target_contain,
                       IN TourI::JointsNumerator &next_joint) :
    TourI(store, robo, next_joint),
    //_target_contain(target_contain),
    _approx(approx),
    _b_predict(false),
    _b_checking(false)
{
    defineTargetBorders(0.05);
    _target_contain = [&target=_target](const Point &p) { return target.contain(p); };
}

//------------------------------------------------------------------------------
void TourTarget::specifyBordersByRecord(const RoboMoves::Record &rec)
{
    for (const auto &c : rec.controls)
    {
        if (c.lasts < _borders[c.muscle].min_lasts)
            _borders[c.muscle].min_lasts = c.lasts;
        if (c.lasts > _borders[c.muscle].max_lasts)
            _borders[c.muscle].max_lasts = c.lasts;
    }
}

//------------------------------------------------------------------------------
/// Статистичеки найти приблизительную границу мишени по длительности работы мускулов
void TourTarget::defineTargetBorders(distance_t side)
{
    _borders.resize(_robo.musclesCount());
    for (const auto &rec : _store)
        if (_target_contain(rec.hit))
            specifyBordersByRecord(rec);

    adjacency_ptrs_t range;
    _store.adjacencyPoints(range, _target.min(), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec);

    range.clear();
    _store.adjacencyPoints(range, Point(_target.min().x, _target.max().y), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec);

    range.clear();
    _store.adjacencyPoints(range, Point(_target.max().x, _target.min().y), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec);

    range.clear();
    _store.adjacencyPoints(range, _target.max(), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec);
}

//------------------------------------------------------------------------------
bool TourTarget::runNestedForMuscle(IN joint_t joint, IN Control &controls, OUT Point &robo_pos_high)
{
    --_max_nested;
    auto next_joint = _next_joint(_robo.jointsCount(), joint, false);
    // стартруем разными сочленениями последовательно
    const frames_t start_i = controls.size() ? (controls[-1].lasts + controls[-1].start) : 0;
    // стартруем разными сочленениями одновременно
    //const frames_t start_i = 0;
    //------------------------------------------
    Point curr_pos = _base_pos, prev_pos = _base_pos;
    //------------------------------------------
    // заметание пространства проходит рядами,
    // сдвигаясь всё дальше от начальной точки
    int high = 0; // число положений захвата робота в вышестоящем ряду
    Point max_robo_pos_high{ 0., 0. }; // сумма положений робота в вышестоящем ряду

    /// TODO:
    /* В ОСНОВНОМ РАБОТАЕТ
     * 1. УСТАНОВИТЬ КОРРЕКТНЫЙ НАЧАЛЬНЫЙ LAST
     * 2. НЕ ВСЕГДА ОСТАНАВЛИВАЕТСЯ ПРИ ПОКИДАНИИ МИШЕНИ (А ИМЕННО ПРОБЛЕМА ВПРАВО)
     */

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
            //return true; // false;
        }

        //------------------------------------------
        frames_t lasts_step = _lasts_step_increment_init; // адаптивный шаг длительности
        frames_t lasts_i_max = _robo.muscleMaxLast(muscle_i);
        //------------------------------------------
        frames_t last_i = 0;
        for (last_i = board.min_lasts; last_i < lasts_i_max && (target_contain || !was_on_target); last_i += lasts_step)
        {
            control_i.lasts = last_i;
            //------------------------------------------
            if (last_i > board.max_lasts && !target_contain) // (prev_pos.x > _target.max().x) || (prev_pos.y < _target.min().y)
                break;
            //------------------------------------------
            if (_max_nested)
            {
                //===============================================================
                target_contain = runNestedForMuscle(next_joint, controls + control_i, curr_pos);
                //===============================================================
            }
            else
            {
                exactsBreakings(joint, controls + control_i);
                //===============================================================
                target_contain = runNestedMove(controls + control_i, curr_pos);
                //===============================================================
            }
            //------------------------------------------
            if (target_contain && !was_on_target)
            {
                board.min_lasts = last_i;
                was_on_target = true;
            }
            //------------------------------------------
            if (high)
            {
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
                        _breakings_controls[joint].lasts > 0 /* !!! */)
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
            prev_pos = curr_pos;
            //------------------------------------------
        } // end for (lasts)

        //-----------------------------
        cleanBreakings(joint);
        //-----------------------------
        {
            //target_contain = true;
            board.max_lasts = last_i;
            //------------------------------------------
            lasts_step = _lasts_step_increment_thick; // lasts_step_initiate;

            for (last_i = board.min_lasts; (last_i - lasts_step) < lasts_i_max && (target_contain); last_i -= lasts_step)
            {
                control_i.lasts = last_i;
                //------------------------------------------
                if (0U <= joint && (joint + 1U) < _max_nested)
                {
                    //===============================================================
                    target_contain = runNestedForMuscle(next_joint, controls + control_i, curr_pos);
                    //===============================================================
                }
                else
                {
                    //===============================================================
                    target_contain = runNestedMove(controls + control_i, curr_pos);
                    //===============================================================
                }
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
    } // end for (muscle)

    //------------------------------------------
    if (high)
    {
        robo_pos_high = Point{ max_robo_pos_high.x / high,
                               max_robo_pos_high.y / high };
    }
    //------------------------------------------
    ++_max_nested;
    return true;
}

//------------------------------------------------------------------------------
#include "WindowData.h"
//class MyWindowData;
std::list<Point> MyWindowData::predicts = {};
std::list<Point> MyWindowData::reals = {};

void Counters::fill(bool model, bool real, const Point &pos, const Point &pred)
{
    //------------------------------------------
    incr(model, real);
    avg_miss += boost_distance(pred, pos);

    if (model != real)
    {
        MyWindowData::predicts.push_back(pred);
        MyWindowData::reals.push_back(pos);
    }
}

//------------------------------------------------------------------------------
bool TourTarget::runNestedMove(IN const Control &controls, OUT Point &robo_pos)
{
    Trajectory trajectory;
    Control controling = controls + _breakings_controls;
    //----------------------------------------------
    if (_b_predict)
    {
        Point pred_end = _approx.predict(controling);

        if (_b_checking)
        {
            _robo.reset();
            _robo.move(controling, trajectory);
            // ++_complexity;
            //------------------------------------------
            bool model = _target_contain(pred_end);
            bool real = _target_contain(_robo.position());

            _counters.fill(model, real, _robo.position(), pred_end);
            //------------------------------------------
            trajectory.clear();
            _robo.reset();
        }

        if (!_target_contain(pred_end))
        {
            robo_pos = pred_end;
            return false;
        }
    }
    //----------------------------------------------
    _robo.move(controling, trajectory);
    ++_complexity;
    robo_pos = _robo.position();

    Record rec(robo_pos, _base_pos, robo_pos, controling, trajectory);
    _store.insert(rec);

    _robo.reset();
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return _target_contain(robo_pos);
}

