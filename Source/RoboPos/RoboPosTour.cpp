#include "StdAfx.h"
#include "Robo.h"
#include "RoboPos.h"
#include "RoboPosTour.h"
#include "RoboPosApprox.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;

//------------------------------------------------------------------------------
const TourI::JointsNumerator TourI::forward = [](joint_t n_joints, joint_t joint, bool first) {
    if (first) return 0;
    return ((joint == (n_joints - 1)) ? 0 : (joint + 1));
};
const TourI::JointsNumerator TourI::reverse = [](joint_t n_joints, joint_t joint, bool first) {
    if (first) return (n_joints - 1);
    return ((joint == 0) ? (n_joints - 1) : (joint - 1));
};

const distance_t TourI::divToMeters = 0.0028;
const double TourI::divToMinutes = 60.;

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
                                           (_b_simul) ? a.start : (a.start + a.lasts + 1),
                                          _lasts_step_braking_init };
            _breakings_controls_actives++;
        }
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
    }
}

//------------------------------------------------------------------------------
void TourI::cleanBreakings(IN joint_t joint)
{
    //if (_breakings_controls[joint].muscle != MInvalid)
    //    CDEBUG("clean breaks for joint=" << joint);
    /* удалить торможения, задействованные в данной вложенной процедуре */
    _breakings_controls[joint] = { MInvalid, 0, 0 };
    //for (size_t ji = 0; ji <= joint; ++ji)
    //    _breakings_controls[ji].last = 0;
}

//------------------------------------------------------------------------------
void TourI::exactBreakings(IN joint_t joint, IN const Control &controls)
{
    if (!_b_simul && _b_braking && _breakings_controls_actives > 0)
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
            {
                CDEBUG("exact breaks " << ji << "/" << _breakings_controls_actives);
                _breakings_controls[ji].start = (it->start + it->lasts + 1);
            }
        }
    }
}

//frames_t glob_lasts_step; /// !!! RM
//------------------------------------------------------------------------------
void TourI::adaptiveLasts(IN const Point &prev_pos, IN const Point &curr_pos,
                          IN const Actuator &control_i, OUT frames_t &lasts_step,
                          IN bool target_contain, IN bool was_on_target)
{
    distance_t d = boost_distance(prev_pos, curr_pos);
    //------------------------------------------
    /* адаптивный шаг изменения длительности */
    if (d > _step_distance && target_contain)
    {
        if ((lasts_step / _lasts_step_increment) > 3 && (d / _step_distance) >= (_lasts_step_increment * 3))
        {
            frames_t n_steps = (lasts_step / _lasts_step_increment - 1);
            lasts_step -= n_steps * _lasts_step_increment;
        }
        else if (lasts_step > _lasts_step_increment)
            lasts_step -= _lasts_step_increment;
        else if (lasts_step > (_lasts_step_increment / 2) && _lasts_step_increment > 1)
            lasts_step -= (_lasts_step_increment / 2);
        else if (lasts_step > 1)
            lasts_step -= 1;
        else if (_b_braking)
        {
            //glob_lasts_step = lasts_step; /// REMOVE
            joint_t joint = RoboI::jointByMuscle(control_i.muscle);
            /* если нельзя сохранить одинаковый промежуток
             * уменьшением длительность основного,
             * подключаем торможение противоположным
             */
            appendBreakings(joint, control_i);
        }
    }
    else if (d < _step_distance || !was_on_target)
    {
        if (_b_braking && _breakings_controls_actives > 0)
        {
            joint_t joint = RoboI::jointByMuscle(control_i.muscle);
            /* сначала по возможности отключаем торможения */
            removeBreakings(joint);
        }
        else
        {
            /* затем увеличиваем длительность */
            lasts_step += _lasts_step_increment;
        }
    }
    if (lasts_step == 0)
        CERROR("lasts_step == 0");
}

//------------------------------------------------------------------------------
TourI::TourI(RoboMoves::Store &store, Robo::RoboI &robo, const TourI::JointsNumerator &next_joint) :
    _store(store), _robo(robo), _next_joint(next_joint),
    _max_nested(_robo.jointsCount()),
    _breakings_controls(_max_nested),
    _breakings_controls_actives(0),
    _b_braking(false),
    _b_simul(true)
{}

//------------------------------------------------------------------------------
void TourI::run()
{
    _complexity = 0;
    _counters.clear();
    _max_nested = _robo.jointsCount();
    // -------------------------------
    try
    {
        if (_step_distance < DBL_EPSILON || _lasts_step_increment == 0)
            throw std::runtime_error{ "Increment Params aren't set" };

        Point useless;
        runNestedForMuscle(_next_joint(_robo.jointsCount(), 0/*any*/, true/*first*/), Robo::Control{}, useless);
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
    // -------------------------------
    if (_counters.count) _counters.print();
    tcout << _T("\nStep: ") << (_step_distance / TourI::divToMeters) << _T("mm.");
    tcout << _T("\nComplexity: ") << complexity();
    tcout << _T("  minutes:") << static_cast<double>(complexity()) / TourI::divToMinutes << std::endl;
    // -------------------------------
    _robo.reset();
}

//------------------------------------------------------------------------------
bool TourI::runNestedMove(IN const Control &controls, OUT Point &robo_hit)
{
    Trajectory trajectory;
    Control controling = controls + _breakings_controls;
    //----------------------------------------------
    _robo.reset();
    Point base_pos = _robo.position();

    _robo.move(controling, trajectory);
    ++_complexity;
    robo_hit = _robo.position();

    Record rec(robo_hit, base_pos, robo_hit, controling, trajectory);
    _store.insert(rec);
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return true;
}


//------------------------------------------------------------------------------
bool TourWorkSpace::runNestedForMuscle(IN joint_t joint, IN Control &controls, OUT Point &avg_pos)
{
    --_max_nested;
    //CDEBUG("runNested " << joint/* << " last_i " << last_i*/);
    /* стартруем разными сочленениями (одновременно) или последовательно */
    const frames_t start_i = (_b_simul && controls.size()) ? (controls[-1].lasts + controls[-1].start) : 0;
    //------------------------------------------
    Point curr_pos = {}, prev_pos = {}, base_pos = {};
    distance_t advantage = 0.;
    //------------------------------------------
    /* заметание пространства проходит рядами, сдвигаясь всё дальше от начальной точки */
    int n_poses_high = 0; /* число положений захвата робота в вышестоящем ряду */
    Point all_poses_high{ 0., 0. }; /* сумма положений робота в вышестоящем ряду */

    Actuator control_i;
    //------------------------------------------
    for (auto muscle_i : { RoboI::muscleByJoint(joint, true),
                           RoboI::muscleByJoint(joint, false) })
    {
        control_i.muscle = muscle_i;
        //------------------------------------------
        /* адаптивный шаг длительности */
        frames_t lasts_step = _lasts_step_increment_init;
        frames_t lasts_i_max = _robo.muscleMaxLast(muscle_i);
        lasts_i_max = (lasts_i_max > TourI::too_long) ? TourI::too_long : lasts_i_max;
        //------------------------------------------
        for (frames_t last_i = lasts_step; last_i < lasts_i_max; last_i += lasts_step)
        {
            control_i.lasts = last_i;
            //------------------------------------------
            if (_max_nested)
            {
                auto next_joint = _next_joint(_robo.jointsCount(), joint, false/*non-first*/);
                //===============================================================
                runNestedForMuscle(next_joint, controls + control_i, curr_pos);
                //===============================================================
            }
            else
            {
                exactBreakings(joint, controls + control_i);
                //===============================================================
                runNestedMove(controls + control_i, curr_pos);
                //===============================================================
            }
            //------------------------------------------
            if (n_poses_high)
            {
                if (//(boost_distance(prev_pos, curr_pos) < Utils::EPSILONT ||
                    // boost_distance(base_pos, curr_pos) < Utils::EPSILONT) &&
                    boost_distance(base_pos, curr_pos) <= (advantage - _step_distance / 2))// && !_max_nested)
                {
                    control_i.start += _lasts_step_increment;
                    //break;
                }
                else advantage = boost_distance(base_pos, curr_pos);
                adaptiveLasts(prev_pos, curr_pos, control_i, lasts_step);
            }
            else
            {
                base_pos = curr_pos;
            }
            //------------------------------------------
            /* сохраняем полученное положение робота
             * в сумму для получения среднего, которым пользуется
             * функция выше по реккурсии
             */
            all_poses_high += curr_pos;
            ++n_poses_high;
            //CDEBUG("curr_pos " << curr_pos);
            //------------------------------------------
            prev_pos = curr_pos;
            //------------------------------------------
        }
        cleanBreakings(joint);
    }
    //------------------------------------------
    if (n_poses_high > 0)
    {
        avg_pos = all_poses_high / n_poses_high;
    }
    //------------------------------------------
    ++_max_nested;
    return true;
}


//------------------------------------------------------------------------------
TourTarget::TourTarget(IN RoboMoves::Store &store,
                       IN Robo::RoboI &robo,
                       IN Approx &approx,
                       IN const TargetI &target, // !! RM
                       IN const TargetContain &target_contain,
                       IN const TourI::JointsNumerator &next_joint) :
    TourI(store, robo, next_joint),
    _target(target), // !! RM
    _target_contain(target_contain),
    _approx(approx),
    _b_predict(false),
    _b_checking(false)
{
    defineTargetBorders(0.05);
}

//------------------------------------------------------------------------------
void TourTarget::specifyBordersByRecord(const RoboMoves::Record &rec)
{
    for (const auto &c : rec.controls)
    {
        if (c.lasts < _target_borders[c.muscle].min_lasts)
            _target_borders[c.muscle].min_lasts = c.lasts;
        if (c.lasts > _target_borders[c.muscle].max_lasts)
            _target_borders[c.muscle].max_lasts = c.lasts;
    }
}

//------------------------------------------------------------------------------
/// Статистичеки найти приблизительную границу мишени по длительности работы мускулов
void TourTarget::defineTargetBorders(distance_t side)
{
    _target_borders.resize(_robo.musclesCount());
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
bool TourTarget::runNestedForMuscle(IN joint_t joint, IN Control &controls, OUT Point &avg_pos)
{
    --_max_nested;
    auto next_joint = _next_joint(_robo.jointsCount(), joint, false/*non-first*/);
    /* стартруем разными сочленениями (одновременно) или последовательно */
    const frames_t start_i = (_b_simul && controls.size()) ? (controls[-1].lasts + controls[-1].start) : 0;
    //------------------------------------------
    Point curr_pos = {}, prev_pos = {}, base_pos = {};
    distance_t advantage = 0.;
    //------------------------------------------
    /* заметание пространства проходит рядами, сдвигаясь всё дальше от начальной точки */
    int n_poses_high = 0; /* число положений захвата робота в вышестоящем ряду */
    Point all_poses_high{ 0., 0. }; /* сумма положений робота в вышестоящем ряду */

    bool once_on_target = false;
    bool was_on_target = false; /* положение захвата ещё не достигло мишени в этом ряду */
    bool target_contain = false; /* положение захвата достигло мишени и остаётся на ней */

    Actuator control_i;
    //------------------------------------------
    for (auto muscle_i : { RoboI::muscleByJoint(joint, true),
                           RoboI::muscleByJoint(joint, false) })
    {
        control_i.muscle = muscle_i;
        //------------------------------------------
        was_on_target = false;
        target_contain = false;
        //------------------------------------------
        auto &border = _target_borders[muscle_i];
        if (border.min_lasts == 0 && border.min_lasts >= border.max_lasts)
        {
            CWARN("Empty borders of muscle=" << muscle_i);
            border.max_lasts = _robo.muscleMaxLast(muscle_i);
            border.max_lasts = (border.max_lasts > TourI::too_long) ? TourI::too_long : border.max_lasts;
        }
        //------------------------------------------
        /* адаптивный шаг длительности */
        frames_t lasts_step = _lasts_step_increment_init;
        for (frames_t last_i = (border.min_lasts + lasts_step); (last_i <= border.max_lasts) || (target_contain); last_i += lasts_step)
        {
            control_i.lasts = last_i;
            //------------------------------------------
            if (last_i >= _robo.muscleMaxLast(muscle_i))
            {
                CWARN("last_i=" << last_i << " > " << _robo.muscleMaxLast(muscle_i));
                break;
            }
            //------------------------------------------
            if (_max_nested)
            {
                //===============================================================
                target_contain = runNestedForMuscle(next_joint, controls + control_i, curr_pos);
                //===============================================================
            }
            else
            {
                exactBreakings(joint, controls + control_i);
                //===============================================================
                target_contain = runNestedMove(controls + control_i, curr_pos);
                //===============================================================
            }
            //------------------------------------------
            if (target_contain && !was_on_target)
            {
                was_on_target = true;
                once_on_target = true;
            }
            //------------------------------------------
            if (!target_contain && was_on_target)
                break;
            //------------------------------------------
            if (n_poses_high) // is not first move
            {
                if ((boost_distance(prev_pos, curr_pos) < Utils::EPSILONT ||
                     boost_distance(base_pos, curr_pos) < Utils::EPSILONT) &&
                    boost_distance(base_pos, curr_pos) >= advantage && !_max_nested)
                    break;

                advantage = boost_distance(base_pos, curr_pos);
                adaptiveLasts(prev_pos, curr_pos, control_i, lasts_step,
                              target_contain, was_on_target);
            }
            else
            {
                base_pos = curr_pos;
            }
            //------------------------------------------
            /* сохраняем полученное положение робота
             * в сумму для получения среднего, которым пользуется
             * функция выше по реккурсии
             */
            all_poses_high.x += curr_pos.x;
            all_poses_high.y += curr_pos.y;
            ++n_poses_high;
            //------------------------------------------
            prev_pos = curr_pos;
            //------------------------------------------
        }
        cleanBreakings(joint);
    }
    //------------------------------------------
    if (n_poses_high > 0)
    {
        avg_pos = all_poses_high / n_poses_high;
    }
    //------------------------------------------
    ++_max_nested;
    return once_on_target;
}

//------------------------------------------------------------------------------
#include "WindowData.h" /// !!! RM
std::list<Point> MyWindowData::predicts = {};
std::list<Point> MyWindowData::reals = {};

void Counters::fill(bool model, bool real, const Point &pos, const Point &pred)
{
    //------------------------------------------
    incr(model, real);
    auto d = boost_distance(pred, pos);
    avg_miss += d;
    if (d > 0.05)
    {
        MyWindowData::predicts.push_back(pred); // orange
        MyWindowData::reals.push_back(pos); // red
    }
    if (d > 1.)
        CDEBUG("pos=" << pos << " pred=" << pred << " dist=" << boost_distance(pred, pos));
    // TOO::MUCH !!!
}

//------------------------------------------------------------------------------
bool TourTarget::runNestedMove(IN const Control &controls, OUT Point &robo_hit)
{
    Trajectory trajectory;
    Control controling = controls + _breakings_controls;
    //----------------------------------------------
    _robo.reset();
    Point base_pos = _robo.position();
    //----------------------------------------------
    if (_b_predict)
    {
        Point pred_end = _approx.predict(controling);
        if (_b_checking)
        {
            _robo.move(controling, trajectory);
            // ++_complexity;
            //--------------------------------------------
            bool model = _target_contain(pred_end);
            bool real = _target_contain(_robo.position());
            //--------------------------------------------
            _counters.fill(model, real, _robo.position(), pred_end);
        }

        if (!_target_contain(pred_end))
        {
            robo_hit = pred_end;
            return false;
        }
        trajectory.clear();
    }
    //----------------------------------------------
    _robo.move(controling, trajectory);
    ++_complexity;
    robo_hit = _robo.position();

    Record rec(robo_hit, base_pos, robo_hit, controling, trajectory);
    _store.insert(rec);
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return _target_contain(robo_hit);
}

