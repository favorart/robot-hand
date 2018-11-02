#include "StdAfx.h"

#include "Robo.h"
#include "RoboPosTour.h"
#include "RoboPosApprox.h"
#include "RoboMovesStore.h"
#include "RoboMovesTarget.h"

#include "TourAdaptiveLasts.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
namespace RoboPos {
struct Counters
{
    int count = 0;
    int count_TP = 0;
    int count_FP = 0;
    int count_TN = 0;
    int count_FN = 0;
    double avg_miss = 0.;

    Counters() = default;
    void incr(bool model, bool real)
    {
        ++count;
        if (model && real)
            ++count_TP;
        else if (!model && !real)
            ++count_TN;
        else if (model && !real)
            ++count_FP;
        else if (!model && real)
            ++count_FN;
    }
    void fill(bool contain_pred, bool contain_pos, const Point &pos, const Point &pred);
    void print()
    {
        tcout << std::endl;
        tcout << _T("count = ") << count << _T(" avg_miss=") << avg_miss / count << std::endl;
        tcout << _T("\t < T > \t < F >") << std::endl;
        tcout << _T("< P >\t") << std::setw(5) << count_TP << _T("\t") << std::setw(5) << count_FP << std::endl;
        tcout << _T("< N >\t") << std::setw(5) << count_TN << _T("\t") << std::setw(5) << count_FN << std::endl;
    }
    void clear()
    {
        count = 0;
        count_TP = 0;
        count_FP = 0;
        count_TN = 0;
        count_FN = 0;
        avg_miss = 0.;
    }
};
}

//------------------------------------------------------------------------------
const TourI::JointsNumerator TourI::forward = [](joint_t n_joints, joint_t joint, bool first) {
    if (first) return 0;
    return ((joint == (n_joints - 1)) ? 0 : (joint + 1));
};
const TourI::JointsNumerator TourI::reverse = [](joint_t n_joints, joint_t joint, bool first) {
    if (first) return (n_joints - 1);
    return ((joint == 0) ? (n_joints - 1) : (joint - 1));
};

const distance_t TourI::divToMiliMeters = 0.0028;
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
                          IN const Actuator &control_i, IN OUT frames_t &lasts_step,
                          IN bool target_contain, IN bool was_on_target)
{
    distance_t d = boost_distance(prev_pos, curr_pos);
    //------------------------------------------
    /* адаптивный шаг изменения длительности */
    if (d > _step_distance /*&& target_contain*/)
    {
        if ((lasts_step / _lasts_step_increment) > 2 || (d / _step_distance) > 2)
        {
            //frames_t n_steps = (lasts_step / _lasts_step_increment - 1);
            //lasts_step -= n_steps * _lasts_step_increment;
            lasts_step = std::max(_lasts_step_increment, frames_t(lasts_step * _step_distance / d));
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
inline Robo::frames_t musclesMinLasts(const Robo::RoboI &robo)
{
    frames_t l = 0;
    for (muscle_t m = 0; m < robo.musclesCount(); ++m)
        if (l < robo.muscleMaxLasts(m))
            l = robo.muscleMaxLasts(m);
    return l;
}

//------------------------------------------------------------------------------
TourI::TourI(RoboMoves::Store &store, Robo::RoboI &robo, tptree &config, const TourI::JointsNumerator &nj) :
    _store(store), _robo(robo), _config(config), _next_joint(nj), _counters(Counters{}),
    _max_nested(_robo.jointsCount()),
    _breakings_controls(_max_nested),
    _breakings_controls_actives(0),
    _b_braking(false),
    _b_simul(true)
{
    GET_OPTS(_config, _b_simul, TourI);
    GET_OPTS(_config, _b_starts, TourI);
    GET_OPTS(_config, _b_braking, TourI);
    GET_OPTS(_config, _step_distance, TourI);

    GET_OPTS(_config, _lasts_step_increment_init, TourI);
    GET_OPTS(_config, _lasts_step_increment, TourI);
    GET_OPTS(_config, _lasts_step_braking_init, TourI);
    GET_OPTS(_config, _lasts_step_braking_incr, TourI);
    GET_OPTS(_config, _lasts_step_on_target, TourI);
    GET_OPTS(_config, _lasts_step_n, TourI);

    //tstring next_joint;
    //GET_OPT(_config, next_joint, TourI);
    //if      (next_joint == _T("reverse")) _next_joint = reverse;
    //else if (next_joint == _T("forward")) _next_joint = forward;    
    //else if (next_joint == _T("customs")) _next_joint = nj;

    //_p_avg_lasts = std::make_shared<TourI::AvgLastsIncrement>(_robo.musclesCount(), musclesMinLasts(_robo),
    //                                                          _lasts_step_n,
    //                                                          _lasts_step_increment_init,
    //                                                          _lasts_step_increment,
    //                                                          _lasts_step_braking_init,
    //                                                          _lasts_step_braking_incr,
    //                                                          _lasts_step_on_target);
}

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
        printParameters();

        Point useless;
        runNestedForMuscle(_next_joint(_robo.jointsCount(), 0/*any*/, true/*first*/), Robo::Control{}, useless);
    }
    catch (boost::thread_interrupted&)
    { CINFO("WorkingThread interrupted"); }
    catch (const std::exception &e)
    { SHOW_CERROR(e.what()); }
    // -------------------------------
    if (_counters.count) _counters.print();
    CINFO(std::endl << 
          _T("step: ") << (_step_distance / TourI::divToMiliMeters) << _T(" mm.") << std::endl <<
          _T("Complexity: ") << complexity() <<
          _T("  minutes:") << static_cast<double>(complexity()) / TourI::divToMinutes);
    // -------------------------------
    _robo.reset();
}

//------------------------------------------------------------------------------
bool TourI::runNestedMove(IN const Control &controls, OUT Point &robo_hit)
{
    Control controling = controls + _breakings_controls;
    //----------------------------------------------
    const Record *pRec = _store.exactRecordByControl(controling);
    if (pRec)
    {
        robo_hit = pRec->hit;
        return true;
    }
    //----------------------------------------------
    _robo.reset();
    Point base_pos = _robo.position();

    _robo.move(controling);
    ++_complexity;
    robo_hit = _robo.position();

    Record rec(robo_hit, base_pos, robo_hit, controling, _robo.trajectory(), _lasts_step_increment);
    _store.insert(rec);
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return true;
}

//------------------------------------------------------------------------------
TourWorkSpace::TourWorkSpace(RoboMoves::Store &store, Robo::RoboI &robo, tptree &config, const TourI::JointsNumerator &next_joint) :
    TourI(store, robo, config, next_joint)
{
    GET_OPTS(_config, _b_simul, workspace);
    GET_OPTS(_config, _b_starts, workspace);
    GET_OPTS(_config, _b_braking, workspace);
    GET_OPTS(_config, _step_distance, workspace);

    GET_OPTS(_config, _lasts_step_increment_init, workspace);
    GET_OPTS(_config, _lasts_step_increment, workspace);
    GET_OPTS(_config, _lasts_step_braking_init, workspace);
    GET_OPTS(_config, _lasts_step_braking_incr, workspace);
    GET_OPTS(_config, _lasts_step_on_target, workspace);
    GET_OPTS(_config, _lasts_step_n, workspace);
}

//------------------------------------------------------------------------------
void TourWorkSpace::printParameters() const
{
    CINFO(std::endl <<
          "\nTourWorkSpace:\n\nb_simul=" << _b_simul <<
          "\nb_starts=" << _b_starts <<
          "\nb_braking=" << _b_braking <<
          "\nstep_distance=" << _step_distance <<
          "\nlasts_step_increment=" << _lasts_step_increment <<
          "\nlasts_step_increment_init=" << _lasts_step_increment_init <<
          "\nlasts_step_braking_init=" << _lasts_step_braking_init <<
          "\nlasts_step_braking_incr=" << _lasts_step_braking_incr <<
          "\nlasts_step_on_target=" << _lasts_step_on_target <<
          "\nlasts_step_n=" << _lasts_step_n << std::endl);
}

//------------------------------------------------------------------------------
bool TourWorkSpace::runNestedForMuscle(IN joint_t joint, IN Control &controls, OUT Point &avg_pos)
{
    --_max_nested;
    //CDEBUG("runNested " << joint/* << " last_i " << last_i*/);
    /* стартруем разными сочленениями (одновременно) или последовательно */
    //const frames_t start_i = (_b_simul && controls.size()) ? (controls[-1].lasts + controls[-1].start) : 0; // ???
    //------------------------------------------
    Point curr_pos = {}, prev_pos = {}, base_pos = {};
    distance_t advantage = 0.;
    //------------------------------------------
    /* заметание пространства проходит рядами, сдвигаясь всё дальше от начальной точки */
    int n_poses_high = 0; /* число положений захвата робота в вышестоящем ряду */
    Point all_poses_high{ 0., 0. }; /* сумма положений робота в вышестоящем ряду */

    Actuator control_i;
    //control_i.start = start_i; // ???
    //------------------------------------------
    for (auto muscle_i : { RoboI::muscleByJoint(joint, true),
                           RoboI::muscleByJoint(joint, false) })
    {
        control_i.muscle = muscle_i;
        //------------------------------------------
        /* адаптивный шаг длительности */
        frames_t lasts_step = _lasts_step_increment_init;
        //frames_t lasts_step = _p_avg_lasts->li_step(control_i, LI_STEP_INIT);
        //bool init = true;

        frames_t lasts_i_max = _robo.muscleMaxLasts(muscle_i);
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
                    //if (_b_simul) control_i.start += _lasts_step_increment; // ?????
                }
                else advantage = boost_distance(base_pos, curr_pos);
                adaptiveLasts(prev_pos, curr_pos, control_i, lasts_step);
                //adaptiveAvgLasts(prev_pos, curr_pos, control_i, lasts_step, true, true, init);
                //init = false;
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
                       IN tptree &config,
                       IN const TargetI &target,
                       IN const TargetContain &target_contain,
                       IN const TourI::JointsNumerator &next_joint) :
    TourI(store, robo, config, next_joint),
    _target(target),
    _target_contain(target_contain),
    _approx(Approx(_store.size(), _max_n_controls,
                   /*noize*/[](size_t) { return 0.00000000001; },
                   /*sizing*/[]() { return 1.01; })),
    _b_predict(false),
    _b_checking(false)
{
    double borders_side;
    GET_OPTS(_config, borders_side, target);
    defineTargetBorders(borders_side);

    GET_OPTS(_config, _b_simul, target);
    GET_OPTS(_config, _b_starts, target);
    GET_OPTS(_config, _b_braking, target);
    GET_OPTS(_config, _b_predict, target);
    GET_OPTS(_config, _b_checking, target);
    GET_OPTS(_config, _step_distance, target);
    GET_OPTS(_config, _lasts_step_increment, target);
    GET_OPTS(_config, _lasts_step_increment_init, target);
    GET_OPTS(_config, _lasts_step_braking_init, target);
    GET_OPTS(_config, _lasts_step_braking_incr, target);
    GET_OPTS(_config, _lasts_step_on_target, target);
    GET_OPTS(_config, _lasts_step_n, target);

    if (_b_predict && !_approx.constructed())
        //  FILTERING !!!!!!!
        _approx.constructXY(store);
}

//------------------------------------------------------------------------------
void TourTarget::printParameters() const
{
    CINFO(std::endl << 
          "\nTourTarget:\n\nb_simul=" << _b_simul << 
          "\nb_starts=" << _b_starts << 
          "\nb_braking=" << _b_braking << 
          "\nb_predict=" << _b_predict << 
          "\nb_checking=" << _b_checking << 
          "\nstep_distance=" << _step_distance << 
          "\nlasts_step_increment=" << _lasts_step_increment << 
          "\nlasts_step_increment_init=" << _lasts_step_increment_init << 
          "\nlasts_step_braking_init=" << _lasts_step_braking_init << 
          "\nlasts_step_braking_incr=" << _lasts_step_braking_incr << 
          "\nlasts_step_on_target=" << _lasts_step_on_target << 
          "\nlasts_step_n=" << _lasts_step_n << std::endl);
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
    //const frames_t start_i = (_b_simul && controls.size()) ? (controls[-1].lasts + controls[-1].start) : 0; // ???
    //------------------------------------------
    Point curr_pos = {}, prev_pos = {}, base_pos = {};
    //------------------------------------------
    /* заметание пространства проходит рядами, сдвигаясь всё дальше от начальной точки */
    int n_poses_high = 0; /* число положений захвата робота в вышестоящем ряду */
    Point all_poses_high{ 0., 0. }; /* сумма положений робота в вышестоящем ряду */

    bool once_on_target = false;
    bool was_on_target = false; /* положение захвата ещё не достигло мишени в этом ряду */
    bool target_contain = false; /* положение захвата достигло мишени и остаётся на ней */

    Actuator control_i;
    //control_i.start = start_i; // ???
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
            continue;
            //CWARN("Empty borders of muscle=" << muscle_i);
            //border.max_lasts = std::min(TourI::too_long, _robo.muscleMaxLasts(muscle_i));
        }
        //------------------------------------------
        /* адаптивный шаг длительности */
        frames_t lasts_step = _lasts_step_increment_init;
        //frames_t lasts_step = _p_avg_lasts->li_step(control_i, LI_STEP_INIT);
        //bool init = true;

        for (frames_t last_i = (border.min_lasts + lasts_step); (last_i <= border.max_lasts) || (target_contain); last_i += lasts_step)
        {
            control_i.lasts = last_i;
            //------------------------------------------
            if (last_i >= _robo.muscleMaxLasts(muscle_i))
            {
                CWARN("last_i=" << last_i << " > " << _robo.muscleMaxLasts(muscle_i));
                break;
            }
            //------------------------------------------
            if (_max_nested > 0)
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
            //if (!target_contain && was_on_target)
            //    break;
            //------------------------------------------
            if (n_poses_high) // is not first move
            {
                if (!target_contain &&
                    boost_distance(_target.center(), curr_pos) > boost_distance(_target.center(), prev_pos))
                {
                    if (_max_nested == _robo.jointsCount())
                        CWARN("Center is far " << joint);
                    break;
                }
                //------------------------------------------
                /* при неподвижности или при удалении от мишени прекращаем данный проход */
                distance_t advantage_prev = boost_distance(base_pos, prev_pos);
                distance_t advantage_curr = boost_distance(base_pos, curr_pos);
                distance_t distance_curr = boost_distance(prev_pos, curr_pos);
                if ((distance_curr < Utils::EPSILONT || advantage_curr < Utils::EPSILONT) &&
                    advantage_curr >= advantage_prev) // && !_max_nested)
                {
                    CWARN("advantage is small");
                    break;
                }
                //------------------------------------------
                adaptiveLasts(prev_pos, curr_pos, control_i, lasts_step,
                              target_contain, was_on_target);
                //adaptiveAvgLasts(prev_pos, curr_pos, control_i, lasts_step,
                //                 target_contain, was_on_target, init);
            }
            else
            {
                // first time only
                base_pos = curr_pos;
            }
            //------------------------------------------
            /* сохраняем полученное положение робота в сумму для среднего арифм.,
             * которое будет обобщённой координатой ряда в функции по реккурсии выше
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
    Control controling = controls + _breakings_controls;
    //----------------------------------------------
    const Record *pRec = _store.exactRecordByControl(controling);
    if (pRec)
    {
        robo_hit = pRec->hit;
        return _target_contain(robo_hit);
    }
    //----------------------------------------------
    _robo.reset();
    Point base_pos = _robo.position(), pred_end{};
    //----------------------------------------------
    if (_b_predict)
    {
        pred_end = _approx.predict(controling);
        if (_b_checking)
        {
            _robo.move(controling);
            // ++_complexity;
            //--------------------------------------------
            bool model = _target_contain(pred_end);
            bool real = _target_contain(_robo.position());
            //--------------------------------------------
            _counters.fill(model, real, _robo.position(), pred_end);
            _robo.reset();
        }

        if (!_target_contain(pred_end))
        {
            robo_hit = pred_end;
            return false;
        }
    }
    //----------------------------------------------
    _robo.move(controling);
    ++_complexity;
    robo_hit = _robo.position();

    Record rec(robo_hit, base_pos, robo_hit, controling, _robo.trajectory(), _lasts_step_increment);
    _store.insert(rec);
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    //tcout << robo_hit << '-' << pred_end << ' ' << _target_contain(robo_hit) << std::endl;
    //----------------------------------------------
    return _target_contain(robo_hit);
}
