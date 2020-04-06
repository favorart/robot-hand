#include "StdAfx.h"
#include "Robo.h"
#include "Hand.h"
#include "Tank.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboPosTourNoRec.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
using namespace Robo::Mobile;
using namespace Robo::NewHand;
//------------------------------------------------------------------------------
namespace RoboPos {
struct Counters final
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
Tour::Tour(Store *store, RoboI *robo) :
    _store(store, [](Store*) {}),
    _robo(robo, [](RoboI*) {}),
    _counters(std::make_shared<Counters>())
{}

//DirectionPredictor  *pDP;
//Point  predict_shift{ 0.15, -0.05 };
////--------------------------------------------------------
//double                 step_distance;
//frames_t   _lasts_step_increment;
//frames_t   _lasts_step_increment_thick = 20U;
//frames_t   _lasts_step_initiate = 25U;
//frames_t   _lasts_step_braking = 5U;

//------------------------------------------------------------------------------
void Tour::run(bool distance, bool target, bool braking, bool predict, bool checking,
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
    _counters->clear();
    // ----------------------------------------------------
    _robo->reset();
    _base_pos = _robo->position();
    // ----------------------------------------------------
    try
    {
        if (_step_distance < DBL_EPSILON || _lasts_step_increment == 0)
            throw std::runtime_error{ "Increment Params aren't set" };

        Point useless{};
        Robo::Control c_useless{};
        runNestedForMuscle(0, c_useless, useless);
    }
    catch (boost::thread_interrupted&)
    { CINFO("WorkingThread interrupted"); }
    catch (const std::exception &e)
    { SHOW_CERROR(e.what()); }
    // ----------------------------------------------------
    if (_b_checking) { _counters->print(); }
    tcout << _T("\nStep: ") << (_step_distance / 0.0028) << _T("mm.");
    tcout << _T("\nComplexity: ") << _complexity;
    tcout << _T("  minutes:") << double(_complexity) / 60. << std::endl;
}

//------------------------------------------------------------------------------
void TourNoRecursion::specifyBordersByRecord(const RoboMoves::Record &rec)
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
void TourNoRecursion::defineTargetBorders(distance_t side)
{
    _borders.resize(_robo->musclesCount());
    for (const auto &rec : *_store)
        if (_target.contain(rec.hit))
            specifyBordersByRecord(rec);

    adjacency_ptrs_t range;
    _store->adjacencyPoints(range, _target.min(), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec);

    range.clear();
    _store->adjacencyPoints(range, Point(_target.min().x, _target.max().y), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec);

    range.clear();
    _store->adjacencyPoints(range, Point(_target.max().x, _target.min().y), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec);

    range.clear();
    _store->adjacencyPoints(range, _target.max(), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec);
}

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
        {
            CWARN("Empty borders of muscle=" << muscle_i);
            return false;
        }
        {
            lasts_step = _lasts_step_increment;
            frames_t _lasts_i_max = _robo->muscleMaxLasts(muscle_i);
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
                    auto c = (controls + control_i);
                    //===============================================================
                    target_contain = runNestedForMuscle(joint + 1, c, curr_pos) || !_b_target;
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
                    target_contain = runNestedMove((controls + control_i), curr_pos) || !_b_target;
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
                        auto c = (controls + control_i);
                        //===============================================================
                        target_contain = runNestedForMuscle(joint + 1, c, curr_pos) || !_b_target;
                        //===============================================================
                    }
                    else
                    {
                        //===============================================================
                        target_contain = runNestedMove((controls + control_i), curr_pos) || !_b_target;
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

//------------------------------------------------------------------------------
bool TourNoRecursion::runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos)
{
    // const Record &rec = _store->ClothestPoint (hand_position, 0.1);
    // if ( boost_distance (rec.hit, hand_position) < step_distance )
    // {
    //   hand_position = rec.hit;
    //   return _target.contain (hand_position);
    // }

    Point pred_end = _base_pos;
    //----------------------------------------------
    Control controling = controls + _breakings_controls;
    //----------------------------------------------
    if (_b_target || _b_distance)
    {
        pred_end = pDP->predict(controls);
        //----------------------------------------------
        // if ( _b_distance && _b_target )
        // { end.x += predict_shift.x;
        //   end.y += predict_shift.y;
        // }
        //----------------------------------------------
        if (_b_checking)
        {
            _robo->reset();
            _robo->move(controling);
            //++_complexity;
            bool model = _target.contain(pred_end);
            bool real = _target.contain(_robo->position());
            _counters->fill(model, real, _robo->position(), pred_end);
            _robo->reset();
        }
    }
    //----------------------------------------------
    if ((!_b_target || _target.contain(pred_end)) && (!_b_distance ||
        boost_distance(_target.center(), pred_end) < boost_distance(_target.max(), _target.min())))
    {
        _robo->reset();
        /* двигаем рукой ближе к мишени */
        _robo->move(controling);
        ++_complexity;
        robo_pos = _robo->position();
        //----------------------------------------------
        Record rec{ robo_pos, _base_pos, robo_pos, controling, _robo->trajectory() };
        _store->insert(rec);
        //----------------------------------------------
    }
    else if (!_b_target) { robo_pos = pred_end; }
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return _target.contain(robo_pos);
}

