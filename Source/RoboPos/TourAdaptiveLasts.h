#include "StdAfx.h"

#include "Robo.h"
#include "RoboPosTour.h"
#include "RoboMovesStore.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
class TourI::AvgLastsIncrement
{
    struct Data { Robo::frames_t lasts; unsigned n; };
    using MusclesAvgLasts = std::vector<std::array<Data, _LI_LAST_>>;

    MusclesAvgLasts _avg_lasts_specs;
    std::vector<Data> _avg_lasts_steps;
    frames_t _range_n;
    frames_t _range_len;

    //std::vector<Robo::frames_t> avg_speed_on_start_for_muscle;
    //std::vector<Robo::frames_t> avg_speed_changes_for_muscle;
    //std::vector<Robo::frames_t> avg_speed_on_target_for_muscle;

public:
    AvgLastsIncrement(muscle_t n_muscles,
                      frames_t max_lasts,
                      frames_t n,
                      frames_t incr_init,
                      frames_t incr_step,
                      frames_t brake_init,
                      frames_t brake_incr,
                      frames_t on_target) :
        AvgLastsIncrement(n_muscles, max_lasts, n, LastsIncrs{ incr_init, incr_step, brake_init, brake_incr, on_target })
    {}
    AvgLastsIncrement(muscle_t n_muscles, frames_t max_lasts,
                      frames_t n, LastsIncrs increments) :
        _range_len(max_lasts / n), _range_n(n)
    {
        _avg_lasts_specs.resize(n_muscles);
        _avg_lasts_steps.resize(n_muscles * n, { increments[LI_STEP], 1 });

        for (muscle_t m = 0; m < n_muscles; ++m)
            for (auto i = 0u; i < increments.size(); ++i)
                _avg_lasts_specs[m][i] = { increments[i], 1 };
    }
    frames_t li_step(const Actuator &a, LastsIncrement li) const
    {
        const Data *data;
        if (li == LI_STEP)
        {
            frames_t range = a.lasts / _range_len;
            data = &_avg_lasts_steps[a.muscle * _range_n + range];
        }
        else
        {
            data = &_avg_lasts_specs[a.muscle][li];
        }
        return (data->lasts / data->n);
    }
    void recalc(const Actuator &a, LastsIncrement li, frames_t incr, double ratio)
    {
        Data *data;
        if (li == LI_STEP)
        {
            frames_t range = a.lasts / _range_len;
            data = &_avg_lasts_steps[a.muscle * _range_n + range];
        }
        else
        {
            data = &_avg_lasts_specs[a.muscle][li];
        }
        data->lasts += frames_t(incr * ratio);
        data->n++;
    }
};

inline void TourI::adaptiveAvgLasts(IN const Point &prev_pos, IN const Point &curr_pos,
                             IN const Robo::Actuator &control_i, IN OUT Robo::frames_t &lasts_step,
                             IN bool target_contain, IN bool was_on_target, IN bool init)
{
    distance_t d = bg::distance(prev_pos, curr_pos);
    //------------------------------------------
    if (_b_braking && d > _step_distance && lasts_step < 2)
    {
        //glob_lasts_step = lasts_step; /// REMOVE
        joint_t joint = RoboI::jointByMuscle(control_i.muscle);
        /* если нельзя сохранить одинаковый промежуток
        * уменьшением длительность основного,
        * подключаем торможение противоположным
        */
        appendBreakings(control_i.muscle);
    }
    else if (_b_braking && (d < _step_distance || !was_on_target) && _breakings_controls_actives > 0)
    {
        /* сначала по возможности отключаем торможения */
        removeBreakings(control_i.muscle);
    }
    else
    {
        double coef = (d <= _step_distance && !was_on_target) ? ((2 * _step_distance - d) / _step_distance) : (d / _step_distance);
        _p_avg_lasts->recalc(control_i, (init) ? LI_STEP_INIT : LI_STEP, lasts_step, coef);
        lasts_step = _p_avg_lasts->li_step(control_i, LI_STEP);
    }
    if (lasts_step == 0)
        CERROR("lasts_step == 0");
}
