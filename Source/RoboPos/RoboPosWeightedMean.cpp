#include "StdAfx.h"
#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboLearnMoves.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
/// несмещённое среднее, сливаем управления для одного мускула
void RoboPos::LearnMoves::weightedMeanControls(IN  const Point &aim,
                                               IN  const adjacency_ptrs_t &range,
                                               OUT Control &controls, // возвращает массив размера <= musclesCount, уберает все повторения мускулов -- неприменим для танка.
                                               OUT Point &mid_hit1,
                                               OUT Point &mid_hit)
{
    mid_hit = {};
    mid_hit1 = {};
    distance_t sum_range_distances = 0.;
    for (auto &pRec : range)
    {
        sum_range_distances += bg::distance(aim, pRec->hit);
        mid_hit += pRec->hit;
    }
    mid_hit /= static_cast<distance_t>(range.size());
    // -----------------------------------------------
    //CDEBUG("weightedMeanControls start");
    std::vector<double> lasts(_robo.musclesCount(), 0.);
    std::vector<double> starts(_robo.musclesCount(), 0.);
    muscle_t m_max = 0, m_min = _robo.musclesCount();
    // -----------------------------------------------
    for (auto &pRec : range)
        for (auto &c : pRec->controls)
        {
            /* взвешенное НЕСМЕЩЁННОЕ cреднее арифметическое */
            distance_t weight = bg::distance(aim, pRec->hit) / sum_range_distances;
            lasts[c.muscle] += c.lasts * weight;
            starts[c.muscle] += c.start * weight;
            if (m_max < c.muscle) m_max = c.muscle;
            if (m_min > c.muscle) m_min = c.muscle;
            //CDEBUG(c << ' ' << weight);
            mid_hit1 += pRec->hit * weight;
        }
    // ----------------------------------------------
    ++m_max;
    auto starts_it = std::min_element(std::begin(starts) + m_min, std::begin(starts) + m_max);
    for (muscle_t m = m_min; m < m_max; ++m)
        controls.append({ m, frames_t(starts[m] - *starts_it), std::max(lasts_min, frames_t(lasts[m])) });
    //CDEBUG(controls);
    // ----------------------------------------------
    /* controls check for correctness: opposite muscles work time */
    controls.validated(_robo.musclesCount());
    //CDEBUG("weightedMeanControls end");
}

//------------------------------------------------------------------------------
void layout_controls(OUT std::vector<Actuator> &v, IN const Control &c, IN const muscle_t n_muscles);
void layout_controls_continue(std::vector<Actuator> &v, const size_t max_sz, const muscle_t n_muscles);

/// не сливаем, упорядоченный по мускулам
void RoboPos::LearnMoves::weightedMeanControlsOrdered(IN const Point &aim, IN const adjacency_ptrs_t &range,
                                                      OUT std::vector<Actuator> &controls, OUT Point &mid_hit)
{

    //distance_t sum_range_distances = 0.;
    //for (auto &pRec : range)
    //{
    //    sum_range_distances += bg::distance(aim, pRec->hit);
    //    mid_hit += pRec->hit;
    //}
    //mid_hit /= static_cast<distance_t>(range.size());
    //// -----------------------------------------------
    ////CDEBUG("weightedMeanControls start");
    //std::vector<double> lasts(_robo.musclesCount(), 0.);
    //std::vector<double> starts(_robo.musclesCount(), 0.);
    //muscle_t m_max = 0, m_min = _robo.musclesCount();
    //// -----------------------------------------------
    //for (auto &pRec : range)
    //    for (auto &c : pRec->controls)
    //    {
    //        /* взвешенное НЕСМЕЩЁННОЕ cреднее арифметическое */
    //        distance_t weight = bg::distance(aim, pRec->hit) / sum_range_distances;
    //        lasts[c.muscle] += c.lasts * weight;
    //        starts[c.muscle] += c.start * weight;
    //        if (m_max < c.muscle) m_max = c.muscle;
    //        if (m_min > c.muscle) m_min = c.muscle;
    //        //CDEBUG(c << ' ' << weight);
    //    }
    //// ----------------------------------------------
    //++m_max;
    //auto starts_it = std::min_element(std::begin(starts) + m_min, std::begin(starts) + m_max);
    //for (muscle_t m = m_min; m < m_max; ++m)
    //    controls.append({ m, frames_t(starts[m] - *starts_it), std::max(lasts_min, frames_t(lasts[m])) });
    ////CDEBUG(controls);
    //// ----------------------------------------------
    ///* controls check for correctness: opposite muscles work time */
    //controls.validated(_robo.musclesCount());
    ////CDEBUG("weightedMeanControls end");
}

//------------------------------------------------------------------------------
distance_t RoboPos::LearnMoves::weightedMean(IN const Point &aim)
{
    auto side_tmp = 1.5 * annealing;
    // -----------------------------------------------
    auto p = _store.getClosestPoint(aim, side_tmp);
    if (!p.first)
    {
        CINFO("weightedMean: empty adjacency aim=" << aim << " side=" << side_tmp);
        return bg::distance(aim, _base_pos);
    }
    //auto rec = _store.closestEndPoint(aim);
    // -----------------------------------------------
    distance_t distance, next_distance = bg::distance(aim, p.second.hit);
    do
    {
        distance = next_distance;
        // -----------------------------------------------
        adjacency_ptrs_t range;
        _store.adjacencyByPBorders(range, aim, side_tmp);
        if (range.empty())
        {
            CINFO("weightedMean: empty adjacency aim=" << aim << " side=" << side_tmp);
            break;
        }
        // -----------------------------------------------
        side_tmp -= side_decrease_step;
        // -----------------------------------------------
        Point mid_hit;
        Point mid_hit1;
#if 1
        Control controls;
        weightedMeanControls(aim, range, controls, mid_hit1, mid_hit);
#else
        Control controls;
        std::vector<Actuator> v;
        layout_controls(v, controls, _robo.musclesCount());
        weightedMeanControlsOrdered(aim, range, v, mid_hit);
#endif
        range.clear();
        next_distance = bg::distance(aim, mid_hit);
        // -----------------------------------------------
        append(mid_hit_stat, next_distance);
        append(mid_hit_stat1, bg::distance(aim, mid_hit1));
        // -----------------------------------------------
        auto hit = predict(controls);
        auto d = bg::distance(aim, hit);
        //if (less(distance, next_distance) || less(distance, d)) // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        {
            //============================
            next_distance = actionRobo(aim, controls);
            ++_wmean_complexity;
            //============================
        }
        // -----------------------------------------------
        if (check_precision(distance, aim))
            break;
    } while (less(distance, next_distance));

    CINFO(_T("weightedMean precision: ") << distance << std::endl);
    return distance;
}
//------------------------------------------------------------------------------
bool RoboPos::LearnMoves::weightedMeanULAdjs(IN  const Point &aim, OUT Record *pRec,
                                             OUT Control &lower_controls,
                                             OUT Control &upper_controls,
                                             OUT distance_t &delta)
{
    if (!pRec) return false;

    adjacency_ptrs_t range;
    Point mid_hit;
    Point min, max, lo_hit, hi_hit;
    // ------------------------------------------------
    lower_controls.clear();
    upper_controls.clear();
    // ------------------------------------------------

    //_store.adjacencyPoints(range, aim, /*side3*/annealing);
    //ClosestPredicate cp(aim);
    //auto it_min = std::min_element(range.begin(), range.end(), cp);
    //if (it_min == range.end())
    //    return false;
    //*pRec = (**it_min);
    //range.clear();


    auto p = _store.getClosestPoint(aim, /*side3*/annealing);
    if (!p.first)
    {
        CINFO("weightedMeanULAdjs: empty adjacency aim=" << aim << " side=" << /*side3*/annealing);
        return bg::distance(aim, _base_pos);
    }
    //*pRec = _store.closestEndPoint(aim);
    *pRec = p.second;

    // ------------------------------------------------
    min = Point(aim.x - /*side3*/annealing, pRec->hit.y - /*side3*/annealing);
    max = Point(aim.x + /*side3*/annealing, pRec->hit.y);

    _store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (range.empty())
        return false;
    // ------------------------------------------------
    weightedMeanControls(aim, range, lower_controls, mid_hit, lo_hit);
    range.clear();
    // ------------------------------------------------
    min = Point(aim.x - /*side3*/annealing, pRec->hit.y);
    max = Point(aim.x + /*side3*/annealing, pRec->hit.y + /*side3*/annealing);

    _store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (range.empty())
        return false;
    // ------------------------------------------------
    weightedMeanControls(aim, range, upper_controls, mid_hit, hi_hit);
    range.clear();
    // ------------------------------------------------
    delta = bg::distance(lo_hit, hi_hit);
    return true;
}
//------------------------------------------------------------------------------

