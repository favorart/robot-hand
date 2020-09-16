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
                                               OUT Point &mid_hit)
{
#ifdef USE_MID_STAT
    Point mid_hit1{};
#endif
    mid_hit = {};
    distance_t sum_range_distances = 0.;
    for (auto &pRec : range)
    {
        sum_range_distances += bg::distance(aim, pRec->hit);
        mid_hit += pRec->hit;
    }
    mid_hit /= static_cast<distance_t>(range.size());
    // -----------------------------------------------
    //CDEBUG("weightedMeanControls start");
    std::vector<distance_t> lasts(robo_nmuscles, 0.);
    std::vector<distance_t> starts(robo_nmuscles, 0.);
    muscle_t m_max = 0, m_min = robo_nmuscles;
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
#ifdef USE_MID_STAT
            mid_hit1 += pRec->hit * weight;
#endif
        }
    // ----------------------------------------------
    ++m_max;
    auto starts_it = std::min_element(std::begin(starts) + m_min, std::begin(starts) + m_max);
    for (muscle_t m = m_min; m < m_max; ++m)
        controls.append({ m, frames_t(starts[m] - *starts_it), std::max(lasts_min, frames_t(lasts[m])) });
    //CDEBUG(controls);
    // ----------------------------------------------
    /* controls check for correctness: opposite muscles work time */
    controls.validated(robo_nmuscles);
    //CDEBUG("weightedMeanControls end");
    // -----------------------------------------------
#ifdef USE_MID_STAT
    append(mid_hit_stat, bg::distance(aim, mid_hit));
    append(mid_hit_stat1, bg::distance(aim, mid_hit1));
#endif
}

//------------------------------------------------------------------------------
void layout_controls(std::vector<Actuator> &v, frames_t &last_start, const Control &c, const muscle_t n_muscles);
void layout_controls_continue(std::vector<Actuator> &v, const frames_t last_start, const size_t max_sz, const muscle_t n_muscles);

/// не сливаем, упорядоченный по мускулам
void RoboPos::LearnMoves::weightedMeanControlsOrdered(IN const Point &aim, IN const adjacency_ptrs_t &range,
                                                      OUT Control &controls, OUT Point &mid_hit)
{
    distance_t sum_range_distances = 0.;
    for (auto &pRec : range)
    {
        sum_range_distances += bg::distance(aim, pRec->hit);
        mid_hit += pRec->hit;
    }
    mid_hit /= static_cast<distance_t>(range.size());
    // -----------------------------------------------
    frames_t lasts_start = 0;
    std::vector<std::vector<Robo::Actuator>> range_ordered(range.size());
    // -----------------------------------------------
    int i = 0;
    for (auto &pRec : range)
        layout_controls(range_ordered[i++], lasts_start, pRec->controls, robo_nmuscles);
    size_t max_sz = br::max_element(range_ordered, [](auto &l, auto &r) { return l.size() < r.size(); })->size();
    i = 0;
    for (auto &pRec : range)
        layout_controls_continue(range_ordered[i++], lasts_start, max_sz, robo_nmuscles);
    // -----------------------------------------------
    //CDEBUG("weightedMeanControls Ordered start");
    std::vector<distance_t> lasts(max_sz, 0.);
    std::vector<distance_t> starts(max_sz, 0.);
    // -----------------------------------------------
    auto it = range.begin();
    i = 0;
    for (auto &v : range_ordered)
    {
        for (auto &c : v)
        {
            /* взвешенное НЕСМЕЩЁННОЕ cреднее арифметическое */
            distance_t weight = bg::distance(aim, (*it)->hit) / sum_range_distances;
            lasts[i] += c.lasts * weight;
            starts[i] += c.start * weight;
            //CDEBUG(c << ' ' << weight);
        }
        ++it;
    }
    // ----------------------------------------------
    frames_t min_start = frames_t(*br::min_element(starts));
    for (i = 0; i < max_sz; ++i)
        if (lasts[i] > 0)
            controls.append({ range_ordered[0][i].muscle, std::max(0ULL, frames_t(starts[i]) - min_start), std::max(0ULL, frames_t(lasts[i])) });
    //tcerr << _T("weightedMeanControls Ordered c=") << controls << std::endl;
    //CDEBUG(controls);
    // ----------------------------------------------
    /* controls check for correctness: opposite muscles work time */
    if (controls.size())
        controls.order(robo_nmuscles);
    //CDEBUG("weightedMeanControls Ordered end");
}

//------------------------------------------------------------------------------
distance_t RoboPos::LearnMoves::weightedMean(IN const Point &aim)
{
    auto side_tmp = 1.5 * annealing;
    // -----------------------------------------------
    auto p = _store->getClosestPoint(aim, side_tmp);
    if (!p.first)
    {
        CINFO("weightedMean: empty adjacency aim=" << aim << " side=" << side_tmp);
        return bg::distance(aim, _base_pos);
    }
    //auto rec = _store->closestEndPoint(aim);
    // -----------------------------------------------
    distance_t distance, next_distance = bg::distance(aim, p.second.hit);
    do
    {
        distance = next_distance;
        // -----------------------------------------------
        adjacency_ptrs_t range;
        _store->adjacencyByPBorders(range, aim, side_tmp);
        if (range.empty())
        {
            CINFO("weightedMean: empty adjacency aim=" << aim << " side=" << side_tmp);
            break;
        }
        // -----------------------------------------------
        side_tmp -= side_decrease_step;
        // -----------------------------------------------
        Point mid_hit;
#if 1
        Control controls;
        //weightedMeanControls(aim, range, controls, mid_hit);
        weightedMeanControlsOrdered(aim, range, controls, mid_hit);
        if (!controls.size())
            return distance;
#else
        Control controls;
        std::vector<Actuator> v;
        layout_controls(v, controls, robo_nmuscles);
        weightedMeanControlsOrdered(aim, range, v, mid_hit);
#endif
        range.clear();
        next_distance = bg::distance(aim, mid_hit);
        // -----------------------------------------------
        auto hit = predict(controls);
        auto d = bg::distance(aim, hit);
        //if (less(distance, next_distance) || less(distance, d)) // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        {
            //============================
            distance_t nd = actionRobo(aim, controls);
            //++_wmean_complexity;
            updateReachedStat(Admix::WeightMean);
            //============================
            if ((!less(distance, next_distance) && !less(distance, d)) && less(d, nd))
                tcout << "weightedMean predict false";
            next_distance = nd;
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
    Point min, max, lo_hit, hi_hit;
    // ------------------------------------------------
    lower_controls.clear();
    upper_controls.clear();
    // ------------------------------------------------

    //_store->adjacencyPoints(range, aim, /*side3*/annealing);
    //ClosestPredicate cp(aim);
    //auto it_min = std::min_element(range.begin(), range.end(), cp);
    //if (it_min == range.end())
    //    return false;
    //*pRec = (**it_min);
    //range.clear();


    auto p = _store->getClosestPoint(aim, /*side3*/annealing);
    if (!p.first)
    {
        CINFO("weightedMeanULAdjs: empty adjacency aim=" << aim << " side=" << /*side3*/annealing);
        return bg::distance(aim, _base_pos);
    }
    //*pRec = _store->closestEndPoint(aim);
    *pRec = p.second;

    // ------------------------------------------------
    min = Point(aim.x - /*side3*/annealing, pRec->hit.y - /*side3*/annealing);
    max = Point(aim.x + /*side3*/annealing, pRec->hit.y);

    _store->adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (range.empty())
        return false;
    // ------------------------------------------------
    weightedMeanControls(aim, range, lower_controls, lo_hit);
    range.clear();
    // ------------------------------------------------
    min = Point(aim.x - /*side3*/annealing, pRec->hit.y);
    max = Point(aim.x + /*side3*/annealing, pRec->hit.y + /*side3*/annealing);

    _store->adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (range.empty())
        return false;
    // ------------------------------------------------
    weightedMeanControls(aim, range, upper_controls, hi_hit);
    range.clear();
    // ------------------------------------------------
    delta = bg::distance(lo_hit, hi_hit);
    return true;
}
//------------------------------------------------------------------------------

