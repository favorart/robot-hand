#include "StdAfx.h"
#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboLearnMoves.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
void RoboPos::LearnMoves::weightedMeanControls(IN  const Point &aim,
                                               IN  const adjacency_ptrs_t &range,
                                               OUT Control &controls, // возвращает массив размера <= musclesCount, уберает все повторения мускулов -- неприменим для танка.
                                               OUT Point &mid_hit)
{
    double sum_range_distances = 0.;
    for (auto &pRec : range)
    {
        sum_range_distances += boost_distance(aim, pRec->hit);
        mid_hit += pRec->hit;
    }
    mid_hit /= range.size();
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
            double weight = boost_distance(aim, pRec->hit) / sum_range_distances;
            lasts[c.muscle] += c.lasts * weight;
            starts[c.muscle] += c.start * weight;
            if (m_max < c.muscle) m_max = c.muscle;
            if (m_min > c.muscle) m_min = c.muscle;
            //CDEBUG(c << ' ' << weight);
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
size_t RoboPos::LearnMoves::weightedMean(IN const Point &aim, OUT Point &hit)
{
    size_t w_means_complexity = 0;
    // -----------------------------------------------
    auto p = _store.getClosestPoint(aim, side3);
    if (!p.first)
        throw std::runtime_error{ "weightedMean: Empty adjacency" };
    Point pos = p.second.hit;
    // -----------------------------------------------
    double distance = boost_distance(aim, pos),
      next_distance = distance;
    // -----------------------------------------------
    _robo.reset();
    do
    {
        if (next_distance < distance)
        {
            distance = next_distance;
            hit = pos;

            if (_target.precision() > distance)
            { break; }
        }
        // -----------------------------------------------
        adjacency_ptrs_t range;
        _store.adjacencyByPBorders(range, aim, side3);
        if (range.empty())
            break;
        // -----------------------------------------------
        side3 -= side_decrease_step;
        // -----------------------------------------------
        Control controls;
        weightedMeanControls(aim, range, controls, Point{});
        // -----------------------------------------------
        if (actionRobo(aim, controls, pos))
            ++w_means_complexity;
        // -----------------------------------------------
        next_distance = boost_distance(pos, aim);
    } while (next_distance < distance);
    // -----------------------------------------------
    CINFO(_T("prec: ") << distance << std::endl << _T("w_means complexity: ") << w_means_complexity << std::endl);
    return w_means_complexity;
}
//------------------------------------------------------------------------------
bool RoboPos::LearnMoves::weightedMeanULAdjs(IN  const Point &aim, OUT Record *pRec,
                                             OUT Control &lower_controls,
                                             OUT Control &upper_controls,
                                             OUT distance_t &delta)
{
    if (!pRec) return false;

    Point min, max, lo_hit, hi_hit;
    // ------------------------------------------------
    lower_controls.clear();
    upper_controls.clear();
    // ------------------------------------------------
    adjacency_ptrs_t range;
    _store.adjacencyPoints(range, aim, side3);

    ClosestPredicate cp(aim);
    auto it_min = boost::range::min_element(range, cp);
    if (it_min == range.end())
        return false;

    *pRec = (**it_min);
    range.clear();
    // ------------------------------------------------
    min = Point(aim.x - side3, pRec->hit.y - side3);
    max = Point(aim.x + side3, pRec->hit.y);

    _store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (range.empty())
        return false;
    // ------------------------------------------------
    weightedMeanControls(aim, range, lower_controls, lo_hit);
    // ------------------------------------------------
    min = Point(aim.x - side3, pRec->hit.y);
    max = Point(aim.x + side3, pRec->hit.y + side3);

    _store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (range.empty())
    { return false; }
    // ------------------------------------------------
    weightedMeanControls(aim, range, upper_controls, hi_hit);
    // ------------------------------------------------
    delta = boost_distance(lo_hit, hi_hit);
    return true;
}
//------------------------------------------------------------------------------

