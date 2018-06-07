#include "StdAfx.h"
#include "RoboPos.h"
#include "RoboLearnMoves.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
void RoboPos::LearnMoves::weightedMeanControls(IN  const Point &aim,
                                               IN  const adjacency_ptrs_t &range,
                                               OUT Control &controls,
                                               OUT double  *weight)
{
    // -----------------------------------------------
    double sum_range_distances = 0.;
    for (auto &pRec : range)
    { sum_range_distances += boost_distance(aim, pRec->hit); }
    // -----------------------------------------------
    if (weight) { *weight = (sum_range_distances / range.size()); }
    // -----------------------------------------------
    /* generate controls */
    for (auto &pRec : range)
        for (auto &c : pRec->controls)
        {
            auto it_muscle = std::find(controls.begin(), controls.end(), c.muscle);

            /* взвешенное НЕСМЕЩЁННОЕ cреднее арифметическое */
            double w = boost_distance(aim, pRec->hit) / sum_range_distances;
            frames_t last = frames_t(c.lasts * w);
            if (it_muscle == controls.end())
            {
                last = last ? last : 1; // ???
                controls.append({ c.muscle, 0U /* ??? c.start * weight */, last });
            }
            else
            {
                it_muscle->lasts += last;
                // ??? it->start += c.start * weight;
            }
        } // end for-for
    // ----------------------------------------------
    /* controls check for correctness: opposite muscles work time */
    _robo.controlsValidate(controls);
    // ----------------------------------------------
}
//------------------------------------------------------------------------------
size_t RoboPos::LearnMoves::weightedMean(IN const Point &aim, OUT Point &hit)
{
    size_t w_means_complexity = 0U;
    double side_ = _stage3_params.side;
    // -----------------------------------------------
    auto p = _store.getClosestPoint(aim, _stage3_params.side);
    if (!p.first)
        throw std::runtime_error{ "weightedMean: Empty adjacency" };
    // -----------------------------------------------
    // HandMoves::controling_t  controls{ rec.controls };
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

            if (_precision > distance)
            { break; }
        }
        // -----------------------------------------------
        adjacency_ptrs_t range;
        _store.adjacencyByPBorders(range, aim, side_);
        if (range.empty())
        { break; }
        // -----------------------------------------------
        side_ -= _stage3_params.side_decrease_step;
        // -----------------------------------------------
        Control controls;
        weightedMeanControls(aim, range, controls);
        // -----------------------------------------------
        if (actionRobo(aim, controls, pos))
        { ++w_means_complexity; }
        // -----------------------------------------------
        next_distance = boost_distance(pos, aim);
        // -----------------------------------------------
    } while (next_distance < distance);
    // -----------------------------------------------
    tcout << _T("prec: ") << distance << std::endl;
    tcout << _T("w_means complexity: ")
          << w_means_complexity
          << std::endl << std::endl;
    // -----------------------------------------------
    return w_means_complexity;
}
//------------------------------------------------------------------------------
bool RoboPos::LearnMoves::weightedMeanULAdjs(IN  const Point &aim, OUT Record *pRec,
                                             OUT Control &lower_controls,
                                             OUT Control &upper_controls,
                                             OUT double  &lower_distance,
                                             OUT double  &upper_distance)
{
    if (!pRec) { return  false; }

    Point  min, max;
    // ------------------------------------------------
    lower_controls.clear();
    upper_controls.clear();
    // ------------------------------------------------
    adjacency_ptrs_t range;
    _store.adjacencyPoints(range, aim, _stage3_params.side);

    ClosestPredicate cp(aim);
    auto it_min = boost::range::min_element(range, cp);
    if (it_min == range.end())
    { return  false; }

    *pRec = (**it_min);
    range.clear();
    // ------------------------------------------------
    min = Point(aim.x - _stage3_params.side, pRec->hit.y - _stage3_params.side);
    max = Point(aim.x + _stage3_params.side, pRec->hit.y);

    _store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (range.empty())
    { return false; }
    // ------------------------------------------------
    weightedMeanControls(aim, range, lower_controls, &lower_distance);
    // ------------------------------------------------
    min = Point(aim.x - _stage3_params.side, pRec->hit.y);
    max = Point(aim.x + _stage3_params.side, pRec->hit.y + _stage3_params.side);

    _store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (range.empty())
    { return false; }
    // ------------------------------------------------
    weightedMeanControls(aim, range, upper_controls, &upper_distance);
    // ------------------------------------------------
    return true;
}
//------------------------------------------------------------------------------

