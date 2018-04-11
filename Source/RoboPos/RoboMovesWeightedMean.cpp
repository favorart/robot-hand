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
            frames_t last = frames_t(c.last * w);
            if (it_muscle == controls.end())
            {
                last = last ? last : 1; // ???
                controls.append({ c.muscle, 0U /* ??? c.start * weight */, last });
            }
            else
            {
                it_muscle->last += last;
                // ??? it->start += c.start * weight;
            }
        } // end for-for
    // ----------------------------------------------
    /* controls check for correctness: opposite muscles work time */
    robo.controlsValidate(controls);
    // ----------------------------------------------
}
//------------------------------------------------------------------------------
size_t RoboPos::LearnMoves::weightedMean(IN const Point &aim, OUT Point &hit, IN bool verbose)
{
    size_t w_means_complexity = 0U;
    double side_ = stage3.side;
    // -----------------------------------------------
    const Record &rec = store.ClothestPoint(aim, stage3.side);
    // -----------------------------------------------
    // HandMoves::controling_t  controls{ rec.controls };
    Point pos = rec.hit;
    // -----------------------------------------------
    double distance = boost_distance(aim, pos),
      next_distance = distance;
    // -----------------------------------------------
    robo.reset();
    do
    {
        if (next_distance < distance)
        {
            distance = next_distance;
            hit = pos;

            if (precision > distance)
            { break; }
        }
        // -----------------------------------------------
        adjacency_ptrs_t range;
        store.adjacencyByPBorders(range, aim, side_);
        if (range.empty())
        { break; }
        // -----------------------------------------------
        side_ -= stage3.side_decrease_step;
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
    if (verbose)
    {
        tcout << _T("prec: ") << distance << std::endl;
        tcout << _T("w_means complexity: ")
              << w_means_complexity
              << std::endl << std::endl;
    }
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
    store.adjacencyPoints(range, aim, stage3.side);

    ClosestPredicate cp(aim);
    auto it_min = boost::range::min_element(range, cp);
    if (it_min == range.end())
    { return  false; }

    *pRec = (**it_min);
    range.clear();
    // ------------------------------------------------
    min = Point(aim.x - stage3.side, pRec->hit.y - stage3.side);
    max = Point(aim.x + stage3.side, pRec->hit.y);

    store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (range.empty())
    { return false; }
    // ------------------------------------------------
    weightedMeanControls(aim, range, lower_controls, &lower_distance);
    // ------------------------------------------------
    min = Point(aim.x - stage3.side, pRec->hit.y);
    max = Point(aim.x + stage3.side, pRec->hit.y + stage3.side);

    store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (range.empty())
    { return false; }
    // ------------------------------------------------
    weightedMeanControls(aim, range, upper_controls, &upper_distance);
    // ------------------------------------------------
    return true;
}
//------------------------------------------------------------------------------

