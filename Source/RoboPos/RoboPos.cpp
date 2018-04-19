#include "StdAfx.h"
#include "RoboPos.h"


using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
void specifyBordersByRecord(IN const Record &rec, IN OUT borders_t &borders)
{
    for (const auto &c : rec.controls)
    {
        auto it = borders.find(c.muscle);
        if (it != borders.end())
        {
            if (c.last < it->second.min_lasts)
                it->second.min_lasts = c.last;
            if (c.last > it->second.max_lasts)
                it->second.max_lasts = c.last;
        }
        else { borders[c.muscle] = { c.last, c.last }; }
    }
}
//------------------------------------------------------------------------------
/// задать граничные длительности работы мускулов робота
void RoboPos::defineRobotBorders(IN const RoboI &robo, IN frames_t min_lasts, OUT borders_t &borders)
{
    for (muscle_t muscle = 0; muscle < robo.musclesCount(); ++muscle)
    { borders[muscle] = { min_lasts, robo.muscleMaxLast(muscle) }; }
}

/// Статистичеки найти приблизительную границу мишени по длительности работы мускулов
void RoboPos::defineTargetBorders(IN const RecTarget &target, IN const Store &store, IN distance_t side, IN OUT borders_t &borders)
{
    for (const auto &rec : store)
        if (target.contain(rec.hit))
            specifyBordersByRecord(rec, borders);

    adjacency_ptrs_t range;
    store.adjacencyPoints(range, target.min(), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec, borders);

    range.clear();
    store.adjacencyPoints(range, Point(target.min().x, target.max().y), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec, borders);

    range.clear();
    store.adjacencyPoints(range, Point(target.max().x, target.min().y), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec, borders);

    range.clear();
    store.adjacencyPoints(range, target.max(), side);
    for (auto p_rec : range)
        specifyBordersByRecord(*p_rec, borders);
}
//------------------------------------------------------------------------------

