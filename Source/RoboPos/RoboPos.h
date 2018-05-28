#include "StdAfx.h"

#ifndef  _POSITION_H_
#define  _POSITION_H_

#include "WindowHeader.h"
#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"

#include "Old\RoboPosMDirections.h"
#include "ConfigJSON.h"


namespace RoboPos {
//------------------------------------------------------------------------------
using distance_t = double;
struct lasts_border_t { Robo::frames_t min_lasts, max_lasts; };
using borders_t = std::unordered_map<Robo::muscle_t, lasts_border_t>;
//------------------------------------------------------------------------------
void defineRobotBorders(const Robo::RoboI&, Robo::frames_t, borders_t&);
void defineTargetBorders(const RecTarget&, const RoboMoves::Store&, distance_t, borders_t&);
//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
}
#endif // _POSITION_H_
