#include "StdAfx.h"

#ifndef  _POSITION_H_
#define  _POSITION_H_

#include "WindowHeader.h"
#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"

#include "RoboPosLinear.h"
#include "RoboPosMDirections.h"
#include "ConfigJSON.h"


namespace RoboPos {

// #define _HAND_TEST_CONSOLE_PRINTF
// #define _DEBUG_PRINT
#define _DEBUG_PRINT_RES

//------------------------------------------------------------------------------
void  testCover(RoboMoves::Store&, Robo::RoboI&, Robo::Trajectories&);
//------------------------------------------------------------------------------
using distance_t = double;
using borders_t = std::map < Robo::muscle_t    /* -------------muscle---*/,
    std::pair< Robo::frames_t /*---min lasts---*/,
    Robo::frames_t /*---max lasts---*/ >
>;
//------------------------------------------------------------------------------
void defineRobotBorders(IN const Robo::RoboI&, IN Robo::frames_t, OUT borders_t&);
void defineTargetBorders(IN const RecTarget&, IN const RoboMoves::Store&, IN distance_t, IN OUT borders_t&);
//------------------------------------------------------------------------------
struct Counters
{
    int count = 0;
    int count_TP = 0;
    int count_FP = 0;
    int count_TN = 0;
    int count_FN = 0;

    void incr(bool model, bool real)
    {
        ++count;
        if (model & real)
            ++count_TP;
        else if (!model & !real)
            ++count_TN;
        else if (model & !real)
            ++count_FP;
        else if (!model & real)
            ++count_FN;
    }
    void fill(Robo::RoboI &robo, RecTarget &target, Robo::Control &controls, const Point &end)
    {
        //-------------------------------------------------------------
        Robo::Trajectory trajectory;
        robo.reset();
        robo.move(controls, trajectory);
        //-------------------------------------------------------------
        bool model = target.contain(end);
        bool real = target.contain(robo.position());
        incr(model, real);
    }
    void print()
    {
        tcout << _T("count = ") << count << std::endl;
        tcout << _T("\t< T >\t < F >") << std::endl;
        tcout << _T("< P >\t") << count_TP << _T("\t") << count_FP << std::endl;
        tcout << _T("< N >\t") << count_TN << _T("\t") << count_FN << std::endl;
    }
    void clear()
    {
        count = 0;
        count_TP = 0;
        count_FP = 0;
        count_TN = 0;
        count_FN = 0;
    }
};
//------------------------------------------------------------------------------
}
#endif // _POSITION_H_
