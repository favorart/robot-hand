#pragma once
#include "StdAfx.h"

#ifndef   _DIR_P_H_
#define   _DIR_P_H_

#include "WindowHeader.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "Robo.h"

//------------------------------------------------------------------------------
namespace RoboPos
{
class DirectionPredictor
{
    struct MainDirection
    {
        Robo::muscle_t muscle;
        std::vector<Point> shifts;
        //------------------------------------------
        MainDirection() {}
        MainDirection(IN Robo::muscle_t m, IN Robo::RoboI &robo);
        //------------------------------------------
        bool  operator<  (const MainDirection &md) const
        { return  (muscle  < md.muscle); }
        bool  operator== (const MainDirection &md) const
        { return  (muscle == md.muscle); }
        bool  operator!= (const MainDirection &md) const
        { return  (muscle != md.muscle); }

        bool  operator== (Robo::muscle_t m) const
        { return  (muscle == m); }
        bool  operator!= (Robo::muscle_t m) const
        { return  (muscle != m); }
    };

    std::vector<MainDirection> _directions;
    Point _base_pos;
public:
    DirectionPredictor(IN Robo::RoboI &robo);
    Point predict(IN Robo::Control controls);
};
};
//------------------------------------------------------------------------------
#endif // _DIR_P_H_
