#pragma once

#include "Robo.h"

namespace RoboPos {
//------------------------------------------------------------------------------
class DirectionPredictor final
{
#ifdef MDIR_OLD
    struct MainDirection final
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
#else //!MDIR_OLD
public:
    DirectionPredictor(IN Robo::RoboI&) {}
    Point predict(IN Robo::Control) { return {}; }
#endif //!MDIR_OLD
};
} // namespace RoboPos
