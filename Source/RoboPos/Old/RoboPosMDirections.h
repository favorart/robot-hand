#include "StdAfx.h"

#ifndef   _DIRECT_PREDICT_H_
#define   _DIRECT_PREDICT_H_

#include "WindowHeader.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"

//------------------------------------------------------------------------------
namespace RoboPos {

class MainDirections
{
#ifdef MDIR_OLD
    struct Direction
    {
        Robo::muscle_t      muscle = Robo::MInvalid;
        //Point               center{};
        std::vector<Point>  shifts{};
        //double              min_radius = 0.;
        //double              max_radius = 0.;
        //------------------------------------------
        Direction(Robo::muscle_t, Robo::Trajectory&, Point);
        //------------------------------------------
        bool  operator== (const Direction& d) const { return (muscle == d.muscle); }
        bool  operator!= (const Direction& d) const { return (muscle != d.muscle); }
        bool  operator== (Robo::muscle_t   m) const { return (muscle == m); }
        bool  operator!= (Robo::muscle_t   m) const { return (muscle != m); }
    };

    Point _base{};
    std::vector<Direction> _directions{};
    std::vector<Point> _shifts{};
    const Robo::RoboI &_owner;
    //------------------------------------------
    bool  shifting_gt(Robo::muscle_t m, unsigned int &inx, const Point &aim);
    bool  shifting_ls(Robo::muscle_t m, unsigned int &inx, const Point &aim);
    //------------------------------------------
    MainDirections(IN const Robo::RoboI &robo) : _owner(robo), _base(robo.position() /*was reset()*/)
    { _directions.reserve(_owner.musclesCount()); }

public:
    Robo::Control measure(const Point &aim);
    /// зависимые сочленения
    Point predict(const Robo::Control &controls);
    /// независимые сочленения
    Point predict(Robo::muscle_t muscle, Robo::frames_t last);
    //------------------------------------------
    friend RoboPos::MainDirections MainDirectionsFactory(IN Robo::RoboI&);
#endif // MDIR_OLD

    Robo::Control measure(const Point &aim) { return {}; }
    Point predict(const Robo::Control &controls) { return {}; }
    Point predict(Robo::muscle_t muscle, Robo::frames_t last) { return {}; }
    friend RoboPos::MainDirections MainDirectionsFactory(IN Robo::RoboI&) { return {}; }
};
} // namespace RoboPos

//------------------------------------------------------------------------------
#endif // _DIRECT_PREDICT_H_
