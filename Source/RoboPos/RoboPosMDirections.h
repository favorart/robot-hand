#include "StdAfx.h"

#ifndef   _DIRECT_PREDICT_H_
#define   _DIRECT_PREDICT_H_

#include "WindowHeader.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"

//------------------------------------------------------------------------------
namespace RoboPos {
namespace NewHand {

class MainDirections
{
    struct Direction
    {
        Robo::muscle_t      muscle = Robo::MInvalid;
        Point               center{};
        std::vector<Point>  shifts{};
        double              min_radius = 0.;
        double              max_radius = 0.;
        //------------------------------------------
        Direction(Robo::muscle_t, Robo::Trajectory&, Point);
        //------------------------------------------
        bool  operator== (const Direction& d) const { return (muscle == d.muscle); }
        bool  operator!= (const Direction& d) const { return (muscle != d.muscle); }
        bool  operator== (Robo::muscle_t   m) const { return (muscle == m); }
        bool  operator!= (Robo::muscle_t   m) const { return (muscle != m); }
    };

    Point base{};
    std::vector<Direction> directions{};
    const Robo::RoboI &owner;
    //------------------------------------------
    bool  shifting_gt(Robo::muscle_t m, unsigned int &inx, const Point &aim);
    bool  shifting_ls(Robo::muscle_t m, unsigned int &inx, const Point &aim);
    //------------------------------------------
    MainDirections(IN const Robo::RoboI &robo) : owner(robo), base(robo.position())
    { directions.reserve(owner.musclesCount()); }

public:
    Robo::Control measure(const Point &aim);

    Point predict(const Robo::Control &control);
    Point predict(Robo::muscle_t muscle, Robo::frames_t last)
    { return predict(Robo::Control{ { muscle, 0, last } }); }
    //------------------------------------------
    friend RoboPos::NewHand::MainDirections MainDirectionsFactory(IN Robo::RoboI&);
};

}
}
//------------------------------------------------------------------------------
#endif // _DIRECT_PREDICT_H_
