#include "StdAfx.h"

#ifndef   _DIRECT_PREDICT_H_
#define   _DIRECT_PREDICT_H_

#include "WindowHeader.h"
#include "target.h"
#include "Store.h"

//------------------------------------------------------------------------------
namespace Positions
{
  class  DirectionPredictor
  {
    //------------------------------------------
    struct  MainDirection
    {
      //------------------------------------------
      Hand::MusclesEnum   muscle;

      std::vector<Point>  shifts;
      Point               center;

      double            min_radius;
      double            max_radius;
      //------------------------------------------
      MainDirection () {}
      MainDirection (Hand::MusclesEnum m, Hand &hand) :
        muscle (m),
        center (hand.jointPosition (jointByMuscle (m)))
      {
        std::list<Point>  trajectory;
        //-----Must be !!! EACH !!!--------
        auto oldEach = hand.visitedSaveEach;
        hand.visitedSaveEach = 1U;
        hand.move (m, hand.maxMuscleLast (m), trajectory /*, !!! */);
        hand.visitedSaveEach = oldEach;
        //---------------------------------
        Point &front = trajectory.front ();
        max_radius = boost_distance (center, front);
        min_radius = boost_distance (center, front);
        //---------------------------------
        for ( auto pt : trajectory )
        {
          double radius = boost_distance (center, pt);
          //---------------------------------
          if ( radius > max_radius ) max_radius = radius;
          if ( radius < min_radius ) min_radius = radius;
          //---------------------------------
          shifts.push_back (Point (pt.x - front.x,
                                   pt.y - front.y));
        }
      }
      //------------------------------------------
      bool  operator<  (const MainDirection &md) const
      { return  (muscle  < md.muscle); }
      bool  operator== (const MainDirection &md) const
      { return  (muscle == md.muscle); }
      bool  operator!= (const MainDirection &md) const
      { return  (muscle != md.muscle); }

      bool  operator== (Hand::MusclesEnum m) const
      { return  (muscle == m); }
      bool  operator!= (Hand::MusclesEnum m) const
      { return  (muscle != m); }
    };
    //------------------------------------------
    std::map<Hand::MusclesEnum,MainDirection>  main_directions;
    Point hand_base;
    //------------------------------------------
  public:
    DirectionPredictor (IN Hand &hand);
    //------------------------------------------
    void  predict (IN  Hand::Control control, OUT Point &end);
    void  predict (IN  Hand::MusclesEnum   m,
                   IN  Hand::frames_t      l, OUT Point &end)
    { predict (Hand::Control (m, 0U, l), end); }
    void  measure (IN  Hand &hand, IN const Point &aim,
                   OUT HandMoves::controling_t &controls);
    //------------------------------------------
  private:
    bool  shifting_gt (Hand::MusclesEnum m, unsigned int &inx, const Point &aim);
    bool  shifting_ls (Hand::MusclesEnum m, unsigned int &inx, const Point &aim);
  };
};
//------------------------------------------------------------------------------
#endif // _DIRECT_PREDICT_H_
