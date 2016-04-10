#include "StdAfx.h"

#ifndef  _POSITION_H_
#define  _POSITION_H_
//------------------------------------------------------------------------------
#define HAND_VER 2
#if   HAND_VER == 1
#include "Hand.h"
using namespace OldHand;
#elif HAND_VER == 2
#include "NewHand.h"
using namespace NewHand;
#include "HandMuscles.h"
#endif

//------------------------------------------------------------------------------
#include "MyWindow.h"
#include "target.h"

#include "HandMovesStore.h"

namespace Positions
{
//#define _HAND_TEST_CONSOLE_PRINTF

  //void  /*HandMoves::*/testCover (Store &store, Hand &hand,
  //                            Hand::MusclesEnum muscles /* recursive */,
  //                            int recursive,
  //                            std::list<int> step,
  //                            std::list<Point> trajectory1)
  //{
  //  // hand.reset ();
  //  hand.SET_DEFAULT;
  //
  //  Point hand_base = hand.position;
  //
  //  for ( int i = step[recursive]; i < hand.muscles_.size (); ++i )
  //  {
  //    auto muscle_i = hand.muscles_[i];
  //    if ( !(muscle_i & muscles) )
  //    {
  //      trajectory_t  trajectory;
  //
  //      /* Попробуем его варьировать по длительности */
  //      for ( Hand::frames_t last_i : boost::irange (1U, hand.maxMuscleLast (muscle_i)) )
  //      {
  //        hand.move (muscle_i, last_i, trajectory);
  //        store.insert (Record (hand.position, hand_base, hand.position,
  //                             { muscle_i }, { 0 }, { last_i }, 1U,
  //                             trajectory)
  //                     );
  //
  //        std::copy (trajectory.begin (), trajectory.end (), trajectory1.begin ());
  //        if ( step.size < recursive + 1 )
  //        {}
  //
  //        testCover (store, hand, muscles & muscle_i, recursive + 1, step, trajectory1);
  //
  //      } // end for
  //    } // end if
  //  } // end for
  //}


  //------------------------------------------------------------------------------
  void  testCover (HandMoves::Store &store, Hand &hand,
                   HandMoves::trajectories_t &trajectories);
  //------------------------------------------------------------------------------
  typedef std::map
  < Hand::MusclesEnum /* -------------muscle---*/,
    std::pair< Hand::frames_t /*---min lasts---*/,
               Hand::frames_t /*---max lasts---*/ >
  > borders_t;
  //------------------------------------------------------------------------------

  class LinearOperator
  {
    double *coefs_ = NULL;
    Point  min_, max_;
  public:
    LinearOperator () {}
    void solveQR (HandMoves::Store &store, const Point &aim, double side) throw (...);

    ~LinearOperator ()
    {
      delete[] coefs_;
    }
  };

  // class PredictedDirection
  // {
  //   Hand::MusclesEnum  muscle;
  //   Hand::frames_t     last;
  //   // trajectory_t trajectory; ???
  //   // Point center;
  // 
  //   PredictedDirection ():
  //     muscle (Hand::EmptyMov), last (0U), end (0,0) {}
  //   PredictedDirection (Hand::MusclesEnum m, Hand::frames_t l, Point e) :
  //     muscle (m), last (l), end (e) {}
  // 
  // public:
  //   Point end;
  // };
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
    std::map<Hand::MusclesEnum, MainDirection>  main_directions;
    Point hand_base;
    //------------------------------------------
  public:
    DirectionPredictor (IN Hand &hand);
    //------------------------------------------
    void  predict (IN Hand::Control control, OUT Point &end);
    void  predict (IN  Hand::MusclesEnum m,
                   IN  Hand::frames_t    l,
                   OUT Point &end)

    { predict (Hand::Control (m, 0U, l), end); }
    void  measure (IN Hand &hand, IN  const Point &aim,
                   OUT HandMoves::controling_t &controls);
    //------------------------------------------
  private:
    bool  shifting_gt (Hand::MusclesEnum m, unsigned int &inx, const Point &aim);
    bool  shifting_ls (Hand::MusclesEnum m, unsigned int &inx, const Point &aim);
  };

  class LearnMovements
  {
    /*  Количество точек, в окресности искомой точки.
     *
     *  Что я могу варьировать?
     *
     *  v  1. Длительность работы каждого мускула
     *  v  2. Время старта каждого мускула
     *  v  3. Тормозить мускулом? (прерывать '1' входа на которкое время)
     *
     *  ?  4. Может быть !1 перелом! траектории (полная остановка)
     *  Он может нам понадобиться в зависимости от законов движения
     *  разных мускулов, если один до второго не успевает ...
     *  (?позже включить??)
     *
     */

     /* stage_1 */
     //==============================================
     /* Желаемое расстояние между конеными точками
      * чернового покрытия
      */
    double             draft_distance = 0.07;
    /* Время преодоления инерции */
    Hand::frames_t   lasts_init_value1 = 50U;
    /* Изменение шага времени */
    Hand::frames_t   lasts_incr_value1 = 10U;
     //==============================================

     /* stage_2 */
     //==============================================
    double            detail_distance = 0.005;
    /* Время преодоления инерции */
    Hand::frames_t   lasts_init_value2 = 1U;
    /* Изменение шага времени */
    Hand::frames_t   lasts_incr_value2 = 3U;
     //==============================================

    /* stage_3 */
    //==============================================

    //==============================================

  public:
    // LearnMovements () : lasts_incr_value1 (10U), lasts_incr_value2 (3U) {}
    // LearnMovements (HandMoves::Store &store, Hand &hand, RecTarget &target)
    // {
    //   lasts_incr_value1 = 0U;
    //   for ( auto m : hand.muscles_ )
    //     lasts_incr_value1 += hand.maxMuscleLast (m);
    //   lasts_incr_value1 /= (Hand::frames_t) hand.muscles_.size ();
    //   lasts_incr_value1 /= count_points;
    // }

    void  STAGE_1 (HandMoves::Store &store, Hand &hand, RecTarget &target);
    void  STAGE_2 (HandMoves::Store &store, Hand &hand, RecTarget &target);
    //------------------------------------------------------------------------------
    /* грубое покрытие всего рабочего пространства */
    void  testStage1 (HandMoves::Store &store, Hand &hand, RecTarget &target);
    /* Покрытие всей мишени не слишком плотно */
    void  testStage2 (HandMoves::Store &store, Hand &hand, RecTarget &target);
    /* Попадание в оставшиеся непокрытыми точки мишени */
    void  testStage3 (HandMoves::Store &store, Hand &hand, RecTarget &target, std::list<Point> &uncovered);
    //------------------------------------------------------------------------------

    void  testCoverTarget (HandMoves::Store &store, Hand &hand, RecTarget &target);

    void  getTargetCenter (HandMoves::Store &store, Hand &hand, Point &center);

    //------------------------------------------------------------------------------
    bool  tryToHitTheAim (HandMoves::Store &store, Hand &hand, const Point &aim, size_t tries, double precision);
    //------------------------------------------------------------------------------
  };
  void testGradientMethod (HandMoves::Store &store, Hand &hand, const Point &aim);
};
#endif // _POSITION_H_
