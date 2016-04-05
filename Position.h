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
#define _HAND_TEST_CONSOLE_PRINTF

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
  void  testCover (HandMoves::Store &store, Hand &hand, HandMoves::trajectories_t &trajectories);
  //------------------------------------------------------------------------------
  typedef std::map
  < Hand::MusclesEnum /* -------------muscle---*/,
    std::pair< Hand::frames_t /*---min lasts---*/,
               Hand::frames_t /*---max lasts---*/ >
  > borders_t;
  //------------------------------------------------------------------------------

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
      Hand::MusclesEnum  muscle;
      std::vector<Point>  shifting;
      Point  center;
      //------------------------------------------
      MainDirection (Hand::MusclesEnum m, Hand &hand, const Point &base) :
        muscle (m),
        center (hand.jointPosition (jointByMuscle (m)))
      {
        // Point base = hand.position;
        std::list<Point>  trajectory;
        //-----Must be !!! EACH !!!--------
        auto oldEach = hand.visitedSaveEach;
        hand.visitedSaveEach = 1U;
        hand.move (m, hand.maxMuscleLast (m), trajectory /*, !!! */);
        hand.visitedSaveEach = oldEach;
        //---------------------------------
        for ( auto pt : trajectory )
        {
          shifting.push_back (Point (pt.x - trajectory.front ().x,
                              pt.y - trajectory.front ().y));
        }
      }
      //------------------------------------------
      bool operator== (Hand::MusclesEnum m) const
      { return  (muscle == m); }
      bool operator!= (Hand::MusclesEnum m) const
      { return  (muscle != m); }
    };
    //------------------------------------------
    std::list<MainDirection>  mainDirections;
    Point hand_base;
    //------------------------------------------
  public:
    DirectionPredictor (Hand &hand)
    {
      hand.SET_DEFAULT;
      hand_base = hand.position;

      for ( auto j : hand.joints_ )
      {
        hand.SET_DEFAULT;

        hand.set (j, { 100. });
        mainDirections.push_back (MainDirection (muscleByJoint (j, true), hand, hand.position));

        hand.set (j, { 0. });
        mainDirections.push_back (MainDirection (muscleByJoint (j, false), hand, hand.position));
      }
      // for ( auto m : hand.muscles_ )
      //   mainDirections.push_back (MainDirection (m, hand, hand_base));
    }
    //------------------------------------------
    void  predict (IN Hand::Control control, OUT Point &end) const
    {
      for ( auto m : muscles )
        for ( auto &md : mainDirections ) // !!!!!!!!!!!!
          if ( md == (control.muscle & m) && (control.last < md.shifting.size ()) )
          {
            end.x += md.shifting[control.last].x;
            end.y += md.shifting[control.last].y;
          }
    }

    void  predict (IN  Hand::MusclesEnum m,
                   IN  Hand::frames_t l,
                   OUT Point &end) const
    {
      predict (Hand::Control (m, 0U, l), end);
    }

    void  measure (IN  const Point &aim,
                   OUT HandMoves::controling_t &controls) const
    {
      Point base = hand_base;
      /*
      *  У нас есть 4 линейки, их надо сопоставить так,
      *  чтобы приблизить aim.
      */

      /*
      *  Система линейных уравнений
      *
      *
      */

      if ( base.x > aim.x )
      {}
      if ( base.y > aim.y )
      {}

      while ( base.x < aim.x )
      {}
      while ( base.y < aim.y )
      {}

      for ( auto &md : mainDirections )
      {
        int  j = 0, const_t = 5;
        Hand::Control  control (md.muscle, 0U, j - const_t);
      }
    }
    //------------------------------------------
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
    /*  Примерное количество точек
     *  от базового состояния до
     *  крайнего положения в каждую
     *  из возможных сторон движения.
     */
     // Hand::frames_t   count_points = 20U;
     //==============================================

     /* stage_2 */
     //==============================================
    double            detail_distance = 0.005;
    /* Время преодоления инерции */
    Hand::frames_t   lasts_init_value2 = 1U;
    /* Изменение шага времени */
    Hand::frames_t   lasts_incr_value2 = 1U;
    // Hand::frames_t   count_points = 20U;
     //==============================================
    borders_t  borders_;
    // !!! НУЖНО ТОРМОЗИТЬ УЖЕ НА ЭТОЙ СТАДИИ !!!
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

    void  STAGE2 (HandMoves::Store &store, Hand &hand, RecTarget &target);
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
};
#endif // _POSITION_H_
