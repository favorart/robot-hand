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

namespace Positions
{

  /*  Количество точек, в окресности искомой точки.

  Что я могу варьировать?

  v  1. Длительность работы каждого мускула
  v  2. Время старта каждого мускула
  ?  3. Тормозить мускулом? (снимать от движущегося начинающий двигаться)
  ?  4. Может быть !1 перелом! траектории (полная остановка)
  Он может нам понадобиться в зависимости от законов движения
  разных мускулов, если один до второго не успевает ...
  (?позже включить??)
  */

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
  //        storeInsert (store,
  //                     Record (hand.position, hand_base, hand.position,
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

  using namespace std;
  using namespace HandMoves;
  //------------------------------------------------------------------------------
  static void  testCover (Store &store, Hand &hand, trajectories_t &trajectories)
  {
    hand.reset ();
    hand.SET_DEFAULT;

    Point hand_base = hand.position;

    /*    Покрыть всё в одно движение
    *
    *    Начать двигать какой-то мышцой (варьировать длительность),
    *    К её движению сложить все комбинации остальных мышц.
    *
    */

    /* Возьмём первый мускул наугад */
    for ( auto muscle_i : hand.muscles_ | boost::adaptors::sliced (3, 4) )
    {
      // if ( !(muscle_i & muscles) && muscleValidAtOnce(muscle_i | muscles) )
      {
        trajectory_t  trajectory;
        /* Попробуем его варьировать по длительности */
        for ( Hand::frames_t last_i : boost::irange<Hand::frames_t> (1U, hand.maxMuscleLast (muscle_i) / 2, 10) )
        {
          Hand::Control control_i (muscle_i, 0U, last_i);
          // hand.move ({ control_i });

          for ( auto muscle_j : hand.muscles_ )
            if ( (muscle_i != muscle_j) && muscleValidAtOnce (muscle_i | muscle_j) )
            {
              for ( Hand::frames_t start_j : boost::irange<Hand::frames_t> (1U, last_i, 10) ) // ?? + inertial_lasts
              {
                for ( Hand::frames_t last_j : boost::irange<Hand::frames_t> (1U, hand.maxMuscleLast (muscle_j) / 2, 10) )
                {
                  Hand::Control control_j (muscle_j, start_j, last_j);
                  hand.move ({ control_i, control_j }, &trajectory);

                  // auto& index = store.get<Record::ByC> ();
                  // 
                  // auto& index = store.get<Record::ByP> ();
                  // index.equal_range (hand.position);

                  storeInsert (store, Record (hand.position, hand_base, hand.position,
                                              { muscle_i, muscle_j }, { 0, start_j }, { last_i, last_j }, 2U,
                                              trajectory));
                  // trajectories.push_back (make_shared<trajectory_t> (trajectory));
                }
              }
            }
        }
      }
      // for ( auto muscle_j : hand.muscles_ )
      // {
      //   if ( !(muscle_j & muscles) && muscleValidAtOnce (muscle_i | muscles) )
      //   {
      //     for ( Hand::frames_t last_i : boost::irange<Hand::frames_t> (1U, 126U, 10) ) //hand.maxMuscleLast (muscle_i)) )
      //     {
      //       hand.move (muscle_i | muscle_j, last_i, trajectory);
      //       storeInsert (store,
      //                    Record (hand.position, hand_base, hand.position,
      //                    { muscle_i }, { 0 }, { last_i, 126U - last_i }, 1U,
      //                    trajectory)
      //                    );
      //     }
      //   } // end if
      // } // end for
      // 
      //   // trajectories.push_back (make_shared<trajectory_t> (trajectory));
      //   /*  Теперь в каждый такт длительности первого мускула,
      //    *  включим второй и будет варьировать его длительность
      //    */
      //   testCover (store, hand, muscles | muscle_i, trajectories);
      // 
      // } // end for
      //} // end if
    } // end for
  }
  //------------------------------------------------------------------------------
  static double  prediction ()
  {
    return 0.;
  }

  static  void  testCoverTarget (Store &store, Hand &hand,
                         RecTarget &target,
                         std::list<std::list<Point>> &trajectories)
  {
    hand.reset ();
    hand.SET_DEFAULT;

    Point hand_base = hand.position;

    /*    Покрыть всё в одно движение
    *
    *    Начать двигать какой-то мышцой (варьировать длительность),
    *    К её движению сложить все комбинации остальных мышц.
    *
    */

    double  x_leftBorder = target.Min ().x,
            x_rightBorder = target.Max ().x,
            y_bottomBorder = target.Max ().y;

    // Point center

    /* Возьмём первый мускул наугад */
    for ( auto muscle_i : hand.muscles_ )
    {
      // if ( !(muscle_i & muscles) && muscleValidAtOnce(muscle_i | muscles) )
      {
        /* Попробуем его варьировать по длительности */
        for ( Hand::frames_t last_i : boost::irange (1U, hand.maxMuscleLast (muscle_i)) )
        {
          Hand::Control control_i (muscle_i, 0U, last_i);
          // hand.move ({ control_i });

          for ( auto muscle_j : hand.muscles_ )
            if ( (muscle_i != muscle_j) && muscleValidAtOnce (muscle_i | muscle_j) )
            {
              for ( Hand::frames_t start_j : boost::irange<Hand::frames_t> (1U, last_i) ) // ?? + inertial_lasts
              {
                for ( Hand::frames_t last_j : boost::irange<Hand::frames_t> (1U, hand.maxMuscleLast (muscle_j), 5) )
                {
                  Hand::Control control_j (muscle_j, start_j, last_j);

                  hand.SET_DEFAULT;
                  trajectory_t  trajectory;
                  hand.move ({ control_i, control_j }, &trajectory);

                  if ( hand.position.x > x_leftBorder
                    && hand.position.x < x_rightBorder
                    && hand.position.y < y_bottomBorder )
                  {
                    storeInsert (store, Record (hand.position, hand_base, hand.position,
                                { muscle_i, muscle_j }, { 0, start_j }, { last_i, last_j }, 2U,
                                 trajectory)
                                 );
                    // trajectories.push_back (make_shared<trajectory_t> (trajectory));
                  }
                  else
                  { break; }
                } // end for
              } // end for
            } // end if
        } // end for
      } // end if
    } // end for
  }
  //------------------------------------------------------------------------------
  class ChooseDirection
  {
    // std::vector<Direction>  directions;




  };

  static void  getTargetCenter (Hand &hand, Point &center)
  {
    hand.SET_DEFAULT;
    Point start = hand.position;

    struct Direction
    {
      double             angle; // with Ox
      Point              direction;
      Hand::MusclesEnum  control;
      Point              norm_dir;

      Direction () :
        direction (0., 0.),
        control (Hand::EmptyMov),
        angle (0.)
      {}

      Direction (Point direction, Hand::MusclesEnum control) :
        direction (direction),
        control (control),
        angle (atan2 (direction.y, direction.x))
        {}

      double  cos_sim (const Direction &d) const
      {
        auto  ad =   direction;
        auto  bd = d.direction;
        /* cosine similarity */
        return    (ad.x * bd.x + ad.y * bd.y)
          / (sqrt (ad.x * ad.x + ad.y * ad.y)
          *  sqrt (bd.x * bd.x + bd.y * bd.y));
      }

      void draw (HDC hdc)
      {
        if ( !norm_dir.x && !norm_dir.y )
          norm_dir = Point (0.1 * cos (angle), 0.1 * sin (angle));
        
        MoveToEx (hdc, Tx(0.), Ty(0.), NULL);
        LineTo (hdc, Tx(norm_dir.x), Ty(norm_dir.y));
      }

    };

    std::vector<Direction>  directions;
    directions.reserve (hand.muscles_.size ());

    //hand.selectControl ()
    // for ( auto muscle_i : hand.muscles_ )
    for ( auto iCtrl = 0U; iCtrl < hand.controlsCount; ++iCtrl )
    {
      auto muscle_i = hand.selectControl (iCtrl);

      hand.move (muscle_i, 1U);
      // hand.step (muscle_i, 1U); hand.step ();

      directions.push_back (Direction (Point (hand.position.x - start.x,
                            hand.position.y - start.y), muscle_i));
    }
    // --------------------
    Direction   direct = Direction (Point (center.x - start.x,
                                    center.y - start.y), Hand::EmptyMov);

    Direction  &d = *std::min_element (directions.begin (), directions.end (),
                                       [&direct](const Direction &a, const Direction &b)
                                       { return  direct.cos_sim (a) < direct.cos_sim (b); });
                                       // { return  boost_distance (direct.direction, a.direction)
                                       //         > boost_distance (direct.direction, b.direction); });
    // --------------------

    auto max_last = hand.maxMuscleLast (d.control);
    hand.move (d.control, max_last);

    double center_distance = boost_distance (center, start);
    double maxdir_distance = boost_distance (hand.position, start);

    Hand::frames_t  last = static_cast<Hand::frames_t> ((center_distance * max_last) / maxdir_distance);
    hand.move (d.control, last);
  }

  static void  testCoverTarget (Store &store, Hand &hand, RecTarget &target)
  {
    hand.SET_DEFAULT;
    // std::list<std::list<Point>> trajectories;

    /* Fixate the aim point */
    boost_point2_t  aim = target.coords ()[45];
    double prev_distance = boost::geometry::distance (aim, hand.pos);

    double             best_distance = prev_distance;
    Hand::MusclesEnum  best_muscle = Hand::EmptyMov;
    Hand::time_t       best_last;

    // while ( aim.y () < hand.position.y )
    {
      /* Try each muscle */
      for ( auto i : boost::irange (1U, hand.controlsCount) )
      {
        auto muscle = hand.selectControl (i);
        auto last = hand.maxMuscleLast (muscle);

        hand.step (muscle);
        hand.step ();
        /* Find the best muscle by distance */
        double cur_distance = boost::geometry::distance (hand.pos, aim);
        if ( cur_distance < best_distance )
        {
          best_muscle = muscle;
          best_last = last;
          best_distance = cur_distance;
        }

        hand.reset ();
        hand.SET_DEFAULT;
      }

      hand.step (best_muscle);
      // while ( last-- )
      prev_distance = best_distance;
      double cur_distance = best_distance;
      auto last_ = 1U;

      do
        // for ( auto last_ = 1U; (prev_distance >= cur_distance) && (last_ <= best_last); ++last_ )
      {
        prev_distance = cur_distance;
        //-----------
        hand.step ();
        //-----------
        cur_distance = boost::geometry::distance (hand.pos, aim);
        ++last_;
      } while ( (prev_distance >= cur_distance) && (last_ <= best_last) );

      if ( !hand.moveEnd )
        hand.step (best_muscle);

      while ( !hand.moveEnd )
        hand.step ();
    }
    //=========================================================================
    // // const to target
    // const boost_point2_t  tlu (target.Min ().x, target.Max ().y),
    //                       tld (target.Min ().x, target.Min ().y),
    //                       trd (target.Max ().x, target.Min ().y),
    //                       tru (target.Max ().x, target.Max ().y);
    // 
    // const array<boost_point2_t, 4> target_corners = { tlu, tld, trd, tru };
    // const boost_point2_t *t_closest;
    // double distance = 0.;

    /* Create the tree of possible passes */
    //for ( Hand::MusclesEnum muscle_i : Hand::muscles )
    //{
    //  // auto h = hand.position;

    //  for ( uint_t last_i = 1U; last_i < hand.maxMuscleLast (muscle_i); ++last_i )
    //  {
    //    std::list<Point> trajectory;
    //    
    //    
    //    const boost_point2_t pos = hand.position;
    //    t_closest = &target_corners[0];
    //    distance = boost::geometry::distance (t_closest, pos);
    //    for ( size_t i = 1U; i < target_corners.size (); ++i )
    //    {
    //      double d = boost::geometry::distance (target_corners[i], pos);
    //      if ( distance > d )
    //      {
    //        distance = d;
    //        t_closest = &target_corners[i];
    //      }
    //    }

    //    // hand.move (muscle_i, last_i, trajectory);
    //    hand.step (muscle_i);
    //    // while ( last-- )
    //    for ( auto last_ = 1U; last_ <= last_i; ++last_ )
    //    {
    //      hand.step ();

    //      const boost_point2_t pos = hand.position;
    //      double d = boost::geometry::distance (pos, t_closest);
    //    }
    //    if ( !hand.isMoveEnd () )
    //      hand.step (muscle_i);

    //    while ( !hand.isMoveEnd () )
    //      hand.step ();

    //{
    //  const Point &aim = hand.position;
    //  Record rec (aim, aim,
    //              { muscle_i }, { 0 }, { last_i },
    //              1U, trajectory);
    //  storeInsert (store, rec);
    //}
    //=========================================================================
    //// std::cout << muscle_i << ' ' << hi << ' ' << last_i << std::endl;
    //insert (trajectories, trajectory);

    //if ( nesting > 1U )
    //  for ( Hand::MusclesEnum muscle_j : Hand::muscles )
    //  {
    //    if ( (muscle_i == muscle_j) || !muscleValidAtOnce (muscle_i | muscle_j) )
    //      // if ( j == i )
    //      continue;

    //    Point cur1 = hand.position;

    //    for ( uint_t last_j = 1U; last_j < hand.maxMuscleLast (muscle_j) / 2; ++last_j )
    //    {
    //      std::list<Point>::iterator tail_j = trajectory.end (); --tail_j;

    //      hand.move (muscle_j, last_j, trajectory);
    //      {
    //        const Point &aim = hand.position;
    //        storeInsert (store, Record (aim, aim,
    //        { muscle_i, muscle_j },
    //        { 0, last_i },
    //        { last_i, last_j },
    //                      2U,
    //                      trajectory)
    //                      );
    //      }

    //      // std::cout << muscles[j] << ' ' << hj << ' ' << last_j << std::endl;
    //      ++tail_j;

    //      // std::list::splice () allows you to concatenate two std::list's in constant time.
    //      insert (trajectories, trajectory);
    //      //=================================================
    //      if ( nesting > 2U )
    //        for ( Hand::MusclesEnum muscle_k : Hand::muscles )
    //        {
    //          if ( (muscle_i == muscle_k || muscle_k == muscle_j)
    //              || !muscleValidAtOnce (muscle_i | muscle_j | muscle_k) )
    //            // if ( k == i || k == j )
    //            continue;

    //          for ( auto last_k = 1U; last_k < hand.maxMuscleLast (muscle_k) / 2; ++last_k )
    //          {
    //            std::list<Point>::iterator tail_k = trajectory.end (); --tail_k;
    //            hand.move (muscle_k, last_k, trajectory);
    //            {
    //              const Point &aim = hand.position;
    //              storeInsert (store, Record (aim, aim,
    //              { muscle_i, muscle_j, muscle_k },
    //              { 0, last_i, last_i + last_j },
    //              { last_i, last_j, last_k },
    //                            3U,
    //                            trajectory)
    //                            );
    //            }

    //            // std::cout << muscles[k] << ' ' << hk << ' ' << last_k << std::endl;
    //            ++tail_k;
    //            insert (trajectories, trajectory);
    //            trajectory.erase (tail_k, trajectory.end ());

    //            hand.set (jointByMuscle (muscle_k), { (jointByMuscle (muscle_k) == Hand::Elbow) ? 70. : 0. });
    //          }
    //        }
    //      //=================================================
    //      trajectory.erase (tail_j, trajectory.end ());
    //      //a.insert (a.end (), b.begin (), b.end ());

    //      hand.set (jointByMuscle (muscle_j), { (jointByMuscle (muscle_j) == Hand::Elbow) ? 70. : 0. });
    //    }
    //  }
    //     hand.set (jointByMuscle (muscle_i), { (jointByMuscle (muscle_i) == Hand::Elbow) ? 70. : 0. });
    //   }
    // }
    //hand.SET_DEFAULT;


  }
  //------------------------------------------------------------------------------
  static HandMoves::Record  findExactPoint (Store &store, const Point &aim)
  {
    Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();
    auto result = index.find (boost::tuple<double, double> (aim.x, aim.y));
    return *result;
  }

  //------------------------------------------------------------------------------
  static void  UncoveredTargetPoints (IN  Store &store,
                               IN  const RecTarget &target,
                               OUT std::list<Point> &uncovered)
  {
    std::list<std::shared_ptr<Record>> range;
    adjacencyRectPoints (store, range, target.Min (), target.Max ());

    Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();
    for ( auto &pt : target.coords () )
    {
      if ( index.find (boost::tuple<double, double> (pt.x, pt.y)) == index.end () )
      { uncovered.push_back (pt); }
    }
  }

  static  bool  tryToHitTheAim (Store &store, Hand &hand, const Point &aim,
                        size_t maxTries, double epsilont = EPS)
  {
    bool result = 0;
    std::list<std::shared_ptr<HandMoves::Record>>  exact_range;
    /* While there is no exact trajectory */
    for ( size_t n_try = 0U; (result = exact_range.empty ()) || n_try < maxTries; ++n_try )
    {
      HandMoves::adjacencyPoints (store, exact_range, aim, epsilont);

    }
    return !result;
  }
  //------------------------------------------------------------------------------
};
#endif // _POSITION_H_
