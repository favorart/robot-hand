#include "StdAfx.h"
#include "Hand.h"
#include "HandMovesStore.h"

using namespace std;
using namespace HandMoves;
//------------------------------------------------------------------------------
bool  /*trajectories_concat*/ insert (std::list<std::list<Point>>  &trajectories,
                                                std::list<Point>   &trajectory)
{
  auto back = boost_point2_t (trajectory.back ());
  for ( auto t : trajectories )
  {
    auto fin = boost_point2_t (t.back ()); 
    if ( boost::geometry::distance (fin, back) < EPS )
      return false;
  }
  // trajectories.splice (it, trajectory);
  trajectories.push_back (trajectory);
  return true;
}
//------------------------------------------------------------------------------
// !!! ALMOST DONE
void  HandMoves::test_random (Store &store, Hand &hand, size_t tries) 
{
  /* Для нового потока нужно снова переинициализировать rand */
  std::srand ((unsigned int) clock ());

  for ( uint_t i = 0U; i < tries; ++i )
  {
    trajectory_t visited;
    Record::muscles_array  muscles = {};
    Record::times_array    start_times = {};
    Record::times_array    lasts = {};
    
    size_t  moves_count = random (1U, Record::maxControlsCount);

    hand.SET_DEFAULT;
    Point hand_base = hand.position;
    for ( size_t j = 0U; j < moves_count; ++j )
    {
      // auto muscle = selectHandMove ( random (1U, HandMovesCount - 1U) );
      auto muscle = hand.selectControl ();
      auto last   = random ( hand.maxMuscleLast (muscle) );
      hand.move (muscle, last, visited);

      /*
         start_times[0] = 0U
         start_times[1] = random (1U, lasts[0])

         lasts[0] = random (1U, maxFrames_activeMuscles_correctedByCurrentPosition)
         lasts[1] = random (1U, maxFrames_activeMuscles_correctedByCurrentPosition)
      */

      muscles[j] = muscle;
      start_times[j] = (j) ? lasts[j - 1] : 0;
      lasts[j] = last;

      // times_stop[j]  = (j) ? (times_stop[j - 1] + last) : last;
    }

    const Point &aim = hand.position;

    try
    { auto rec = Record (aim, hand_base, aim,
                         muscles,
                         start_times, lasts,
                         moves_count,
                         visited);
      store.insert (rec);
    }
    catch(...)
    { continue; }

  }
}

// ??? PROGRESS
void  HandMoves::test_cover  (Store &store, Hand &hand, size_t nesting)
{
  /* Create the tree of possible passes */
  for ( Hand::MusclesEnum  muscle_i : hand.muscles_ )
  {
    hand.SET_DEFAULT;
    Point hand_base = hand.position;
    for ( Hand::frames_t last_i : boost::irange(1U, hand.maxMuscleLast (muscle_i)) )
    {
      std::list<Point> trajectory;

      hand.move (muscle_i, last_i, trajectory);
      
      store.insert (Record (hand.position, hand_base, hand.position,
                            { muscle_i }, { 0 }, { last_i },
                            1U, trajectory));

      if ( nesting > 1U )
        for ( Hand::MusclesEnum  muscle_j : hand.muscles_ )
        {
          if ( (muscle_i == muscle_j) || !muscleValidAtOnce (muscle_i | muscle_j) )
            continue;

          for ( uint_t last_j = 1U; last_j < hand.maxMuscleLast (muscle_j); ++last_j )
          {
            std::list<Point>::iterator tail_j = trajectory.end ();
            --tail_j;

            hand.move (muscle_j, last_j, trajectory);
            
            store.insert (Record (hand.position, hand_base, hand.position,
                                  { muscle_i, muscle_j }, 
                                  { 0, last_i },
                                  { last_i, last_j },
                                  2U, trajectory));
            ++tail_j;
            //=================================================
            if ( nesting > 2U )
              for ( Hand::MusclesEnum muscle_k : hand.muscles_ )
              {
                if ( (muscle_i == muscle_k || muscle_k == muscle_j) 
                  || !muscleValidAtOnce (muscle_i | muscle_j | muscle_k) )
                  continue;

                for ( auto last_k = 1U; last_k < hand.maxMuscleLast (muscle_k); ++last_k )
                {
                  std::list<Point>::iterator tail_k = trajectory.end ();
                  --tail_k;
                  
                  hand.move (muscle_k, last_k, trajectory);
                  
                  store.insert (Record (hand.position, hand_base, hand.position,
                                         { muscle_i, muscle_j, muscle_k },
                                         { 0, last_i, last_i + last_j },
                                         { last_i, last_j, last_k },
                                         3U, trajectory));
                  ++tail_k;
                  
                  trajectory.erase (tail_k, trajectory.end ());
                  hand.set (jointByMuscle (muscle_k), { (jointByMuscle (muscle_k) == Hand::Elbow) ? 70. : 0. });
                }
              }
            //=================================================
            trajectory.erase (tail_j, trajectory.end ());
            hand.set (jointByMuscle (muscle_j), { (jointByMuscle (muscle_j) == Hand::Elbow) ? 70. : 0. });
          }
      }
      hand.set (jointByMuscle (muscle_i), { (jointByMuscle (muscle_i) == Hand::Elbow) ? 70. : 0. });
    }
  }
  hand.SET_DEFAULT;
}


namespace HandMoves
{
  void  testCover       (Store &store, Hand &hand, Hand::MusclesEnum muscles);
  //void  testCoverTarget (Store &store, Hand &hand, RecTarget &target);
};
//------------------------------------------------------------------------------
/*
   Количество точек, в окресности искомой точки.


   Что я могу варьировать?

   v  1. Длительность работы каждого мускула
   v  2. Время старта каждого мускула
   ?  3. Тормозить мускулом? (снимать от движущегося начинающий двигаться)
   ?  4. Может быть !1 перелом! траектории (полная остановка)
         Он может нам понадобиться в зависимости от законов движения
         разных мускулов, если один до второго не успевает ...
         (?позже включить??)
*/

// ??? PROGRESS
void  HandMoves::testCover (Store &store, Hand &hand, Hand::MusclesEnum muscles)
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

  /* возьмём первый мускул наугад */
  for ( auto muscle_i : hand.muscles_ )
  {
    if ( !(muscle_i & muscles) )
    {
      std::list<Point> trajectory;

      /* Попробуем его варьировать по длительности */
      for ( Hand::frames_t last_i : boost::irange (1U, hand.maxMuscleLast (muscle_i)) )
      {
        hand.move (muscle_i, last_i, trajectory);
        storeInsert (store,
                     Record (hand.position, hand_base, hand.position,
                     { muscle_i }, { 0 }, { last_i },
                     1U, trajectory));

        /*  Теперь в каждый такт длительности первого мускула, включим второй
         *  и будет варьировать его длительность
         */
        testCover (store, hand, muscles & muscle_i);

      } // end for
    } // end if
  } // end for

// for ( auto last_i = 1U; last_i < hand.maxMuscleLast (muscles[i]); ++last_i )
//    {
//      hand.move (muscles[i], 1); // last_i);
//      Point hi = hand.position;
//      std::cout << muscles[i] << ' ' << hi << ' ' << last_i << std::endl;
//      store.insert (hi);
//
//      for ( auto j = 0U; j < muscles.size (); ++j )
//      {
//        if ( j == i ) continue;
//        for ( auto last_j = 1U; last_j < muscles.size (); ++last_j )
//        {
//          hand.move (muscles[j], 1); // last_j);
//          Point hj = hand.position;
//
//          std::cout << muscles[j] << ' ' << hj << ' ' << last_j << std::endl;
//          store.insert (hj);
//
//          for ( auto k = 0U; k < muscles.size (); ++k )
//          {
//            if ( k == i || k == j ) continue;
//            for ( auto last_k = 1U; last_k < muscles.size (); ++last_k )
//            {
//              hand.move (muscles[k], last_k);
//              Point hk = hand.position;
//
//              std::cout << muscles[k] << ' ' << hk << ' ' << last_k << std::endl;
//              store.insert (hk);
//            }
//            hand.reset ();
//            hand.move (muscles[i], last_i);
//            hand.move (muscles[j], last_j);
//          }
//        }
//        hand.reset ();
//        hand.move (muscles[i], last_i);
//      }
//    }
//    hand.reset ();
//  }
}
//------------------------------------------------------------------------------
#include "target.h"

// ??? PROGRESS
void  getTargetCenter (Hand &hand)
{


}
// ??? PROGRESS
void  /*HandMoves::*/testCoverTarget (Store &store, Hand &hand, RecTarget &target)
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
      //  store.insert (rec);
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
      //        store.insert (Record (aim, aim,
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
      //              store.insert (Record (aim, aim,
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
HandMoves::Record  /* HandMoves:: */ findExactPoint (Store &store, const Point &aim)
{
  Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();
  auto result = index.find (boost::tuple<double, double> (aim.x, aim.y));
  return *result;
}

void  UncoveredTargetPoints (IN  Store &store,
                             IN  const RecTarget &target,
                             OUT std::list<Point> &uncovered)
{
  std::list<std::shared_ptr<Record>> range;
  adjacencyRectPoints (store, range, target.Min (), target.Max ());

  Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();
  for ( auto pt : target.coords () )
  {
    if ( index.find (boost::tuple<double, double> (pt.x, pt.y)) == index.end () )
    { uncovered.push_back (pt); }
  }
}

bool  tryToHitTheAim (Store &store, Hand &hand, const Point &aim,
                      size_t maxTries, double epsilont=EPS)
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

#include "littleTests.h"
#include "WindowData.h"

void  LittleTest::draw (HDC hdc, MyWindowData &wd) const
{
  DrawCircle (hdc, aim, 0.007);
  //---------------------------------
  for ( auto traj : traj_s1 )
    DrawTrajectory (hdc, *traj, NULL /*wd.hPen_orng*/);

  DrawAdjacency (hdc, aim, radius, ellipse, wd.hPen_red);

  HPEN Pen_old = (HPEN) SelectObject (hdc, wd.hPen_red);
  for ( auto pt : pt_s )
  { DrawCircle (hdc, *pt, 0.007); }
  SelectObject (hdc, Pen_old);
  //---------------------------------
}

void littleTest (MyWindowData &wd, double radius)
{
  for ( auto pt : wd.target.coords () | boost::adaptors::sliced (wd.no, wd.target.coordsCount ()))
  {
    ++wd.no;
    std::list<std::shared_ptr<HandMoves::Record>> exact_range;

    HandMoves::adjacencyPoints (wd.store, exact_range, pt, EPS);
    /* If there is no exact trajectory */
    if ( exact_range.empty () )
    {
      /* Try to find the several closest trajectories */
      std::list<std::shared_ptr<HandMoves::Record>> range;

      HandMoves::adjacencyPoints (wd.store, range, pt, radius);

      if ( wd.lt ) delete wd.lt;
      wd.lt = new LittleTest (pt, radius);
      wd.lt->appendPts (range, true);
      return;
    }
  }
}

/* 
 *  Strategies:
 *   + findBest
 *   + 
 */
void  /*HandMoves::*/ testLittleCorrectives (Store &store, Hand &hand, RecTarget &target,
                                             double radius, /* minimal distance between 2 neighbour points of target */
                                             double epsilont)
{
  hand.SET_DEFAULT;

  for ( const Point &ptTarget : target.coords () )
  {
    /* For each point in Target */
    std::list<std::shared_ptr<HandMoves::Record>> exact;
    HandMoves::adjacencyPoints (store, exact, ptTarget, epsilont);
    /* If there is no exact trajectory */
    if ( exact.empty () )
    {
      /* Try to find the several closest trajectories */
      std::list<std::shared_ptr<HandMoves::Record>> range;
      HandMoves::adjacencyPoints (store, range, ptTarget, radius);

      auto lt = new LittleTest (ptTarget, radius);
      lt->appendPts (range, true);
          
      /* Construct the linear combinations */
      if ( range.size () >= 3U )
      {
        HandMoves::ClosestPredicate  pred (ptTarget);
        auto it_min = std::min_element (range.begin (), range.end (), pred);
      
        /*  */
        for ( auto rec : range )
        {
        
        }
      }
      // else if ( range.size () == 2U )
      // {
      // 
      // 
      // }
      else if ( range.size () == 1U )
      {
        auto it_best = range.front ();

        // best_muscle = it_best->muscle;
        // best_last
        //   hand_moves
      }
      else /* Nothing in range */
      {
        if ( radius > 0 )
          testLittleCorrectives (store, hand, target, 2 * radius, epsilont);
        else
          return;
      }
      
      /* hand moving */
      // hand.SET_DEFAULT;
      // while ( boost_distance (ptTarget, hand.pos) >= epsilont )
      // {
      //   for ( auto i : boost::irange (1U, hand_moves) )
      //     hand.move (best_muscle[i], best_last[i]);
      // }
    }

    /* */
    // if (  )
  }
}
