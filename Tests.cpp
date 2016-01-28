#include "StdAfx.h"
#include "Hand.h"
#include "HandMovesStore.h"

using namespace std;
using namespace HandMoves;
//------------------------------------------------------------------------------
bool  insert (std::list< std::list<Point> > &trajectories, std::list<Point> &trajectory)
{
  auto back = boost_point2_t(trajectory.back ());
  for ( auto t : trajectories )
  {
    auto fin = boost_point2_t (t.back ());
    if ( boost::geometry::distance (fin, back) < 0.001 )
      return false;
  }
  // trajectories.splice (it, trajectory);
  trajectories.push_back (trajectory);
  return true;
}
//------------------------------------------------------------------------------
void  HandMoves::test_random (Store &store, Hand &hand, size_t tries)
{
  /* Для нового потока нужно снова переинициализировать rand */
  std::srand ((unsigned int) clock ());

  for ( uint_t i = 0U; i < tries; ++i )
  {
    trajectory_t visited;
    Record::muscles_array  muscles = {};
    Record::times_array    lasts = {};
    Record::times_array    times = {};
    // Record::times_array times_stop  = {};
    size_t moves_count = random (1U, Record::maxMovesCount);

    hand.SET_DEFAULT;
    for ( uint_t j = 0U; j < moves_count; ++j )
    {
      auto muscle = selectHandMove ( random (1U, HandMovesCount - 1U) );
      auto last   = random ( hand.timeMuscleWorking (muscle) );
      hand.move (muscle, last, visited);

      muscles[j] = muscle;
      times[j] = (j) ? lasts[j - 1] : 0;
      lasts[j] = last;

      // times_stop[j]  = (j) ? (times_stop[j - 1] + last) : last;
    }

    const Point &aim = hand.position;

    try
    { auto rec = Record (aim, aim, muscles,
                         times, lasts,
                         moves_count, visited);
      store.insert (rec);
    }
    catch(...)
    { continue; }

  }
}
void  HandMoves::test_cover  (Store &store, Hand &hand, 
                              // std::list< std::list<Point> > &trajectories,
                              size_t nesting)
{
  std::list<std::list<Point>> trajectories;
  /* Create the tree of possible passes */
  // for ( auto i = 0U; i < Hand::musclesCount; ++i )
  for ( Hand::MusclesEnum  muscle_i : Hand::muscles )
  {
    hand.SET_DEFAULT;
    // auto h = hand.position;

    for ( uint_t last_i = 1U; last_i < hand.timeMuscleWorking (muscle_i) / 2; ++last_i )
    {
      std::list<Point> trajectory;
      hand.move (muscle_i, last_i, trajectory);
      {
        const Point &aim = hand.position;
        Record rec (aim, aim,
                    { muscle_i }, { 0 }, { last_i },
                    1U, trajectory);
        store.insert (rec);
      }

      // std::cout << muscle_i << ' ' << hi << ' ' << last_i << std::endl;
      insert (trajectories, trajectory);

      if ( nesting > 1U )
        for ( Hand::MusclesEnum  muscle_j : Hand::muscles )
        {
          if ( (muscle_i == muscle_j) || !muscleValidAtOnce (muscle_i | muscle_j) )
            // if ( j == i )
            continue;

          Point cur1 = hand.position;

          for ( uint_t last_j = 1U; last_j < hand.timeMuscleWorking (muscle_j) / 2; ++last_j )
          {
            std::list<Point>::iterator tail_j = trajectory.end (); --tail_j;

            hand.move (muscle_j, last_j, trajectory);
            {
              const Point &aim = hand.position;
              store.insert (Record (aim, aim, 
                                    { muscle_i, muscle_j }, 
                                    { 0, last_i },
                                    { last_i, last_j },
                                    2U,
                                    trajectory)
                            );
            }

            // std::cout << muscles[j] << ' ' << hj << ' ' << last_j << std::endl;
            ++tail_j;

            // std::list::splice () allows you to concatenate two std::list's in constant time.
            insert (trajectories, trajectory);
            //=================================================
            if ( nesting > 2U )
              for ( Hand::MusclesEnum muscle_k : Hand::muscles )
              {
                if ( (muscle_i == muscle_k || muscle_k == muscle_j) 
                  || !muscleValidAtOnce (muscle_i | muscle_j | muscle_k) )
                  // if ( k == i || k == j )
                  continue;

                for ( auto last_k = 1U; last_k < hand.timeMuscleWorking (muscle_k) / 2; ++last_k )
                {
                  std::list<Point>::iterator tail_k = trajectory.end (); --tail_k;
                  hand.move (muscle_k, last_k, trajectory);
                  {
                    const Point &aim = hand.position;
                    store.insert (Record (aim, aim, 
                                         { muscle_i, muscle_j, muscle_k },
                                         { 0, last_i, last_i + last_j },
                                         { last_i, last_j, last_k },
                                         3U,
                                         trajectory)
                                  );
                  }
          
                  // std::cout << muscles[k] << ' ' << hk << ' ' << last_k << std::endl;
                  ++tail_k;
                  insert (trajectories, trajectory);
                  trajectory.erase (tail_k, trajectory.end ());

                  hand.set (jointByMuscle (muscle_k), { (jointByMuscle (muscle_k) == Hand::Elbow) ? 70. : 0. });
                }
              }
            //=================================================
            trajectory.erase (tail_j, trajectory.end ());
            //a.insert (a.end (), b.begin (), b.end ());

            hand.set (jointByMuscle (muscle_j), { (jointByMuscle (muscle_j) == Hand::Elbow) ? 70. : 0. });
          }
      }
      hand.set (jointByMuscle (muscle_i), { (jointByMuscle (muscle_i) == Hand::Elbow) ? 70. : 0. });
    }
  }
  hand.SET_DEFAULT;
}

//------------------------------------------------------------------------------
void  /*HandMoves::*/ test_cover2 (Store &store, Hand &hand, double radius,
                                   std::list< std::list<Point> > &trajectories)
{
//
//  typedef std::vector<Hand::MusclesEnum> vector_muscles_t;
//  vector_muscles_t  muscle_opn_cmds = { Hand::ClvclOpn, Hand::ShldrOpn, Hand::ElbowOpn };
//  vector_muscles_t  muscle_cls_cmds = { Hand::ClvclCls, Hand::ShldrCls, Hand::ElbowCls };
//
//  hand.SET_DEFAULT;
//
//  auto muscles = muscle_opn_cmds;
//  for ( auto i = 0U; i < muscles.size (); ++i )
//  {
//    Point h = hand.position;
//    store.insert (h);
//
//    for ( auto last_i = 1U; last_i < hand.timeMuscleWorking (muscles[i]); ++last_i )
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
void  /*HandMoves::*/ testCoverTarget (Store &store, Hand &hand, RecTarget &target)
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
    for ( auto i : boost::irange (1U, 25U) )
    {
      auto muscle = selectHandMove (i);
      auto last = hand.timeMuscleWorking (muscle);

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

  //  for ( uint_t last_i = 1U; last_i < hand.timeMuscleWorking (muscle_i); ++last_i )
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

      //    for ( uint_t last_j = 1U; last_j < hand.timeMuscleWorking (muscle_j) / 2; ++last_j )
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

      //          for ( auto last_k = 1U; last_k < hand.timeMuscleWorking (muscle_k) / 2; ++last_k )
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
