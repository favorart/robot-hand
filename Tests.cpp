#include "StdAfx.h"
#include "Draw.h"
#include "Hand.h"
#include "target.h"
#include "MyWindow.h"
#include "HandMovesStore.h"


using namespace HandMoves;



bool  insert (std::list< std::list<Point> > &trajectories, std::list<Point> &trajectory)
{
  auto back = boost_point2_t(trajectory.back ());
  for ( auto t : trajectories )
  {
    auto fin = boost_point2_t (t.back ());
    if ( boost::geometry::distance (fin, back) < 0.001 )
      return false;
  }
  trajectories.push_back (trajectory);
  return true;
}
//------------------------------------------------------------------------------
void  HandMoves::test_random (Store &store, Hand &hand, uint_t tries)
{
  for ( uint_t j = 0; j < tries; ++j )
  {
    Hand::MusclesEnum  hyd[Record::maxMovesCount] = {};
    ulong_t           last[Record::maxMovesCount] = {};

    //uint_t count_moves = random_input<uint_t> (Record::maxMovesCount, 1U);
    //for ( uint_t i = 0U; i <= count_moves / 2U; ++i )
    //{
      //hyd[i] = HandMoves::switch_move (random_input<uint_t> (26U));
      //last[i] = random_input (hand.maxElbowMoveFrames, hand.minJStopMoveFrames);
      //hand.move (hyd[i], last[i]);
    //}

    // store.insert (hand.position, count_moves, hyd, last);
  }
}
void  HandMoves::test_cover (Store &store, Hand &hand, 
                             std::list< std::list<Point> > &trajectories,
                             int nesting)
{
  /* Create the tree of possible passes */
  // for ( auto i = 0U; i < Hand::musclesCount; ++i )
  for ( Hand::MusclesEnum  muscle_i : Hand::muscles )
  {
    hand.set (Hand::Clvcl | Hand::Shldr | Hand::Elbow, { 50., 50., 50. });
    // auto h = hand.position;

    for ( uint_t last_i = 1U; last_i < hand.timeMuscleWorking (muscle_i) / 2; ++last_i )
    {
      std::list<Point> trajectory;
      hand.move (muscle_i, last_i, trajectory);
      {
        const Point &aim = hand.position;
        uint_t starts[] = { 0 }, stops[] = { last_i };
        Hand::MusclesEnum ms[] = { muscle_i };
        Record rec (aim, aim, ms, starts, stops, 1U);
        store.insert (rec);
      }

      // std::cout << muscle_i << ' ' << hi << ' ' << last_i << std::endl;
      insert (trajectories, trajectory);

      if (nesting > 1)
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
              uint_t starts[] = { 0, last_i }, stops[] = { last_i, last_i + last_j };
              Hand::MusclesEnum ms[] = { muscle_i, muscle_j };
              store.insert (Record (aim, aim, ms, starts, stops, 2U));
            }

            // std::cout << muscles[j] << ' ' << hj << ' ' << last_j << std::endl;
            ++tail_j;
            insert (trajectories, trajectory);
            //=================================================
            if ( nesting > 2 )
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
                    uint_t starts[] = { 0, last_i, last_i + last_j }, 
                            stops[] = { last_i, last_i + last_j, last_i + last_j + last_k };
                    Hand::MusclesEnum ms[] = { muscle_i, muscle_j, muscle_k };
                    store.insert (Record (aim, aim, ms, starts, stops, 3U));
                  }
          
                  // std::cout << muscles[k] << ' ' << hk << ' ' << last_k << std::endl;
                  ++tail_k;
                  insert (trajectories, trajectory);
                  trajectory.erase (tail_k, trajectory.end ());

                  hand.set (jointByMuscle (muscle_k), { 50. });
                }
              }
            //=================================================
            trajectory.erase (tail_j, trajectory.end ());
            //a.insert (a.end (), b.begin (), b.end ());

            hand.set (jointByMuscle (muscle_j), { 50. });
          }
      }
      hand.set (jointByMuscle (muscle_i), { 50. });
    }
  }
  hand.set (Hand::Clvcl | Hand::Shldr | Hand::Elbow, { 50., 50., 50. });
}
//void  /*HandMoves::*/ test_cover2 (Store &store, Hand &hand, double radius,
//                                   std::list< std::list<Point> > &trajectories)
//{
//
//  typedef std::vector<Hand::MusclesEnum> vector_muscles_t;
//  vector_muscles_t  muscle_opn_cmds = { Hand::ClvclOpn, Hand::ShldrOpn, Hand::ElbowOpn };
//  vector_muscles_t  muscle_cls_cmds = { Hand::ClvclCls, Hand::ShldrCls, Hand::ElbowCls };
//
//  uchar_t js[] = { 0, 0, 0 };
//  hand.set (js);
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
//
//}
//void  HandMoves::test_cover1 (/* hm &Hm, */ Hand &hand, double radius)
//{
  /* Покрывающий тест */
  /*
     fixate (step_of_measure)

     for each muscle_of_hand
         total_time_of_muscle_opening = full_open (muscle_of_hand)
         saveDB (point_of_hand)

         tries = total_time_of_muscle_opening / step_of_measure
         for each time = step_of_measure * tries
             open (muscle_of_hand)
             saveDB (point_of_hand)

         for other muscle_of_hand


  */
  // using namespace boost;
  // int i = 0;
  // 
  // typedef std::vector< int > element_range_type;
  // typedef std::list< int > index_type;
  // 
  // 
  // static const int muscle_opn_cmd_size = 3U;
  // static const int index_size = 4U;
  // 
  // std::vector<Hand::MusclesEnum> muscle_opn_cmd = { Hand::ClvclOpn, Hand::ShldrOpn, Hand::ElbowOpn };
  // std::vector<Hand::MusclesEnum> muscle_cls_cmd = { Hand::ClvclCls, Hand::ShldrCls, Hand::ElbowCls };
  // 
  // do
  // {
  //   std::cout << myints[0] << ' ' << myints[1] << ' ' << myints[2] << '\n';
  // } while ( std::next_permutation (myints, myints + 3) );
  // 
  // std::cout << "After loop: " << myints[0] << ' ' << myints[1] << ' ' << myints[2] << '\n';


//  return;
//}
//------------------------------------------------------------------------------
