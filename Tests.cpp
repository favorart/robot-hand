#include "StdAfx.h"
#include "Draw.h"
#include "Hand.h"
#include "target.h"
#include "MyWindow.h"
#include "HandMovesStore.h"


using namespace HandMoves;

Hand::MusclesEnum  HandMoves::switch_move (uint_t choose)
{
  Hand::MusclesEnum  hyd;
  switch ( choose )
  {
    default: hyd = Hand::EmptyMov;                                   break;
    case  0: hyd = Hand::ClvclOpn;                                   break;
    case  1: hyd = Hand::ShldrOpn;                                   break;
    case  2: hyd = Hand::ElbowOpn;                                   break;
    case  3: hyd = Hand::ClvclCls;                                   break;
    case  4: hyd = Hand::ShldrCls;                                   break;
    case  5: hyd = Hand::ElbowCls;                                   break;
    case  6: hyd = Hand::ClvclOpn | Hand::ShldrOpn;                  break;
    case  7: hyd = Hand::ClvclOpn | Hand::ElbowOpn;                  break;
    case  8: hyd = Hand::ShldrOpn | Hand::ElbowOpn;                  break;
    case  9: hyd = Hand::ClvclOpn | Hand::ShldrOpn | Hand::ElbowOpn; break;
    case 10: hyd = Hand::ClvclOpn | Hand::ShldrOpn | Hand::ElbowCls; break;
    case 11: hyd = Hand::ClvclCls | Hand::ShldrOpn;                  break;
    case 12: hyd = Hand::ClvclCls | Hand::ElbowOpn;                  break;
    case 13: hyd = Hand::ShldrCls | Hand::ElbowOpn;                  break;
    case 14: hyd = Hand::ClvclCls | Hand::ShldrOpn | Hand::ElbowOpn; break;
    case 15: hyd = Hand::ClvclCls | Hand::ShldrOpn | Hand::ElbowCls; break;
    case 16: hyd = Hand::ClvclOpn | Hand::ShldrCls;                  break;
    case 17: hyd = Hand::ClvclOpn | Hand::ElbowCls;                  break;
    case 18: hyd = Hand::ShldrOpn | Hand::ElbowCls;                  break;
    case 19: hyd = Hand::ClvclOpn | Hand::ShldrCls | Hand::ElbowOpn; break;
    case 20: hyd = Hand::ClvclOpn | Hand::ShldrCls | Hand::ElbowCls; break;
    case 21: hyd = Hand::ClvclCls | Hand::ShldrCls;                  break;
    case 22: hyd = Hand::ClvclCls | Hand::ElbowCls;                  break;
    case 23: hyd = Hand::ShldrCls | Hand::ElbowCls;                  break;
    case 24: hyd = Hand::ClvclCls | Hand::ShldrCls | Hand::ElbowOpn; break;
    case 25: hyd = Hand::ClvclCls | Hand::ShldrCls | Hand::ElbowCls; break;
  }
  return hyd;
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
void  HandMoves::test_cover (Store &store, Hand &hand, double radius,
                             std::list< std::list<Point> > &trajectories,
                             int number)
{
  /* Create the tree of possible passes */
  // for ( auto i = 0U; i < Hand::musclesCount; ++i )
  for ( Hand::MusclesEnum  muscle_i : Hand::muscles )
  {
    std::list<Point> trajectory;

    uchar_t js[] = { 50, 50, 50 };
    hand.set (js);

    for ( uint_t last_i = 1U; last_i < hand.timeMuscleWorking (muscle_i); ++last_i )
    {
      hand.move (muscle_i, last_i, trajectory);
      {
        const Point &aim = hand.position;
        uint_t starts[] = { 0 }, stops[] = { last_i };
        Record rec (aim, aim, muscle_i, starts, stops);
        store.insert (rec);
      }
      // std::cout << muscle_i << ' ' << hi << ' ' << last_i << std::endl;

      //for ( auto j = 0U; j < muscles.size (); ++j )
      hand.set (muscle_i, 0);
      for ( Hand::MusclesEnum  muscle_j : Hand::muscles )
      {
        if ( (muscle_i == muscle_j) || !muscleValidAtOnce (muscle_i | muscle_j) )
          // if ( j == i )
          continue;

        hand.set (muscle_j, 0);
        for ( uint_t last_j = 1U; last_j < hand.timeMuscleWorking (muscle_j); ++last_j )
        {
          hand.move (muscle_i, last_j, trajectory);
          {
            const Point &aim = hand.position;
            uint_t starts[] = { 0, last_i }, stops[] = { last_i, last_j };
            store.insert ( Record (aim, aim, muscle_j, starts, stops) );
          }
          //  std::cout << muscles[j] << ' ' << hj << ' ' << last_j << std::endl;
      //    
      //
      //    for ( auto k = 0U; k < muscles.size (); ++k )
      //    {
      //      if ( k == i || k == j ) continue;
      //      for ( auto last_k = 1U; last_k < muscles.size (); ++last_k )
      //      {
      //        hand.move (muscles[k], last_k);
      //        Point hk = hand.position;
      //
      //        std::cout << muscles[k] << ' ' << hk << ' ' << last_k << std::endl;
      //        store.insert (hk);
      //      }
      //      hand.reset ();
      //      hand.move (muscles[i], last_i);
      //      hand.move (muscles[j], last_j);
      //    }
        }
        // hand.reset ();
        // hand.move (muscles[i], last_i);
      }
    }
    // hand.reset ();
    trajectories.push_back (trajectory);
  }
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
