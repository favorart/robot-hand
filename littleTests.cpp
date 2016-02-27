#include "StdAfx.h"
#include "Hand.h"
#include "HandMovesStore.h"

#include "littleTests.h"
#include "WindowData.h"


using namespace std;
using namespace HandMoves;
//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
void  littleTest (MyWindowData &wd, double radius)
{
  for ( auto pt : wd.target.coords () | boost::adaptors::sliced (wd.no, wd.target.coordsCount ()) )
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
*   + Arounded ???
*   + ?? NO POINTS ???
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
//------------------------------------------------------------------------------
