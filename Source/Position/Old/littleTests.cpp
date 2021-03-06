#include "StdAfx.h"

#include "littleTests.h"
#include "WindowData.h"


using namespace std;
using namespace HandMoves;
//------------------------------------------------------------------------------
void  LittleTest::draw (HDC hdc, MyWindowData &wd, HPEN hPen /*wd.hPen_orng*/) const
{
  DrawCircle (hdc, aim, 0.007);
  //---------------------------------
  for ( auto &traj : traj_s1 )
    DrawTrajectory (hdc, *traj, hPen);

  DrawAdjacency (hdc, aim, radius, ellipse, wd.hPen_red);

  HPEN Pen_old = (HPEN) SelectObject (hdc, wd.hPen_red);
  for ( auto &pt : pt_s )
  { DrawCircle (hdc, *pt, 0.007); }
  SelectObject (hdc, Pen_old);
  //---------------------------------
}
//------------------------------------------------------------------------------
size_t  littleTest (MyWindowData &wd, double radius)
{
  for ( auto &pt : wd.target.coords () | boost::adaptors::sliced (wd.no, wd.target.coordsCount ()) )
  {
    ++wd.no;
    HandMoves::adjacency_refs_t  exact_range;
    wd.store.adjacencyPoints (exact_range, pt, EPS);
    /* If there is no exact trajectory */

    // if ( exact_range.empty () )
    // {
    //   /* Try to find the several closest trajectories */
    //   HandMoves::adjacency_refs_t  range;
    //   HandMoves::adjacencyPoints (wd.store, range, pt, radius);
    // 
    //   if ( wd.lt ) delete wd.lt;
    //   wd.lt = new LittleTest (pt, radius);
    //   wd.lt->appendPts (range, true);
    // 
    //   return exact_range.size ();
    // }
  }
  return  0U;
}
size_t  littleTest (MyWindowData &wd) //, double radius)
{
  for ( auto &pt : wd.target.coords () | boost::adaptors::sliced (wd.no, wd.target.coordsCount ()) )
  {
    ++wd.no;
    HandMoves::adjacency_refs_t  exact_range;
    Point aim = pt;

    wd.store.adjacencyPoints (exact_range, pt, 3 * EPS_VIS);
    if ( !exact_range.empty () )
    {
      // exact_range.sort (ElegancePredicate ());
      HandMoves::controling_t  controls{ exact_range.front ()->controls };

      if ( wd.lt ) delete wd.lt;
      wd.lt = new LittleTest (aim, 3 * EPS_VIS);
      // wd.lt->appendPts (range, true);
      wd.lt->appendTraj (exact_range.front ()->trajectory );

      wd.hand.SET_DEFAULT;
      Point hand_base = wd.hand.position;

      wd.hand.move (controls.begin (), controls.end ());
      double init_distance = boost_distance (wd.hand.position, aim);
      
      wd.hand.SET_DEFAULT;
      for ( auto  &c : controls )
      {
        auto last_backup = c.last;
        for ( auto i : boost::irange (-5, 6) )
        {
          trajectory_t visited;
          c.last += i;

          wd.hand.move (controls.begin (), controls.end (), &visited);
          // if ( init_distance > boost_distance (wd.hand.position, aim) )
          {
            wd.store.insert (Record (aim, hand_base, wd.hand.position,
                                     controls, visited) );
          }
          c.last = last_backup;
          wd.hand.SET_DEFAULT;
        }
        
        auto start_backup = c.start;
        for ( auto i : boost::irange (-5, 6) )
        {
          trajectory_t visited;
          c.start += i;
          
          wd.hand.move (controls.begin (), controls.end (), &visited);
          // if ( init_distance > boost_distance (wd.hand.position, aim) )
          {
            wd.store.insert (Record (aim, hand_base, wd.hand.position,
                                     controls, visited));
          }
          c.start = start_backup;
          wd.hand.SET_DEFAULT;
        }
      } // end for
      return  exact_range.size ();
    /*
     *   
     *   
     */
   
    /* If there is no exact trajectory */
    
    //   /* Try to find the several closest trajectories */
    //   std::list<std::shared_ptr<HandMoves::Record>> range;
    // 
    //   HandMoves::adjacencyPoints (wd.store, range, pt, radius);
    // 
    //   if ( wd.lt ) delete wd.lt;
    //   wd.lt = new LittleTest (pt, radius);
    //   wd.lt->appendPts (range, true);
    // 
    //   return exact_range.size ();
    }
  }
  return 0U;
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
    store.adjacencyPoints (exact, ptTarget, epsilont);
    /* If there is no exact trajectory */
    if ( exact.empty () )
    {
      /* Try to find the several closest trajectories */
      std::list<std::shared_ptr<HandMoves::Record>> range;
      store.adjacencyPoints (range, ptTarget, radius);

      auto lt = new LittleTest (ptTarget, radius);
      lt->appendPts (range, true);

      /* Construct the linear combinations */
      if ( range.size () >= 3U )
      {
        HandMoves::ClosestPredicate  CPred (ptTarget);
        auto it_min = std::min_element (range.begin (), range.end (), CPred);

        /*  */
        for ( auto &rec : range )
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
