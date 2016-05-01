#include "StdAfx.h"
#include "Position.h"

namespace Positions
{
  using namespace std;
  using namespace HandMoves;
  //------------------------------------------------------------------------------
  void  LearnMovements::weightedMeanControls (IN  const Point &aim,
                                              IN  const HandMoves::adjacency_ptrs_t &range,
                                              OUT HandMoves::controling_t &controls,
                                              OUT double *weight)
  {
    // -----------------------------------------------
    double all_distance = 0.;
    for ( auto &pRec : range )
    { all_distance += boost_distance (aim, pRec->hit); }

    if ( weight ) { *weight = (all_distance / range.size ()); }
    // -----------------------------------------------
    for ( auto &pRec : range )
      for ( auto &c : pRec->controls )
      {
        auto it = std::find (controls.begin (), controls.end (), c.muscle);
        // for ( auto m : hand.muscles_ )

        /* взвешенное НЕСМЕЩЁННОЕ cреднее арифметическое */
        double weight = boost_distance (aim, pRec->hit) / all_distance;
        if ( it == controls.end () )
        {
          controls.push_back (Hand::Control (c.muscle,
                                             0U, //c.start * weight,
                                             c.last * weight));
        }
        else
        {
          // it->start += c.start * weight;
          it->last += c.last * weight;
        } // end if
      } // end for-for

        // ----------------------------------------------
        /* controls correct-check */
    for ( auto j : hand.joints_ )
    {
      auto mo = muscleByJoint (j, true);
      auto mc = muscleByJoint (j, false);

      auto it_o = std::find (controls.begin (), controls.end (), mo);
      auto it_c = std::find (controls.begin (), controls.end (), mc);

      if ( it_o != controls.end () && it_c != controls.end () )
      {
        if ( it_o->last > it_c->last )
        {
          it_c->start = it_o->last + 1U; //  it_o->start;
          it_o->start = 0U;
        }
        else
        {
          it_o->start = it_c->last + 1U; // it_c->start;
          it_c->start = 0U;
        }
      }
    }
    // ----------------------------------------------
    auto it = controls.begin ();
    while ( it != controls.end () )
    {
      if ( !it->last )
      { it = controls.erase (it); }
      else
      { ++it; }
    }
    // ----------------------------------------------
  }
  //------------------------------------------------------------------------------
  size_t  LearnMovements::weightedMean (IN  const Point &aim, OUT Point &hand_position, IN bool verbose)
  {
    size_t w_means_complexity = 0U;
    double side_ = side;
    // -----------------------------------------------
    const HandMoves::Record  &rec = store.ClothestPoint (aim, side);
    // -----------------------------------------------
    // HandMoves::controling_t  controls{ rec.controls };
    Point  hand_pos = rec.hit;
    // -----------------------------------------------
    double  distance = boost_distance (aim, hand_pos),
       next_distance = distance;
    // -----------------------------------------------
    hand.SET_DEFAULT;
    do
    {
      if ( next_distance < distance )
      { 
        distance = next_distance;
        hand_position = hand_pos;

        if ( precision > distance )
        { break; }
      }
      // -----------------------------------------------
      HandMoves::adjacency_ptrs_t  range;
      store.adjacencyByPBorders (range, aim, side_);
      if ( range.empty () )
      { break; }
      // -----------------------------------------------
      side_ -= side_decrease_step;
      // -----------------------------------------------
      HandMoves::controling_t  controls;
      weightedMeanControls (aim, range, controls);
      // -----------------------------------------------
      if ( handAct (aim, controls, hand_pos) )
      { ++w_means_complexity; }
      // -----------------------------------------------
      next_distance = boost_distance (hand_pos, aim);
      // -----------------------------------------------
    } while ( next_distance < distance );
    // -----------------------------------------------
    if ( verbose )
    {
      tcout << _T ("prec: ") << distance << std::endl;
      tcout << _T ("w_means complexity: ")
            <<      w_means_complexity
            << std::endl << std::endl;
    }
    // -----------------------------------------------
    return  w_means_complexity;
  }
  //------------------------------------------------------------------------------
  bool  LearnMovements::weightedMeanULAdjs (IN  const Point  &aim, OUT HandMoves::Record *pRec,
                                            OUT HandMoves::controling_t &lower_controls,
                                            OUT HandMoves::controling_t &upper_controls,
                                            OUT double &lower_distance,
                                            OUT double &upper_distance)
  {
    if ( !pRec ) { return  false; }

    Point  min, max;
    // ------------------------------------------------
    lower_controls.clear ();
    upper_controls.clear ();
    // ------------------------------------------------
    adjacency_ptrs_t range;
    store.adjacencyPoints (range, aim, side);

    ClosestPredicate cp (aim);
    auto it_min = boost::range::min_element (range, cp);
    if ( it_min == range.end () )
    { return  false; }

    *pRec = (**it_min);
    range.clear ();
    // ------------------------------------------------
    min = Point (aim.x - side, pRec->hit.y - side);
    max = Point (aim.x + side, pRec->hit.y);

    store.adjacencyRectPoints<adjacency_ptrs_t, ByP> (range, min, max);
    if ( range.empty () )
    { return false; }
    // ------------------------------------------------
    weightedMeanControls (aim, range, lower_controls, &lower_distance);
    // ------------------------------------------------
    min = Point (aim.x - side, pRec->hit.y);
    max = Point (aim.x + side, pRec->hit.y + side);

    store.adjacencyRectPoints<adjacency_ptrs_t, ByP> (range, min, max);
    if ( range.empty () )
    { return false; }
    // ------------------------------------------------
    weightedMeanControls (aim, range, upper_controls, &upper_distance);
    // ------------------------------------------------
    return true;
  }
  //------------------------------------------------------------------------------
};
