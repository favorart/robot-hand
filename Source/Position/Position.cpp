#include "StdAfx.h"
#include "Position.h"

namespace Positions
{
  using namespace std;
  using namespace HandMoves;
  //------------------------------------------------------------------------------
  void  insertRecordToBorders (borders_t &borders, const HandMoves::Record &rec)
  {
    for ( auto &ctrl : rec.controls )
    {
      for ( auto m : muscles )
      {
        if ( m & ctrl.muscle )
        {
          auto it = borders.find (m);
          if ( it != borders.end () )
          {
            if ( ctrl.last < it->second.first )
              it->second.first = ctrl.last;
            if ( ctrl.last > it->second.second )
              it->second.second = ctrl.last;
          }
          else
          { borders[ctrl.muscle] = std::make_pair (ctrl.last, ctrl.last); }
        } // end if
      } // end for
    } // end for
  }
  //------------------------------------------------------------------------------
  void  defineBorders (borders_t &borders, Hand &hand, Hand::frames_t lasts_init)
  {
    for ( auto muscle : hand.muscles_ )
    { borders[muscle] = std::make_pair (lasts_init, hand.maxMuscleLast (muscle)); }
  }
  /* Статистичеки найти приблизительную границу */
  void  defineBorders (borders_t &borders, RecTarget &target,
                       HandMoves::Store &store, double side)
  {
    for ( auto &rec : store )
    {
      if ( target.contain (rec.hit) )
      { insertRecordToBorders (borders, rec); } // end if
    } // end for

    HandMoves::adjacency_ptrs_t range;
    store.adjacencyPoints (range, (target.min) (), side);
    for ( auto p_rec : range )
    { insertRecordToBorders (borders, *p_rec); }

    range.clear ();
    store.adjacencyPoints (range, Point ((target.min) ().x, (target.max) ().y), side);
    for ( auto p_rec : range )
    { insertRecordToBorders (borders, *p_rec); }

    range.clear ();
    store.adjacencyPoints (range, Point ((target.max) ().x, (target.min) ().y), side);
    for ( auto p_rec : range )
    { insertRecordToBorders (borders, *p_rec); }

    range.clear ();
    store.adjacencyPoints (range, (target.max) (), side);
    for ( auto p_rec : range )
    { insertRecordToBorders (borders, *p_rec); }
  }
  //------------------------------------------------------------------------------
  bool  LearnMovements::handAct (IN  const Point &aim,
                                  IN  const HandMoves::controling_t  &controls,
                                  OUT Point &hand_position,
                                  IN  bool copy)
  {
    ControlingHasher ch;
    size_t  h = ch (controls);
    // -----------------------------------------------
    boost::this_thread::interruption_point ();
    // -----------------------------------------------
    const Record  *pRec = store.ExactRecordByControl (controls);
    if ( pRec ) // visited.find (h) != visited.end () )
    {
      // if ( pRec )
      // { 
        hand_position = pRec->hit;
        return false;
      // }
      // else { throw exception ("handAct: Not in Store"); }
    }
    
    {
      HandMoves::trajectory_t trajectory;
      // -----------------------------------------------
      // visited.insert (h);
      // -----------------------------------------------
      if ( copy )
      {
        HandMoves::controling_t  controls_copy (controls);
        controls_copy.sort ();
        hand.move (controls_copy.begin (), controls_copy.end (), &trajectory);
      }
      else
      { hand.move (controls.begin (), controls.end (), &trajectory); }
      // -----------------------------------------------
      hand_position = hand.position;
      hand.SET_DEFAULT;
      // -----------------------------------------------
      HandMoves::Record  rec (aim, hand_pos_base, hand_position,
                              controls, trajectory);
      store.insert (rec);
      // -----------------------------------------------
    }
    return true;
  }
  //------------------------------------------------------------------------------  
};
