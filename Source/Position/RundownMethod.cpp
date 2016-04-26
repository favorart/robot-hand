#include "StdAfx.h"
#include "Position.h"

namespace Positions
{
  using namespace std;
  using namespace HandMoves;
  //------------------------------------------------------------------------------
  void  LearnMovements::rundownControlsOrder (IN const HandMoves::controling_t &init_controls,
                                              OUT      HandMoves::controling_t      &controls)
  {
    auto iter = controls.begin ();
    for ( auto j : hand.joints_ )
    {
      auto mo = muscleByJoint (j, true);
      auto mc = muscleByJoint (j, false);

      auto it_o = std::find (init_controls.begin (), init_controls.end (), mo);
      auto it_c = std::find (init_controls.begin (), init_controls.end (), mc);

      if ( it_o != init_controls.end () && it_c != init_controls.end () )
      {
        controls.insert (iter, *it_o);
        controls.insert (iter, *it_c);
      }
      else if ( it_o != init_controls.end () )
      {
        controls.insert (iter, *it_o);
        controls.insert (iter, Hand::Control (mc, it_o->last + 1, 1));
      }
      else if ( it_c != init_controls.end () )
      {
        controls.insert (iter, Hand::Control (mo, it_c->last + 1, 1));
        controls.insert (iter, *it_c);
      }
      else
      {
        controls.insert (iter, Hand::Control (mo, 0, 1));
        controls.insert (iter, Hand::Control (mc, 2, 1));
      }
    }
  }
  //------------------------------------------------------------------------------
  void  LearnMovements::rundownControls (IN OUT HandMoves::controling_t &controls)
  {
    const Hand::frames_t last_min = 1U;

    for ( auto j : hand.joints_ )
    {
      auto mo = muscleByJoint (j, true);
      auto mc = muscleByJoint (j, false);

      auto it_o = std::find (controls.begin (), controls.end (), mo);
      auto it_c = std::find (controls.begin (), controls.end (), mc);

      if ( it_o == controls.end () && it_c != controls.end () )
      { controls.push_back (Hand::Control (mo, it_c->last + 1, last_min)); }
      else if ( it_o == controls.end () )
      { controls.push_back (Hand::Control (mo, 0, last_min)); }

      if ( it_c == controls.end () ) // && it_o != controls.end () )
      { controls.push_back (Hand::Control (mc, it_o->last + 1, last_min)); }
    }
  }
  //------------------------------------------------------------------------------
  bool  LearnMovements::rundownNextControl (IN OUT HandMoves::controling_t &controls,
                                            IN OUT                  size_t &controls_curr,
                                            IN OUT          Hand::frames_t &velosity,
                                            IN OUT          Hand::frames_t &velosity_prev)
  {
    if ( controls_curr >= controls.size () )
    { return true; }
    // ---------------------------------
    auto it = controls.begin ();
    if ( controls_curr == 0U )
    {
      it->last += velosity;
      if ( it->start == 0U )
      {
        auto it_op = boost::range::find (controls, muscleOpposite (it->muscle));
        it_op->start = (it->last + 1U);
      }
    }
    else
    {
      std::advance (it, controls_curr / 2U);
      // ---------------------------------
      if ( (controls_curr % 2U) )
      {
        it->last -= (velosity_prev + velosity);
        if ( it->start == 0U )
        {
          auto it_op = boost::range::find (controls, muscleOpposite (it->muscle));
          it_op->start = (it->last + 1U);
        }
      }
      else
      {
        auto jt = std::prev (it);
        std::swap (it, jt);
        // ---------------------------------
        (it)->last += velosity_prev;
        (jt)->last += velosity;
        // ---------------------------------
        if ( it->start == 0U )
        {
          auto it_op = boost::range::find (controls, muscleOpposite (it->muscle));
          it_op->start = (it->last + 1U);
        }
        // ---------------------------------
        if ( jt->start == 0U )
        {
          auto it_op = boost::range::find (controls, muscleOpposite (jt->muscle));
          it_op->start = (jt->last + 1U);
        }
        // ---------------------------------
      }
    } // end else
      // ---------------------------------
    velosity_prev = velosity;
    // ---------------------------------
    ++controls_curr;
    return false;
  }
  //------------------------------------------------------------------------------
  size_t  LearnMovements::rundown (IN const Point &aim, OUT Point &hand_position, IN bool verbose)
  {
    size_t  rundown_complexity = 0U;
    // -----------------------------------------------
    const Record &rec = store.ClothestPoint (aim, side);
    Point hand_pos = rec.hit;
    // -----------------------------------------------
    HandMoves::controling_t  controls{ rec.controls };
    rundownControls (controls);
    // -----------------------------------------------
    double  distance = boost_distance (hand_pos, aim),
      start_distance = distance;
    // -----------------------------------------------
    Hand::frames_t  velosity = floor (distance / precision + 0.5);
    Hand::frames_t  velosity_prev = 0U;
    // -----------------------------------------------
    size_t  controls_curr = 0U;
    // -----------------------------------------------
    hand.SET_DEFAULT;
    while ( !rundownNextControl (controls, controls_curr,
                                 velosity, velosity_prev) )
    {
      // -----------------------------------------------
      if ( hand_act (aim, controls, hand_pos) )
      { ++rundown_complexity; }
      // -----------------------------------------------
      double next_distance = boost_distance (hand_pos, aim);
      // -----------------------------------------------
      double  prev_distance = start_distance;
      while ( next_distance < prev_distance )
      {
        prev_distance = next_distance;
        // -----------------------------------------------
        if ( next_distance < distance )
        {
          distance = next_distance;
          hand_position = hand_pos;

          if ( precision > distance )
          { break; }
        }
        // -----------------------------------------------
        velosity = floor (distance / precision + 0.5);
        // -----------------------------------------------
        auto it = controls.begin ();
        std::advance (it, controls_curr / 2U);
        // ---------------------------------
        if ( (controls_curr % 2U) )
        { it->last -= velosity; }
        else
        { it->last += velosity; }
        // ---------------------------------
        if ( it->start == 0U )
        {
          auto it_op = boost::range::find (controls, muscleOpposite (it->muscle));
          it_op->start = (it->last + 1U);
        }
        // ---------------------------------        
        velosity_prev = velosity;
        // -----------------------------------------------
        if ( hand_act (aim, controls, hand_pos) )
        { ++rundown_complexity; }
        // -----------------------------------------------
        next_distance = boost_distance (hand_pos, aim);
        // -----------------------------------------------
      } // while
      
      // -----------------------------------------------
      if ( precision > distance ) { break; }
      // -----------------------------------------------
    } // while
      // -----------------------------------------------
    if ( verbose )
    {
      tcout << _T ("prec: ") << distance << std::endl;
      tcout << _T ("rundown complexity: ")
            <<      rundown_complexity
            <<  std::endl << std::endl;
    }
    // -----------------------------------------------
    return rundown_complexity;
  }
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
  template <typename T> int  sgn (T val)
  { return (T (0) < val) - (val < T (0)); }
  /*  Размедение с повторениями:
   *  current       - текущее размещение
   *  alphabet_size - размер алфавита { 0, 1, ... k }
   */
  bool  next_placement_repeats (std::vector<int> &current, int alphabet_size)
  {
    bool    result = true;
    size_t  position = current.size ();
    while ( position > 0U && result )
    {
      ++current[position - 1U];
      result = (current[position - 1U] == alphabet_size);
      if ( result )
      { current[position - 1U] = 0U; }
      --position;
    }
    return (!result);
  }
  //------------------------------------------------------------------------------
  size_t  LearnMovements::rundownMethod (IN const Point &aim, OUT Point &hand_position, IN bool verbose)
  {
    size_t  rundown_complexity = 0U;
    // -----------------------------------------------
    const Record &rec = store.ClothestPoint (aim, side);
    Point hand_pos = rec.hit;
    // -----------------------------------------------
    double distance = boost_distance (hand_pos, aim),
      next_distance = distance,
     start_distance = distance;
    // -----------------------------------------------
    HandMoves::controling_t  controls;
    rundownControlsOrder (rec.controls, controls);
    // HandMoves::controling_t  controls{ rec.controls };
    // rundownControls (controls);
    // -----------------------------------------------
    std::vector<int>    last_steps (hand.muscles_.size ());
    std::vector<int>    last_prevs (hand.muscles_.size ());
    // -----------------------------------------------
    auto it_l = last_prevs.begin ();
    for ( auto it_c  = controls.begin ();
              (it_c != controls.end ()) && (it_l != last_prevs.end ());
             ++it_c, ++it_l )
    { *it_l = it_c->last; }
    // -----------------------------------------------
    int alphabet = 1;
    int alphabet2 = (alphabet % 2) ? (2 * alphabet + 1) : (2 * alphabet);
    int i = 0U;
    // -----------------------------------------------
    hand.SET_DEFAULT;
    while ( next_placement_repeats (last_steps, alphabet2) )
    {
      int  velosity = floor ((distance * 10) / precision + 0.5 );
      // int  velosity_prev = velosity;

      velosity = (velosity) ? velosity : 1;

      for ( auto &l : last_steps )
      {
        if ( l > alphabet )
        { l = (alphabet - l); }
        l *= velosity;
      }

      i = 0U;
      for ( auto it = controls.begin (); it != controls.end (); ++it, ++i )
      {
        if ( last_steps[i] != 0 )
        {
          if ( last_steps[i] < 0 )
          { it->last = (last_prevs[i] >= -last_steps[i]) ? (last_prevs[i] + last_steps[i]) : 0U; }
          else // if ( last_steps[i] > 0 )
          { it->last = (last_prevs[i] + last_steps[i]); }
          // --------------------------
          int   j = (i % 2) ? (i - 1) : (i + 1);
          auto op = (i % 2) ? std::prev (it) : std::next (it);
          // --------------------------
          Hand::frames_t  op_last;
          if ( last_steps[j] < 0 )
          { op_last = (last_prevs[j] >= -last_steps[j]) ? (last_prevs[j] + last_steps[j]) : 0U; }
          else // if (last_steps[j] > 0)
          { op_last = (last_prevs[j] + last_steps[j]); }
          // --------------------------
          if ( it->last > op_last )
          {
            op->start = it->last + 1U;
            it->start = 0U;
          }
          else
          {
            it->start = op_last + 1U;
            op->start = 0U;
          } // end else
        } // end if
      } // end for

      if ( hand_act (aim, controls, hand_pos) )
      { ++rundown_complexity; }

      next_distance = boost_distance (hand_pos, aim);
      if ( next_distance < precision )
      {
        hand_position = hand_pos;
        break;
      }

      while ( next_distance < distance )
      {
        distance = next_distance;
        hand_position = hand_pos;
        // -----------------------------------------------
        int  velosity_new = floor ((distance * 10) / precision + 0.5);
        velosity_new = (velosity_new) ? velosity_new : 1;

        if ( velosity_new != velosity )
        {
          for ( auto &l : last_steps )
          { l = sgn (l) * velosity_new; }

          velosity = velosity_new;
        }
        // -----------------------------------------------
        auto it_l = last_prevs.begin ();
        for ( auto it_c  = controls.begin (); (it_c != controls.end ())
               && (it_l != last_prevs.end ());
                 ++it_c, ++it_l )
        { *it_l = it_c->last; }
        // -----------------------------------------------
        i = 0U;
        for ( auto it = controls.begin (); it != controls.end (); ++it, ++i )
        {
          if ( last_steps[i] != 0 )
            {
              if ( last_steps[i] < 0 )
              { it->last = (last_prevs[i] >= -last_steps[i]) ? (last_prevs[i] + last_steps[i]) : 0U; }
              else
              { it->last = (last_prevs[i] + last_steps[i]); }
              // --------------------------
              int   j = (i % 2) ? (i - 1) : (i + 1);
              auto op = (i % 2) ? std::prev (it) : std::next (it);
              // --------------------------
              Hand::frames_t  op_last;
              if ( last_steps[j] < 0 )
              { op_last = (last_prevs[j] >= -last_steps[j]) ? (last_prevs[j] + last_steps[j]) : 0U; }
              else
              { op_last = (last_prevs[j] + last_steps[j]); }
              // --------------------------
              if ( it->last > op_last )
              {
                op->start = it->last + 1U;
                it->start = 0U;
              }
              else
              {
                it->start = op_last + 1U;
                op->start = 0U;
              } // end else
            } // end if
        } // end for
        // -----------------------------------------------
        if ( hand_act (aim, controls, hand_pos) )
        { ++rundown_complexity; }
        // -----------------------------------------------
        next_distance = boost_distance (hand_pos, aim);
        if ( next_distance < precision )
        {
          hand_position = hand_pos;
          break;
        }
      }

      if ( distance < start_distance )
      { break; }
      
      for ( auto &l : last_steps )
      {
        l = sgn (l);
        if ( l < 0 )
        { l = (alphabet - l); }
      }
      // -----------------------------------------------
#ifdef _DEBUG_PRINT
      tcout << _T ("prec: ") << best_distance << std::endl;
#endif // _DEBUG_PRINT

    } // end while

    // -----------------------------------------------
    if ( verbose )
    {
      tcout << _T ("rundown complexity: ")
            <<      rundown_complexity
            <<  std::endl << std::endl;
    }
    // -----------------------------------------------
    return rundown_complexity;
  }
  //------------------------------------------------------------------------------
};
