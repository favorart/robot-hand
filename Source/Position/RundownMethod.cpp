#include "StdAfx.h"
#include "Position.h"
#include "Utilities.h"
#include "Combinations.h"


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

      if ( it_c == controls.end () )
      {
        if ( it_o == controls.end () )
        { it_o = std::find (controls.begin (), controls.end (), mo); }
        controls.push_back (Hand::Control (mc, it_o->last + 1, last_min));
      }
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
        auto  m_op = muscleOpposite (it->muscle);
        auto it_op = boost::range::find (controls, m_op);
        it_op->start = (it->last + 1U);
      }
    }
    else
    {
      std::advance (it, controls_curr / 2U);
      // ---------------------------------
      if ( (controls_curr % 2U) )
      {
        if ( it->last >= (velosity_prev + velosity) )
        { it->last -= (velosity_prev + velosity); }
        else
        {
          auto  m_op = muscleOpposite (it->muscle);
          auto it_op = boost::range::find (controls, m_op);
          // ---------------------------------
          it_op->last += (velosity_prev + velosity - it->last);
          it->last = 0U;
          // ---------------------------------
          if ( it_op->start == 0U )
          {
            it->start = (it_op->last + 1U);
            it_op->start = 0U;
          }
          else
          {
            it_op->start = (it->last + 1U);
            it->start = 0U;
          }
        } // end else

        if ( it->start == 0U )
        {
          auto  m_op = muscleOpposite (it->muscle);
          auto it_op = boost::range::find (controls, m_op);
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
          auto  m_op = muscleOpposite (it->muscle);
          auto it_op = boost::range::find (controls, m_op);
          it_op->start = (it->last + 1U);
        }
        // ---------------------------------
        if ( jt->start == 0U )
        {
          auto  m_op = muscleOpposite (jt->muscle);
          auto jt_op = boost::range::find (controls, m_op);
          jt_op->start = (jt->last + 1U);
        }
        // ---------------------------------
      } // end else
    } // end else
    // ---------------------------------
    velosity_prev = velosity;
    // ---------------------------------
    ++controls_curr;
    return false;
  }
  //------------------------------------------------------------------------------
  size_t  LearnMovements::rundownMain (IN const Point &aim, OUT Point &hand_position, IN bool verbose)
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
      if ( handAct (aim, controls, hand_pos) )
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
        { 
          if ( it->last >= velosity )
          { it->last -= velosity; }
          else
          {
            auto  m_op = muscleOpposite (it->muscle);
            auto it_op = boost::range::find (controls, m_op);
            // ---------------------------------
            it_op->last += (velosity - it->last);
            it->last = 0U;
            // ---------------------------------
            if ( it_op->start == 0U )
            { it->start = (it_op->last + 1U); }
            else
            { it_op->start = (it->last + 1U); }
          }
        }
        else
        { it->last += velosity; }
        // ---------------------------------
        if ( it->start == 0U )
        {
          auto  m_op = muscleOpposite (it->muscle);
          auto it_op = boost::range::find (controls, m_op);
          it_op->start = (it->last + 1U);
        }
        // ---------------------------------        
        velosity_prev = velosity;
        // -----------------------------------------------
        if ( handAct (aim, controls, hand_pos) )
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
  class RundownFullIncrementor
  {
    const size_t directions_ = 2U; // +/-
    size_t  n_joints_;
    size_t  n_combs_ = 1U;

    std::vector<int>  alphabet_;

  public:
    RundownFullIncrementor (size_t n_joints) :
      n_joints_ (n_joints),
      alphabet_ (2 * n_joints * directions_)
    { reset (); }

    void  reset ()
    { 
      n_combs_ = 1U;
      // --------------------------------------------------
      int inc = 0;
      // --------------------------------------------------
      for ( auto &letter : alphabet_ )
      { letter = inc++; }
    }
    bool  next (std::vector<int> &currents)
    {
      bool result, one_more;
      // --------------------------------------------------
      do
      {
        one_more = false;
        currents.assign (currents.size (), 0);
        // --------------------------------------------------
        int  it_curr = 0,
             it_prev = static_cast<int> (currents.size ());
        // --------------------------------------------------
        for ( auto it  = alphabet_.begin ();
                   it != alphabet_.begin () + n_combs_;
                 ++it )
        {
          it_curr = *it % currents.size ();
          if ( n_combs_ > 1 && it_prev == it_curr )
          {
            one_more = true;
            break;
          }
          // --------------------------------------------------
          currents[it_curr] = (*it >= currents.size ()) ? -1 : 1;
          it_prev = it_curr;
        } // end for
        // --------------------------------------------------
        result = next_combination (alphabet_.begin (),
                                   alphabet_.begin () + n_combs_,
                                   alphabet_.end ());
        // --------------------------------------------------
        if ( !result )
        { ++n_combs_;
          if ( n_combs_ > n_joints_ )
          { reset (); result = true; }
        }
        // --------------------------------------------------
      } while ( one_more );
      // --------------------------------------------------
      return result;
    }
  };
  //------------------------------------------------------------------------------
  bool    LearnMovements::rundownNextControl (IN OUT HandMoves::controling_t &controls,
                                              IN OUT       std::vector<int>  &lasts_changes,
                                              IN OUT          Hand::frames_t &velosity)
  {
    bool result = true;
    // ---------------------------------
    if ( controls.size () != lasts_changes.size () )
      throw std::exception ("rundownNextControl: not equal sizes controls and lasts_changes");
    // ---------------------------------
    auto it = controls.begin ();
    for ( auto last_change : lasts_changes )
    {
      if ( last_change )
      {
        // ---------------------------------
        if ( last_change < 0 )
        {
          if ( it->last >= velosity )
          { it->last -= velosity; }
          else
          { 
            velosity = it->last;
            it->last = 0U;
          }

          // {
          //   auto  m_op = muscleOpposite (it->muscle);
          //   auto it_op = boost::range::find (controls, m_op);
          //   // ---------------------------------
          //   it_op->last += (velosity - it->last);
          //   it->last = 0U;
          //   // ---------------------------------
          //   if ( it_op->start < it->start )
          //   {
          //     it->start = (it_op->last + 1U);
          //     it_op->start = 0U;
          //   }
          //   else // it_op->start > it->start
          //   {
          //     it_op->start = (it->last + 1U);
          //     it->start = 0U;
          //   }
          // } // end else

          auto  m_op = muscleOpposite (it->muscle);
          auto it_op = boost::range::find (controls, m_op);

          if ( it->start < it_op->start )
          { it_op->start = (it->last + 1U); }
        }
        else // last_change > 0
        {
          it->last += velosity;
          // ---------------------------------
          auto  m_op = muscleOpposite (it->muscle);
          auto it_op = boost::range::find (controls, m_op);

          if ( it->start < it_op->start )
          { it_op->start = (it->last + 1U); }
          // ---------------------------------
        } // end else
        // ---------------------------------
        result = false;
      } // end if
      ++it;
    } // end for
    // ---------------------------------
    return result;
  }
  //------------------------------------------------------------------------------
  size_t  LearnMovements::rundownFull (IN const Point &aim, OUT Point &hand_position, IN bool verbose)
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
    HandMoves::controling_t  controls{ rec.controls };
    rundownControls (controls);
    // -----------------------------------------------
    Hand::frames_t  velosity = 0U;
    Hand::frames_t  velosity_prev = 0U;
    // -----------------------------------------------
    std::vector<int>  lasts_changes ( hand.muscles_.size () );
    // -----------------------------------------------
    RundownFullIncrementor   increm ( hand.joints_.size () );
    // -----------------------------------------------
    hand.SET_DEFAULT;
    while ( distance > precision )
    {
      velosity = floor (distance / precision + 0.5);
      velosity = (velosity) ? velosity : 1U;
      // -----------------------------------------------
      /* Восстановить прошлое управление */
      for ( auto &last_change : lasts_changes )
      {
        if ( last_change != 0 )
        { last_change = -last_change; }
      }
      rundownNextControl (controls, lasts_changes, velosity_prev);
      // -----------------------------------------------
      /* Взять новое сочетание */
      bool increm_result = increm.next (lasts_changes);
      // -----------------------------------------------
      rundownNextControl (controls, lasts_changes, velosity);

      velosity_prev = velosity;
      // -----------------------------------------------
      if ( handAct (aim, controls, hand_pos) )
      { ++rundown_complexity; }
      // -----------------------------------------------
      next_distance = boost_distance (hand_pos, aim);
      if ( next_distance < precision )
      {
        hand_position = hand_pos;
        break;
      }
      // -----------------------------------------------
      while ( next_distance < distance )
      {
        distance = next_distance;
        hand_position = hand_pos;
        // -----------------------------------------------
        Hand::frames_t  velosity_new = floor ((distance) / precision + 0.5);
        velosity_new = (velosity_new) ? velosity_new : 1U;

        if ( velosity_new != velosity )
        {
          velosity = velosity_new;
          for ( auto &l : lasts_changes )
          { l = sign (l) * velosity; }
        }
        // -----------------------------------------------
        rundownNextControl (controls, lasts_changes, velosity);

        velosity_prev = velosity;
        // -----------------------------------------------
        if ( handAct (aim, controls, hand_pos) )
        { ++rundown_complexity; }
        // -----------------------------------------------
        next_distance = boost_distance (hand_pos, aim);
        if ( next_distance < precision )
        {
          hand_position = hand_pos;
          break;
        }

        if ( next_distance >= distance )
        { increm.reset ();
          increm_result = false;
        }
      }
      // -----------------------------------------------
      // if ( next_distance > side )
      if ( increm_result )
      { /* FAIL */
        break;
      }
      // -----------------------------------------------
      // if ( distance < start_distance )
      // { break; }
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
