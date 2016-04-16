#include "StdAfx.h"
#include "Position.h"

namespace Positions
{
  using namespace std;
  using namespace HandMoves;
  //------------------------------------------------------------------------------
  void  insertRecordToBorders (borders_t &borders, const HandMoves::Record &rec)
  {
    for ( auto &ctrl : rec.controls () )
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
  void  defineBorders (borders_t &borders, RecTarget &target, HandMoves::Store &store, double distance)
  {
    for ( auto &rec : store )
    {
      if ( target.contain (rec.hit) )
      { insertRecordToBorders (borders, rec); } // end if
    } // end for

    HandMoves::adjacency_refs_t range;
    store.adjacencyPoints (range, (target.min) (), distance);
    for ( auto p_rec : range )
    { insertRecordToBorders (borders, *p_rec); }

    range.clear ();
    store.adjacencyPoints (range, Point ((target.min) ().x, (target.max) ().y), distance);
    for ( auto p_rec : range )
    { insertRecordToBorders (borders, *p_rec); }

    range.clear ();
    store.adjacencyPoints (range, Point ((target.max) ().x, (target.min) ().y), distance);
    for ( auto p_rec : range )
    { insertRecordToBorders (borders, *p_rec); }

    range.clear ();
    store.adjacencyPoints (range, (target.max) (), distance);
    for ( auto p_rec : range )
    { insertRecordToBorders (borders, *p_rec); }
  }
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
  class  TourWorkSpace
  {
    //---basics-----------------------------------------------
    HandMoves::Store &store; Hand &hand; RecTarget &target;
    size_t max_nested;
    //---iterating--------------------------------------------
    HandMoves::controling_t     controls;
    //---braking----------------------------------------------
    std::vector<Hand::Control>  arr_controlings;
    //--------------------------------------------------------
    borders_t           *borders;
    DirectionPredictor  *pd;
    //--------------------------------------------------------
    double                 step_distance;
    Hand::frames_t   lasts_step_increment;
    Hand::frames_t   lasts_step_braking = 1U;
    //--------------------------------------------------------
    Point     hand_pos_base;
    //--------------------------------------------------------
    size_t    complexity = 0U;
    counts_t  stats;
    //--------------------------------------------------------
    bool  b_target, b_braking;
    //--------------------------------------------------------

    Point last_point; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //--------------------------------------------------------
  public:
    TourWorkSpace (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target) :
      store (store), hand (hand), target (target),
      max_nested (hand.joints_.size ()),
      arr_controlings (max_nested),
      borders (nullptr), pd (nullptr)
    {
      hand.SET_DEFAULT;
      hand_pos_base = hand.position;
    }

    void  run (IN borders_t &borders, IN DirectionPredictor &pd,
               IN bool target, IN bool braking, IN double step_distance,
               IN Hand::frames_t lasts_step_increment)
    {
      this->borders = &borders;
      this->pd = &pd;
      // ----------------------------------------------------
      this->b_target = target;
      this->b_braking = braking;
      // ----------------------------------------------------
      this->step_distance = step_distance;
      this->lasts_step_increment = lasts_step_increment;
      // ----------------------------------------------------
      this->complexity = 0U;
      this->stats.clear ();
      // ----------------------------------------------------
      try
      {
        Point pt;
        runNestedForMuscle (pt, 0U);
      }
      catch ( boost::thread_interrupted& )
      { return; } // end catch

      // ----------------------------------------------------
      stats.print ();
      // ----------------------------------------------------
      tcout << _T ("\nSTAGE_2 Complexity: ") << complexity << std::endl;
      // ----------------------------------------------------
      tcout << _T ("\nLast point: ") << tstring (last_point) << std::endl;
    }

  private:
    bool  runNestedForMuscle (OUT Point &hand_pos_high, IN int joint_index)
    {
      bool  target_contain = true;
      Hand::frames_t start_i = 0U;
      Hand::frames_t last_lasts = 0U;
      bool  was_on_target = false;

      Point hand_pos_curr = hand_pos_base,
        hand_pos_prev = hand_pos_base;
      //------------------------------------------
      controls.push_back (Hand::Control ());
      Hand::Control &control_i = controls.back ();
      //------------------------------------------
      for ( auto muscle_i : { muscleByJoint (hand.joints_[joint_index], true),
                              muscleByJoint (hand.joints_[joint_index], false) } )
      {
        control_i.muscle = muscle_i;

        if ( (*borders)[muscle_i].first > 0
            && (*borders)[muscle_i].first < (*borders)[muscle_i].second )
        {
          Hand::frames_t lasts_step = lasts_step_increment;
          Hand::frames_t lasts_i_max = hand.maxMuscleLast (muscle_i);

          start_i = 0U;
          target_contain = true;

          for ( Hand::frames_t last_i = ((*borders)[muscle_i].first); // +(*borders)[muscle_i].second) / 3U;
                              (last_i < lasts_i_max) && (target_contain) && // || /* only last */ !was_on_target) &&
                              (last_i <= (*borders)[muscle_i].second - start_i);
            last_i += lasts_step )
          {
            control_i.last = last_i;
            //------------------------------------------
            if ( 0 <= joint_index && (joint_index + 1) < max_nested )
            { target_contain = runNestedForMuscle (hand_pos_curr, joint_index + 1) || !b_target; }
            else
            {

              // if ( b_target && !was_on_target )
              // {
              //   // while ( !was_on_target )
              //   {
              //     Point end = hand_pos_base;
              //     for ( auto c : controls )
              //       pd->predict (c, end);
              // 
              //     if ( target.contain (end) )
              //       was_on_target = true;
              //     else
              //       continue;
              //   }
              // }

              auto it = controls.begin ();
              for ( size_t ji = 0; ji <= joint_index; ++ji, ++it ) // auto &control : controls )
              {
                // control.last -= lasts_step_braking;
                if ( arr_controlings[ji].last )
                  arr_controlings[ji].start = it->last + 1;
              }

              target_contain = runNestedInserting (hand_pos_curr) || !b_target;
            }

            //if ( !was_on_target && target_contain )
            //  was_on_target = true;
            //------------------------------------------
            if ( last_i == (*borders)[muscle_i].first )
              hand_pos_high = hand_pos_curr;
            //=====================
            // if ( !joint_index )
            //   tcout << joint_index << ' ' << last_i << ' '
            //         << boost_distance (hand_pos_prev, hand_pos_curr) << endl;
            //=====================
            if ( boost_distance (hand_pos_prev, hand_pos_curr) > step_distance )
            {
              if ( lasts_step >= 2 * lasts_step_increment )
                lasts_step -= lasts_step_increment;
              else if ( lasts_step > lasts_step_increment )
                lasts_step -= lasts_step_increment;
              else if ( (lasts_step_increment > 1) && (lasts_step > lasts_step_increment / 2) )
                lasts_step -= lasts_step_increment / 2;
              else if ( b_braking && (lasts_step_increment == 1U) )
              {
                auto it = controls.begin ();
                for ( size_t ji = 0; ji <= joint_index; ++ji, ++it )
                  // for ( auto &control : controls )
                {
                  // control.last -= lasts_step_braking;
                  auto j = jointByMuscle (it->muscle);
                  auto mo = muscleByJoint (j, true);
                  auto mc = muscleByJoint (j, false);
                  auto opposite_muscle = (it->muscle & mo) ? mc : mo;
                  // auto opposite_muscle = (control.muscle % 2) ? mc : mo;

                  auto prev_last = arr_controlings[ji].last;
                  arr_controlings[ji] = (Hand::Control (opposite_muscle, it->last + 1,
                                         (prev_last ? prev_last : 30) + lasts_step_braking));
                }
                // arr_controlings[joint_index].push_back (Hand::Control (muscle_i, start_i, last_i));
                // control_i.start = start_i = last_i;
                // last_i = 1;
              }
            }
            else if ( boost_distance (hand_pos_prev, hand_pos_curr) < step_distance )
            {
              if ( arr_controlings[0].last )
              {
                if ( arr_controlings[0].last > lasts_step_braking )
                  for ( size_t ji = 0; ji <= joint_index; ++ji )
                    arr_controlings[ji].last -= lasts_step_braking;
              }
              else
              { lasts_step += lasts_step_increment; }
            }
            //-----------------------------
            hand_pos_prev = hand_pos_curr;
          } // end for (last)
          //------------------------------------------
          for ( size_t ji = 0; ji <= joint_index; ++ji )
            arr_controlings[ji].last = 0U;
          // arr_controlings[joint_index].clear ();
          //------------------------------------------
          if ( b_target )
          {
            start_i = 0U;
            target_contain = true;

            const Hand::frames_t lasts_step_increment_thick = 10U;

            lasts_step = lasts_step_increment_thick + 1U;
            for ( // Hand::frames_t last_i = ((*borders)[muscle_i].first + (*borders)[muscle_i].second) / 3U;
                 Hand::frames_t last_i = (*borders)[muscle_i].first;
                 (last_i - lasts_step) < lasts_i_max && target_contain;
                 last_i -= lasts_step )
            {
              control_i.last = last_i;

              if ( 0 <= joint_index && (joint_index + 1) < max_nested )
              { target_contain = runNestedForMuscle (hand_pos_curr, joint_index + 1) || !b_target; }
              else
              { target_contain = runNestedInserting (hand_pos_curr) || !b_target; }

              if ( last_i == (*borders)[muscle_i].first )
                hand_pos_high = hand_pos_curr;

              if ( boost_distance (hand_pos_prev, hand_pos_curr) > step_distance )
              {
                if ( lasts_step > lasts_step_increment_thick )
                  lasts_step -= lasts_step_increment_thick;
              }
              else if ( boost_distance (hand_pos_prev, hand_pos_curr) < step_distance )
              { lasts_step += lasts_step_increment_thick; }
              //-----------------------------
              hand_pos_prev = hand_pos_curr;
            } // end for (lasts)
          } // if (target)
        } // end if
      } // end for (muscle)
      //-----------------------------
      target_contain = true;
      controls.pop_back ();
      //-----------------------------
      return  target_contain;
    }
    bool  runNestedInserting (OUT Point &hand_position)
    {
      bool  target_contain = true;
      Point end = hand_pos_base;
      //----------------------------------------------
      bool  has_braking = b_braking && boost::algorithm::any_of (arr_controlings,
                                                                 [] (const Hand::Control &item)
                                                                 // { return  item.size (); });
      { return  item.last; });
      controling_t  new_controling;
      if ( has_braking )
      {
        for ( const Hand::Control &c : controls )
          new_controling.push_back (c);
        // controling_t stop_controls;

        /* записываем все разрывы */
        for ( auto &c : arr_controlings )
          // controls.splice (controls.end (), lst);
          // for ( auto &c : lst )
            // stop_controls.push_back (c);
          new_controling.push_back (c);

        // stop_controls.sort ();
        // controls.splice (controls.begin (), stop_controls);
        new_controling.sort ();
      }
      controling_t  &controling = (has_braking) ? new_controling : controls;
      //----------------------------------------------
      if ( b_target )
      {
        for ( auto &c : controls )
          pd->predict (c, end);
        //----------------------------------------------
        // stats.fill (hand, target, controling, end);
      }
      //----------------------------------------------
      /* типо на мишени */
      if ( !b_target || target.contain (end) )
      {
        //----------------------------------------------
        /* двигаем рукой */
        HandMoves::trajectory_t  trajectory;
        hand.SET_DEFAULT;
        hand.move (controling.begin (), controling.end (), &trajectory);
        last_point = hand_position = hand.position;
        //----------------------------------------------
        target_contain = target.contain (hand_position);
        //----------------------------------------------
        HandMoves::Record  rec (hand_position, hand_pos_base,
                                hand_position, controling,
                                trajectory);
        store.insert (rec);
        //----------------------------------------------
        ++complexity;
        boost::this_thread::interruption_point ();
        //----------------------------------------------
      } /* end if */
      else { target_contain = false; }
      //----------------------------------------------
      // while ( controls.size () > hand.joints_.size () )
      //   controls.pop_front ();
      //----------------------------------------------
      return  target_contain;
    }
  };
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
  /* грубое покрытие всего рабочего пространства */
  void  LearnMovements::STAGE_1 (IN  HandMoves::Store &store, IN Hand &hand, IN RecTarget &target)
  {
    borders_t  borders;
    defineBorders (borders, hand, 70U);
    DirectionPredictor  dp (hand);

    TourWorkSpace  tour (store, hand, target);
    tour.run (borders, dp, false, false, 0.05, 5U);
  }
  /* Покрытие всей мишени не слишком плотно */
  void  LearnMovements::STAGE_2 (IN  HandMoves::Store &store, IN Hand &hand, IN RecTarget &target)
  {
    borders_t  borders;
    defineBorders (borders, target, store, 0.005);
    DirectionPredictor  dp (hand);

    TourWorkSpace  tour (store, hand, target);
    tour.run (borders, dp, /* target */ true, /* braking */ true, 0.005, 1U);
  }
  /* Попадание в оставшиеся непокрытыми точки мишени */
  void  LearnMovements::STAGE_3 (IN  HandMoves::Store &store, IN Hand &hand, IN RecTarget &target)
  {}
  //------------------------------------------------------------------------------
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // !!! Борьба с КОЛИЧЕСТВОМ, которая перетягивает начальную точку !!!
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  void  LearnMovements::STAGE_3 (IN  HandMoves::Store &store, IN Hand &hand, IN const Point &aim,
                                 OUT HandMoves::trajectory_t    *trajectory,
                                 OUT HandMoves::trajectories_t  *trajectories)
  {
    complexity = 0U;
    // -----------------------------------------------    
    hand.SET_DEFAULT;
    Point  hand_position = hand.position,
           hand_pos_base = hand.position;
    // -----------------------------------------------
    double  distance = boost_distance (hand_position, aim);
    double  next_distance = distance;
    // -----------------------------------------------
    do
    {
      if ( next_distance < distance )
      { distance = next_distance; }
      // -----------------------------------------------
      Point  min (aim.x - side, aim.y - side),
             max (aim.x + side, aim.y + side);
      // -----------------------------------------------
#ifdef _DEBUG_PRINT
      tcout << std::endl;
      tcout << std::endl << _T ("LearnMovements::STAGE_3") << std::endl;
      tcout << _T (" min=") << min << _T ("  max=") << max << std::endl;
#endif // _DEBUG_PRINT
      // -----------------------------------------------
      HandMoves::adjacency_refs_t  range;
      // auto count = store.adjacencyPoints (range, aim, side);
      auto  count = store.adjacencyRectPoints<adjacency_refs_t, ByP> (range, min, max);
      if ( count == 0U )
      {
#ifdef _DEBUG_PRINT
        // tcout << _T ("ERROR: empty adjacency") << std::endl;
#endif // _DEBUG_PRINT
        // -------------
        return;
      }
      // -----------------------------------------------
      if ( trajectories )
      {
        for ( auto &pRec : range )
        { trajectories->push_back (pRec->trajectory); }
      }
      // -----------------------------------------------
      HandMoves::ClosestPredicate  cp (aim);
      range.sort (cp);
      // -----------------------------------------------
      double all_distance = 0.;
      if ( weighted_mean )
      {
        for ( auto &pRec : range )
        {
          all_distance += boost_distance (aim, pRec->hit);
          // -----------------------------------------------
#ifdef _DEBUG_PRINT
          // tcout << pRec->controls () << std::endl;
#endif // _DEBUG_PRINT
        }
#ifdef _DEBUG_PRINT
        tcout << std::endl;
#endif // _DEBUG_PRINT
      }
      // -----------------------------------------------
      controling_t  controls;
      for ( auto m : hand.muscles_ )
      {
        Hand::Control  control (m, 0U, 0U);
        // -----------------------------------------------
        Hand::frames_t  n_control_summands = 0U;
        for ( auto &pRec : range )
          for ( auto &c : pRec->controls () )
          {
            if ( c.muscle == m )
            {
              if ( weighted_mean )
              {
                /* взвешенное cреднее арифметическое */
                double weight = boost_distance (aim, pRec->hit) / all_distance;
                control.start += weight * c.start;
                control.last += weight * c.last;
              }
              else
              {
                /* обычное cреднее арифметическое */
                control.start += c.start;
                control.last += c.last;
                ++n_control_summands;
              } // end else
            } // end if
          } // end for-for
        // -----------------------------------------------
        if ( !weighted_mean )
        {
          if ( n_control_summands ) control.last  /= n_control_summands;
          if ( n_control_summands ) control.start /= n_control_summands;
        }
        // -----------------------------------------------
        if ( control.last )
        { controls.push_back (control); }
      }
      // -----------------------------------------------
      for ( auto j : hand.joints_ )
      {
        auto mo = muscleByJoint (j, true);
        auto mc = muscleByJoint (j, false);

        auto it_o = std::find (controls.begin (), controls.end (), mo);
        auto it_c = std::find (controls.begin (), controls.end (), mc);

        if ( it_o != controls.end () && it_c != controls.end () )
          if ( it_o->last > it_c->last )
          { it_c->start = it_o->last + 1U; }
          else
          { it_o->start = it_c->last + 1U; }
      }
      controls.sort ();
      // -----------------------------------------------
      {
        trajectory_t trajectory_;
        hand.move (controls.begin (), controls.end (), &trajectory_);
        hand.SET_DEFAULT;
        hand_position = hand.position;

        ++complexity;
        // -----------------------------------------------
        HandMoves::Record  rec (aim, hand_pos_base,
                                hand_position, controls,
                                trajectory_);
        store.insert (rec);
        // -----------------------------------------------
        if ( trajectory )
        { *trajectory = trajectory_; }
      }
      // -----------------------------------------------
      next_distance = boost_distance (hand_position, aim);
      if ( next_distance < distance )
      { distance = next_distance; }

      if ( precision >= distance )
      { break; }

#ifdef _DEBUG_PRINT
      tcout << _T ("Prec: ") << next_distance << std::endl;
#endif // _DEBUG_PRINT
      // -----------------------------------------------
      if ( distance > precision )
      { rundownMethod (store, hand, aim, controls, hand_position); }

      next_distance = boost_distance (hand_position, aim);
      if ( next_distance < distance )
      { distance = next_distance; }

      if ( complexity > 2000 ) // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      { break; }

      if ( precision >= distance )
      { break; }
      // -----------------------------------------------
      side -= side_decrease_step;
      // -----------------------------------------------
      controls.clear ();
      range.clear ();
      // -----------------------------------------------
    } while ( precision < distance || distance != next_distance );
    // -----------------------------------------------
#ifdef _DEBUG_PRINT_RES
    tcout << _T ("ResPrec: ") << distance << std::endl;
    tcout << _T ("Complexity: ") << complexity << std::endl << std::endl;
#endif // _DEBUG_PRINT_RES
  }
  //------------------------------------------------------------------------------
  void hand_act (HandMoves::Store &store, Hand &hand,
                 HandMoves::controling_t  &controls,
                 const Point &aim, Point &hand_position,
                 const Point &hand_pos_base) //,
                 //HandMoves::trajectory_t *trajectory=NULL)
  {
    HandMoves::trajectory_t trajectory;
    // -----------------------------------------------
    controling_t controls1 (controls);
    controls1.sort ();

    hand.move (controls1.begin (), controls1.end (), &trajectory);
    hand_position = hand.position;
    hand.SET_DEFAULT;
    // -----------------------------------------------
    HandMoves::Record  rec (aim, hand_pos_base,
                            hand_position, controls,
                            trajectory);
    store.insert (rec);
    // -----------------------------------------------
  }
  //------------------------------------------------------------------------------
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
  void  LearnMovements::rundownMethod     (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim,
                                           IN HandMoves::controling_t &init_controls, IN Point &hand_position)
  {
    size_t  rundown_complexity = 0U;
    lasts_step = 1U;
    // -----------------------------------------------
    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;
    Point hand_pos_best = hand_position;
    // -----------------------------------------------
    controling_t  controls;
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
    // -----------------------------------------------
#ifdef _DEBUG_PRINT
    for ( auto &c : controls )
    { tcout << c << std::endl; }
    tcout << std::endl;
#endif // _DEBUG_PRINT
    // -----------------------------------------------
    double distance = boost_distance (hand_position, aim);

    std::vector<int>    last_steps (hand.muscles_.size ());
    std::vector<int>    last_prevs (hand.muscles_.size ());
    // -----------------------------------------------
    auto it_l = last_prevs.begin ();
    for ( auto it_c = controls.begin (); (it_c != controls.end ())
                                      && (it_l != last_prevs.end ());
                                        ++it_c, ++it_l )
    { *it_l = it_c->last; }
    // -----------------------------------------------
    int alphabet = 1;
    int alphabet2 = (alphabet % 2) ? (2 * alphabet + 1) : (2 * alphabet);
    int i = 0U;
    // -----------------------------------------------
    while ( precision < distance )
    {
      if ( !next_placement_repeats (last_steps, alphabet2) )
      { break; }

      for ( auto &l : last_steps )
        if ( l > alphabet )
        { l = -(l - alphabet); }

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

      hand_act (store, hand, controls, aim, hand_position, hand_pos_base);
      ++rundown_complexity;

      double next_distance = boost_distance (hand_position, aim);
      if ( next_distance < distance )
      {
        distance = next_distance;
        hand_pos_best = hand_position;

        while ( precision < distance )
        {
          // -----------------------------------------------
          auto it_l = last_prevs.begin ();
          for ( auto it_c = controls.begin (); (it_c != controls.end ())
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

          hand_act (store, hand, controls, aim, hand_position, hand_pos_base);
          ++rundown_complexity;

          next_distance = boost_distance (hand_position, aim);
          if ( next_distance > distance )
          { break; }
          else
          {
            distance = next_distance;
            hand_pos_best = hand_position;
          }
        }
      }

      for ( auto &l : last_steps )
        if ( l < 0 )
        { l = (-l + alphabet); }
      // -----------------------------------------------
#ifdef _DEBUG_PRINT
      tcout << _T ("prec: ") << best_distance << std::endl;
#endif // _DEBUG_PRINT
    
    } // end while
    // -----------------------------------------------
    complexity += rundown_complexity;
#ifdef _DEBUG_PRINT_RES
    tcout << _T ("rundown complexity: ")
          <<      rundown_complexity
          <<  std::endl << std::endl;
#endif // _DEBUG_PRINT_RES
    hand_position = hand_pos_best;
  }
  void  LearnMovements::rundownMethod_old (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim,
                                           IN HandMoves::controling_t &init_controls, IN Point &hand_position)
  {
    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;
    // -----------------------------------------------
    controling_t controls;
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
        controls.insert (iter, Hand::Control (mc, it_o->last + 1, 50));
      }
      else if ( it_c != init_controls.end () )
      {
        controls.insert (iter, Hand::Control (mo, it_c->last + 1, 50));
        controls.insert (iter, *it_c);
      }
      else
      {
        controls.insert (iter, Hand::Control (mo, 0, 51));
        controls.insert (iter, Hand::Control (mc, 52, 50));
      }
    }
    // -----------------------------------------------
    controls.sort ();
    // -----------------------------------------------
    std::list<shared_ptr<Hand::Control>>  ordered_controls;
    for ( auto &c : controls )
    { ordered_controls.push_back (std::make_shared<Hand::Control> (c)); }
    ordered_controls.sort ([](const shared_ptr<Hand::Control> &sca,
                              const shared_ptr<Hand::Control> &scb)
                             { return sca->muscle < scb->muscle; });
    // -----------------------------------------------
    lasts_step = 1U;
    double distance = boost_distance (hand_position, aim);
    // -----------------------------------------------
    while ( precision < distance )
    {
      // -----------------------------------------------
      auto it_straight = ordered_controls.begin ();
      auto it_opposite = std::next (it_straight);

      if ( (**it_straight).last < (**it_opposite).last )
      { std::swap (it_straight, it_opposite); }
      // -----------------------------------------------
      double best_distance = distance;
      for ( auto j : hand.joints_ )
      {
        Hand::frames_t last_straight = (**it_straight).last;
        Hand::frames_t last_opposite = (**it_opposite).last;

        Hand::frames_t best_last_straight = 0U;
        Hand::frames_t best_last_opposite = 0U;

        double next_distance = distance;
        double prev_distance = distance;
        // ---------------------------------------------------------
        do
        {
          prev_distance = next_distance;
          (**it_straight).last += lasts_step;
          (**it_opposite).start += lasts_step;

#ifdef _DEBUG_PRINT
          for ( auto &c : controls )
          { tcout << c << std::endl; }
#endif // _DEBUG_PRINT

          hand_act (store, hand, controls, aim, hand_position, hand_pos_base); // , NULL);
          next_distance = boost_distance (hand_position, aim);

          if ( next_distance >= prev_distance )
          {
            (**it_straight).last -= lasts_step;
            (**it_opposite).start -= lasts_step;

            do
            {
              prev_distance = next_distance;
              (**it_opposite).last -= lasts_step;
              hand_act (store, hand, controls, aim, hand_position, hand_pos_base); // , NULL);
              next_distance = boost_distance (hand_position, aim);
            } while ( next_distance < prev_distance );
          }

        } while ( next_distance < prev_distance );

        if ( prev_distance < best_distance )
        {
          best_last_straight = (**it_straight).last;
          best_last_opposite = (**it_opposite).last;

          best_distance = prev_distance;
        }
        // ---------------------------------------------------------
        (**it_straight).last = last_straight;
        (**it_opposite).last = last_opposite;

        do
        {
          prev_distance = next_distance;
          (**it_straight).last -= lasts_step;
          (**it_opposite).start -= lasts_step;

          hand_act (store, hand, controls, aim, hand_position, hand_pos_base); // , NULL);
          next_distance = boost_distance (hand_position, aim);

          if ( next_distance >= prev_distance )
          {
            (**it_straight).last += lasts_step;
            (**it_opposite).start += lasts_step;
            do
            {
              prev_distance = next_distance;
              (**it_opposite).last += lasts_step;

              hand_act (store, hand, controls, aim, hand_position, hand_pos_base); // , NULL);
              next_distance = boost_distance (hand_position, aim);
            } while ( next_distance < prev_distance );
          }

        } while ( next_distance < prev_distance );

        if ( prev_distance < best_distance )
        {
          best_last_straight = (**it_straight).last;
          best_last_opposite = (**it_opposite).last;

          best_distance = prev_distance;
        }
        // ---------------------------------------------------------
        (**it_straight).last = best_last_straight;
        (**it_opposite).last = best_last_opposite;

        distance = best_distance;

#ifdef _DEBUG_PRINT
        tcout << _T ("prec: ") << best_distance << std::endl;
#endif // _DEBUG_PRINT
        ++it_straight;
        ++it_opposite;
      }
    }
    // -----------------------------------------------
#ifdef _DEBUG_PRINT_RES
    tcout << _T ("prec: ") << distance << std::endl;
#endif // _DEBUG_PRINT_RES
  }
  //------------------------------------------------------------------------------
  void  LearnMovements::gradientMethod (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim,
                                        IN HandMoves::controling_t &init_controls, IN Point &hand_position)
  {
    size_t  gradient_complexity = 0U;
    // -----------------------------------------------
    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;
    Point hand_pos_best = hand_position;
    // -----------------------------------------------
    Point  min (aim.x - side, aim.y - side),
           max (aim.x + side, aim.y + side);
    // -----------------------------------------------
    adjacency_refs_t  range;
    store.adjacencyRectPoints<adjacency_refs_t, ByP> (range, min, max);
    // -----------------------------------------------
    double distance = boost_distance (hand_position, aim);
    while ( precision < distance )
    {

      // -----------------------------------------------
      HandMoves::controling_t controls;
      // -----------------------------------------------
      hand_act (store, hand, controls, aim, hand_position, hand_pos_base);
      ++gradient_complexity;
      // -----------------------------------------------
#ifdef _DEBUG_PRINT
      tcout << _T ("prec: ") << best_distance << std::endl;
#endif // _DEBUG_PRINT
      // -----------------------------------------------
    } // end while
    // -----------------------------------------------
    complexity += gradient_complexity;
#ifdef _DEBUG_PRINT_RES
    tcout << _T ("gradient complexity: ")
          <<      gradient_complexity
          <<   std::endl << std::endl;
#endif // _DEBUG_PRINT_RES
    hand_position = hand_pos_best;
  }
  //------------------------------------------------------------------------------
};
