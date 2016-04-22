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
    borders_t  borders_;

    borders_t           *borders;
    DirectionPredictor  *pd;
    //--------------------------------------------------------
    double                 step_distance;
    Hand::frames_t   lasts_step_increment;
    Hand::frames_t   lasts_step_braking = 5U;
    //--------------------------------------------------------
    Point     hand_pos_base;
    //--------------------------------------------------------
    size_t    complexity = 0U;
    counts_t  stats;
    //--------------------------------------------------------
    bool  b_distance, b_target, b_braking;
    //--------------------------------------------------------
    double  max_distance;
    const Point &center;

    // Point last_point; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //--------------------------------------------------------
  public:
    TourWorkSpace (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target) :
      store (store), hand (hand), target (target),
      max_nested (hand.joints_.size ()),
      arr_controlings (max_nested),
      center (target.center ()),
      borders (nullptr), pd (nullptr)
    {
      hand.SET_DEFAULT;
      hand_pos_base = hand.position;
      max_distance = (target.max ().x - target.min ().x ) / 2.3; // (target.center (), target.max () /* + 0.001*/); // !!!!!!!!!!!!!!!
    }

    void  run (IN borders_t &borders, IN DirectionPredictor &pd,
               IN bool distance, IN bool target, IN bool braking,
               IN double step_distance,
               IN Hand::frames_t lasts_step_increment)
    {
      this->borders = &borders;
      this->pd = &pd;
      // ----------------------------------------------------
      this->b_target = target;
      this->b_braking = braking;
      this->b_distance = distance;
      // ----------------------------------------------------
      this->step_distance = step_distance;
      this->lasts_step_increment = lasts_step_increment;
      // ----------------------------------------------------
      this->complexity = 0U;
      this->stats.clear ();
      // ----------------------------------------------------
      this->borders_ = borders;
      // ----------------------------------------------------
      try
      {
        Point pt;
        std::vector<Hand::frames_t>  lasts_i (hand.joints_.size ());
        runNestedForMuscle (pt, 0U);
      }
      catch ( boost::thread_interrupted& )
      { return; } // end catch

      // ----------------------------------------------------
      stats.print ();
      // ----------------------------------------------------
      tcout << _T ("\nComplexity: ") << complexity << std::endl;
      // ----------------------------------------------------
      // tcout << _T ("\nLast point: ") << tstring (last_point) << std::endl;
    }

  private:
    bool  runNestedForMuscle (OUT Point &hand_pos_high, IN int joint_index) //,
                              // IN  std::vector<Hand::frames_t> &lasts_i)
    {
      bool  target_contain = true;
      Hand::frames_t start_i = 0U;
      Hand::frames_t last_lasts = 0U;
      bool  was_on_target = false;

      int ml;

      Point hand_pos_curr = hand_pos_base,
            hand_pos_prev = hand_pos_base;

      int high = 0;
      Point   max_hand_pos_high{ 0., 0. };
      //------------------------------------------
      controls.push_back (Hand::Control ());
      Hand::Control &control_i = controls.back ();
      //------------------------------------------
      for ( auto muscle_i : { muscleByJoint (hand.joints_[joint_index], (ml = true)),
                              muscleByJoint (hand.joints_[joint_index], (ml = false)) } )
      {
        control_i.muscle = muscle_i;

        auto &board = borders_[muscle_i];
        if ( board.first > 0
          && board.first < board.second )
        {
          Hand::frames_t lasts_step  = lasts_step_increment;
          Hand::frames_t lasts_i_max = hand.maxMuscleLast (muscle_i);

          start_i = 0U;
          target_contain = true;

          Hand::frames_t last_i = 0U;
          for ( last_i = (board.first); // +(*borders)[muscle_i].second) / 3U;
               (last_i < lasts_i_max) && (target_contain || !was_on_target) &&
               (last_i <= board.second - start_i);
                last_i += lasts_step )
          {
            control_i.last = last_i;
            //------------------------------------------
            if ( 0 <= joint_index && (joint_index + 1) < max_nested )
            {
              //===============================================================
              target_contain = runNestedForMuscle (hand_pos_curr, joint_index + 1) || !b_target;
              //===============================================================
              if ( b_target && (target_contain && !was_on_target) )
              {
                board.first = last_i;
                was_on_target = true;
              }
            }
            else
            {
              auto it = controls.begin ();
              for ( size_t ji = 0; ji <= joint_index; ++ji, ++it )
              {
                // control.last -= lasts_step_braking;
                if ( arr_controlings[ji].last )
                  arr_controlings[ji].start = it->last + 1;
              }
              //===============================================================
              target_contain = runNestedInserting (hand_pos_curr) || !b_target;
              //===============================================================
              if ( b_target && (target_contain && !was_on_target) )
              {
                board.first = last_i;
                was_on_target = true;
              }
            } // end else (insert)
            
            //------------------------------------------
            // if ( last_i == (*borders)[muscle_i].first )
            {
              // if ( boost_distance (hand_pos_high, max_hand_pos_high)
              //    < boost_distance (hand_pos_high, hand_pos_curr) )
              { 
                max_hand_pos_high.x += hand_pos_curr.x;
                max_hand_pos_high.y += hand_pos_curr.y;
                ++high;
              }
            }
            //------------------------------------------
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
                {
                  auto j = jointByMuscle (it->muscle);
                  auto mo = muscleByJoint (j, true);
                  auto mc = muscleByJoint (j, false);
                  auto opposite_muscle = (it->muscle & mo) ? mc : mo;
                  // auto opposite_muscle = (control.muscle % 2) ? mc : mo;

                  auto prev_last = arr_controlings[ji].last;
                  arr_controlings[ji] = (Hand::Control (opposite_muscle, it->last + 1,
                                                        (prev_last ? prev_last : 30) +
                                                        lasts_step_braking));
                }
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
              else // if ( hand_pos_curr != hand_pos_base || lasts_step < 10 ) // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
              { lasts_step += lasts_step_increment; }
            }
            //-----------------------------
            hand_pos_prev = hand_pos_curr;
          } // end for (last)
          //} // end else (insert)

          if ( b_target )
          { board.second = last_i; }
          //------------------------------------------
          for ( size_t ji = 0; ji <= joint_index; ++ji )
            arr_controlings[ji].last = 0U;
          // arr_controlings[joint_index].clear ();
          //------------------------------------------
          if ( b_target )
          {
            start_i = 0U;
            target_contain = true;
            //------------------------------------------
            const Hand::frames_t  lasts_step_increment_thick = 10U;
            //------------------------------------------
            lasts_step = lasts_step_increment_thick + 1U;
            for ( // last_i = (board.first + board.second) / 3U;
                  last_i  = board.first;
                 (last_i -  lasts_step) < lasts_i_max && target_contain;
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
        } // end if (thick)
      } // end for (muscle)
      //-----------------------------
      target_contain = true;
      controls.pop_back ();
      if ( high )
      { hand_pos_high = Point{ max_hand_pos_high.x / high, max_hand_pos_high.y / high }; }
      //-----------------------------
      return  target_contain;
    }
    bool  runNestedInserting (OUT Point &hand_position)
    {
      bool  target_contain = true;
      Point end = hand_pos_base;
      //----------------------------------------------
      bool  has_braking = b_braking && boost::algorithm::any_of (arr_controlings,
                           [] (const Hand::Control &item) { return item.last; });
      controling_t  new_controling;
      if ( has_braking )
      {
        for ( const Hand::Control &c : controls )
        { new_controling.push_back (c); }
        /* записываем все разрывы */
        for ( auto &c : arr_controlings )
        { new_controling.push_back (c); }
        new_controling.sort ();
      }
      controling_t  &controling = (has_braking) ? new_controling : controls;
      //----------------------------------------------
      if ( b_target || b_distance )
      {
        for ( auto &c : controls )
          pd->predict (c, end);

        end.x += 0.1;
        end.y -= 0.05;
        //----------------------------------------------
        // stats.fill (hand, target, controling, end);
      }
      //----------------------------------------------
      /* типо на мишени */
      if ( (!b_target   || target.contain (end))
        && (!b_distance || boost_distance (center, end) < max_distance) )
      {
        HandMoves::trajectory_t  trajectory;
        //----------------------------------------------
        hand.SET_DEFAULT;
        /* двигаем рукой */
        hand.move (controling.begin (), controling.end (), &trajectory);
        /* last_point = */ hand_position = hand.position;
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
    tour.run (borders, dp, true, false, false, 0.03, 3U);
  }
  /* Покрытие всей мишени не слишком плотно */
  void  LearnMovements::STAGE_2 (IN  HandMoves::Store &store, IN Hand &hand, IN RecTarget &target)
  {
    borders_t  borders;
    defineBorders (borders, target, store, 0.005);
    DirectionPredictor  dp (hand);

    TourWorkSpace  tour (store, hand, target);
    tour.run (borders, dp, /* distance */ false, /* target */ true, /* braking */ true, 0.01, 3U);
  }
  /* Попадание в оставшиеся непокрытыми точки мишени */
  void  LearnMovements::STAGE_3 (IN  HandMoves::Store &store, IN Hand &hand, IN RecTarget &target)
  {
    for ( const auto &aim : target.coords () )
    {
      HandMoves::adjacency_refs_t range;
      auto count = store.adjacencyPoints (range, aim, precision);

      HandMoves::ClosestPredicate  cp (aim);
      auto it_min = boost::range::min_element (range, cp);
      if ( !count || it_min == range.end () ) // || boost_distance (aim, (**it_min).hit) )
      { Close (store, hand, aim, 0.05, NULL, NULL); }
    }
  }
  //------------------------------------------------------------------------------
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // !!! Борьба с КОЛИЧЕСТВОМ, которая перетягивает начальную точку !!!
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  void  LearnMovements::Mean  (IN  HandMoves::Store &store, IN Hand &hand, IN const Point &aim, IN double side,
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
      // Point  min (aim.x - side, aim.y - side),
      //        max (aim.x + side, aim.y + side);
      // -----------------------------------------------
#ifdef _DEBUG_PRINT
      tcout << std::endl;
      tcout << std::endl << _T ("LearnMovements::Mean")  << std::endl;
      tcout << _T ("min=") << min << _T (" max=") << max << std::endl;
#endif // _DEBUG_PRINT
      // -----------------------------------------------
      HandMoves::adjacency_refs_t  range;
      // size_t  count = store.adjacencyPoints (range, aim, side);
      // size_t  count = store.adjacencyRectPoints<adjacency_refs_t, ByP> (range, min, max);
      size_t  count = store.adjacencyByPBorders (range, aim, side);
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
      // HandMoves::ClosestPredicate  cp (aim);
      // range.sort (cp);
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
          for ( auto &c : pRec->controls )
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

  void  LearnMovements::Close (IN  HandMoves::Store &store, IN Hand &hand, IN const Point &aim, IN double side,
                               OUT HandMoves::trajectory_t    *trajectory,
                               OUT HandMoves::trajectories_t  *trajectories)
  {
    complexity = 0U;
    // -----------------------------------------------    
    hand.SET_DEFAULT;
    Point  hand_position = hand.position,
           hand_pos_base = hand.position;
    // -----------------------------------------------
    double  distance;
    // -----------------------------------------------
    Point  min (aim.x - side, aim.y - side),
           max (aim.x + side, aim.y + side);
    // -----------------------------------------------
#ifdef _DEBUG_PRINT
    tcout << std::endl;
    tcout << std::endl << _T ("LearnMovements::Close") << std::endl;
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
    auto it_min = boost::range::min_element (range, cp);
    // -----------------------------------------------
    hand_position = range.front ()->hit;

    distance = boost_distance (hand_position, aim);
    if ( distance > precision )
    { rundownMethod (store, hand, aim, range.front ()->controls, hand_position); }

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
                 const Point &hand_pos_base)
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
                                           IN const HandMoves::controling_t &init_controls, IN Point &hand_position)
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
              { it->last = (last_prevs[i]  +  last_steps[i]); }
              // --------------------------
              int   j = (i % 2) ? (i - 1) : (i + 1);
              auto op = (i % 2) ? std::prev (it) : std::next (it);
              // --------------------------
              Hand::frames_t  op_last;
              if ( last_steps[j] < 0 )
              { op_last = (last_prevs[j] >= -last_steps[j]) ? (last_prevs[j] + last_steps[j]) : 0U; }
              else
              { op_last = (last_prevs[j]  +  last_steps[j]); }
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
                                           IN const HandMoves::controling_t &init_controls, IN Point &hand_position)
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
  typedef std::function<bool (const Record &, const Point &)> func_t;
  Record*  GradientMin (adjacency_refs_t &range, const Point &aim, std::set<size_t> *visited=NULL, func_t *func=NULL)
  {
    Record *rmin = NULL;
    double  dr, dm;

    size_t h;
    for ( auto &rec : range )
    {
      if ( visited )
      { RecordHasher rh;
        h = rh (*rec);
      }

      dr = boost_distance (rec->hit, aim);
      if ( (!visited || visited->find (h) == visited->end ())
        && (!func || (*func) (*rec, aim))
        && (!rmin || dr < dm) )
      { 
        rmin = &(*rec);
        dm = dr;
      }
    }
    return  rmin;
  };

  bool  ClothestRecords (IN HandMoves::Store &store, IN const Point &aim,
                         OUT Record *rec0, OUT Record *rec1, OUT Record *rec2,
                         IN std::set<size_t> *visited=NULL)
  {
    if ( !rec0 )
    { return  false; }
    // ------------------------------------------------
    Point  min (aim.x - 0.1, aim.y - 0.1),
           max (aim.x + 0.1, aim.y + 0.1);
    // ------------------------------------------------
    adjacency_refs_t range;
    store.adjacencyRectPoints<adjacency_refs_t, ByP> (range, min, max);
    // ------------------------------------------------
    func_t  cmp_l = [](const Record &p, const Point &aim)
    { return  (p.hit.x < aim.x) && (p.hit.y < aim.y); };
    func_t  cmp_g = [](const Record &p, const Point &aim)
    { return  (p.hit.x > aim.x) && (p.hit.y > aim.y); };
    // ------------------------------------------------
    Record *pRec = GradientMin (range, aim, visited, NULL);
    if ( !pRec ) { return false; }

    RecordHasher rh;
    size_t h = rh (*pRec);
    visited->insert (h);
    // ===========
    *rec0 = *pRec;
    // ===========
    // ------------------------------------------------
    if ( rec1 && rec2 )
    {
      // ------------------------------------------------
      pRec = GradientMin (range, aim, visited, &cmp_l);
      if ( !pRec ) { return false; }
      // ===========
      *rec1 = *pRec;
      // ===========
      // ------------------------------------------------
      pRec = GradientMin (range, aim, visited, &cmp_g);
      if ( !pRec ) { return false; }
      // ===========
      *rec2 = *pRec;
      // ===========
      // ------------------------------------------------
    }
    // ------------------------------------------------
    return true;

    {
      // const Record &rec = store.ClothestPoint (aim);
      // -----------------------------------------------
      // std::pair<Record, Record> x_pair, y_pair;
      // std::pair<Record, Record> d_pair;
      // -----------------------------------------------
      // size_t  count = store.adjacencyByPBorders (rec.hit, side, x_pair, y_pair);
      // size_t  count = store.adjacencyByPBorders (rec.hit, d_pair);
      // if ( !count )
      // { return; }
    }
  }

  void  LearnMovements::gradientMethod (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim) //,
                                        //IN const HandMoves::controling_t &in_controls, IN Point &hand_pos)
  {
    size_t  gradient_complexity = 0U;
    // -----------------------------------------------
    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;
    // -----------------------------------------------
    Point hand_position; // = rec.hit;

    std::set<size_t>  visited;
    // double d = boost_distance ((**it).hit, aim);
    // -----------------------------------------------
    double distance = boost_distance (hand_pos_base, aim);
    double new_distance = 0.;
    while ( precision < distance )
    {
      Record rec0, rec1, rec2;
      if ( !ClothestRecords (store, aim, &rec0, &rec1, &rec2, &visited) )
      { return; }

      new_distance = boost_distance (rec0.hit, aim);
      if ( new_distance < distance )
        distance = new_distance;

      HandMoves::controling_t  init_controls = rec0.controls;
      // -----------------------------------------------
      double  l_distance = boost_distance (aim, rec1.hit);
      double  g_distance = boost_distance (aim, rec2.hit);

      double  d_d = (g_distance - l_distance);
      // -----------------------------------------------
      /* нужна !!данная точка!!! + 4 - в каждую сторону */
      const auto &less_controls = rec1.controls;
      const auto &grtr_controls = rec2.controls;
      // -----------------------------------------------
      HandMoves::controling_t  controls;
      // -----------------------------------------------
      auto  iter = controls.begin ();
      for ( auto j : hand.joints_ )
      {
        auto mo = muscleByJoint (j, true);
        auto mc = muscleByJoint (j, false);

        auto it_mo_i = boost::range::find (init_controls, mo);
        auto it_mo_l = boost::range::find (less_controls, mo);
        auto it_mo_g = boost::range::find (grtr_controls, mo);

        auto it_mc_i = boost::range::find (init_controls, mc);
        auto it_mc_l = boost::range::find (less_controls, mc);
        auto it_mc_g = boost::range::find (grtr_controls, mc);

        int last_mo_i = (it_mo_i != init_controls.end ()) ?  it_mo_i->last : 0U;
        int last_mo_l = (it_mo_l != less_controls.end ()) ? (it_mo_l->last - last_mo_i) : 0U;
        int last_mo_g = (it_mo_g != grtr_controls.end ()) ? (it_mo_g->last - last_mo_i) : 0U;
        
        int last_mc_i = (it_mc_i != init_controls.end ()) ?  it_mc_i->last : 0U;
        int last_mc_l = (it_mc_l != less_controls.end ()) ? (it_mc_l->last - last_mc_i) : 0U;
        int last_mc_g = (it_mc_g != grtr_controls.end ()) ? (it_mc_g->last - last_mc_i) : 0U;

        const int  int_normalizer = 400;

        int  direction_o = 0U;
        int  direction_c = 0U;

        if ( last_mo_l || last_mo_g )
        {
          double  d = d_d / (last_mo_l + last_mo_g);
          // direction_o = ((d_d * int_normalizer) / (last_mo_l + last_mo_g));
          direction_o = d * int_normalizer;

          if ( new_distance >= distance )
          { direction_o = (direction_o < 0) ? -1: 1; }
          
          // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          // BACKWARD MOVEMENT
          if ( direction_o == 0 && d < 0. )
          { direction_c = (direction_c) ? (direction_c + 1) : 50; }
        }

        if ( last_mc_l || last_mc_g )
        {
          double d = d_d / (last_mc_l + last_mc_g);
          // direction_c = ((d_d * int_normalizer) / (last_mc_l + last_mc_g));
          direction_c = d * int_normalizer;

          if ( new_distance >= distance )
          { direction_c = (direction_c < 0) ? -1 : 1; }

          // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          // BACKWARD MOVEMENT
          if ( direction_c == 0 && d < 0. )
          { direction_o = (direction_o) ? (direction_o + 1) : 50; }
        }

        Hand::frames_t last_o = 0U;
        Hand::frames_t last_c = 0U;
        if ( last_mo_i > direction_o )
        { last_o += (last_mo_i - direction_o); }
        else if ( last_mc_i > direction_o )
        { last_c += (last_mo_i - direction_o); }

        if ( last_mc_i > direction_c )
        { last_c += (last_mc_i - direction_c); }
        else if ( last_mo_i > direction_c )
        { last_o += (last_mc_i - direction_c); }

        Hand::frames_t start_o = (last_o > last_c) ? 0U : (last_c + 1U);
        Hand::frames_t start_c = (last_o < last_c) ? 0U : (last_o + 1U);

        if ( last_o )
          controls.insert (iter, Hand::Control (mo, start_o, last_o));
        if ( last_c )
          controls.insert (iter, Hand::Control (mc, start_c, last_c));
      }
      // -----------------------------------------------
      hand_act (store, hand, controls, aim, hand_position, hand_pos_base);
      ++gradient_complexity;

      double d = boost_distance (hand_position, aim);
      if ( d > side ) //== new_distance )
      {
        visited.clear ();

        Record rec0;
        ClothestRecords (store, aim, &rec0, NULL, NULL, &visited);
        if ( rec0.controls.size () )
        {
          Point hand_position = rec0.hit;
          rundownMethod (store, hand, aim, rec0.controls, hand_position);
          d = boost_distance (hand_position, aim);
        }
        
        if ( d == new_distance )
        { break; }
      }

      if ( new_distance > 0.1 )
      {
        visited.clear ();
        // continue;
      }

      if ( new_distance > distance && d > 0.2 ) // new_distance )
      { break; }
      
      new_distance = d;
      if ( distance > new_distance )
      { distance = new_distance; }
      // -----------------------------------------------
#ifdef _DEBUG_PRINT
      tcout << _T ("prec: ") << best_distance << std::endl;
#endif // _DEBUG_PRINT
    } // end while
    // -----------------------------------------------
    complexity += gradient_complexity;
#ifdef _DEBUG_PRINT_RES
    tcout << _T ("precision: ") << distance << std::endl;
    tcout << _T ("gradient complexity: ")
          <<      gradient_complexity
          <<   std::endl << std::endl;
#endif // _DEBUG_PRINT_RES
  }
  //------------------------------------------------------------------------------
  bool  next_last_steps (std::vector<int> &last_steps)
  {
    for ( auto it = last_steps.begin (); it != last_steps.end (); ++it )
    {
      if ( *it > 0 )
      {
        *it = -*it;
        return true;
      }
      else if ( *it < 0 )
      { *it = 0;
        if ( std::next (it) == last_steps.end () )
        { return false; }
        else
        { *std::next (it) = 1;
          return true;
        }
      }
    }
    last_steps.front () = 1;
    return true;
  }
 
  void  fillControlsOrder (IN  const std::vector<Hand::JointsEnum> hand_joints,
                           IN  const HandMoves::controling_t &init_controls,
                           OUT       HandMoves::controling_t &controls)
  {
    auto iter = controls.begin ();
    for ( auto j : hand_joints )
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
    
  void  fillControls      (IN const std::vector<Hand::JointsEnum> hand_joints,
                           IN OUT HandMoves::controling_t &controls)
  {
    for ( auto j : hand_joints )
    {
      auto mo = muscleByJoint (j, true);
      auto mc = muscleByJoint (j, false);

      auto it_o = std::find (controls.begin (), controls.end (), mo);
      auto it_c = std::find (controls.begin (), controls.end (), mc);

      if ( it_o == controls.end () && it_c != controls.end () )
      { controls.push_back (Hand::Control (mo, it_c->last + 1, 1)); }
      else
      { controls.push_back (Hand::Control (mo, 0, 1)); }
      
      if ( it_c == controls.end () ) // && it_o != controls.end () )
      { controls.push_back (Hand::Control (mc, it_o->last + 1, 1)); }
    }
  }

  bool  next_last_steps (HandMoves::controling_t &controls, size_t &controls_curr,
                         double velosity, double velosity_prev)
  {
    if ( controls_curr >= controls.size () )
    { return true; }

    auto it = controls.begin ();
    if ( controls_curr == 0U )
    { it->last += velosity; }
    else
    {
      std::advance (it, controls_curr / 2);

      if ( !(controls_curr % 2) )
      { it->last -= (velosity_prev + velosity); }
      else
      {          (it)->last += velosity_prev;
        std::next(it)->last += velosity;
      }
    } // end else

    velosity_prev = velosity;
    ++controls_curr;
    return false;
  }

  size_t  rundown (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim, IN double precision,
                   IN OUT HandMoves::controling_t &controls, IN OUT Point &hand_position)
  {
    size_t  rundown_complexity = 0U;
    // -----------------------------------------------
    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;
    Point hand_pos      = hand_position;
    // -----------------------------------------------
    fillControls (hand.joints_, controls);
    // -----------------------------------------------
    double  distance = boost_distance (hand_position, aim);
    double  velosity = distance / precision;
    double  velosity_prev = 0U;
    // -----------------------------------------------
    size_t  controls_curr = 0U;
    // -----------------------------------------------
    while ( !next_last_steps (controls, controls_curr,
                              velosity, velosity_prev) )
    {
      hand_act (store, hand, controls, aim,
                hand_pos, hand_pos_base);
      ++rundown_complexity;
      // -----------------------------------------------
      double next_distance = boost_distance (hand_pos, aim);
      // -----------------------------------------------
      while ( next_distance < distance )
      {
        distance = next_distance;
        hand_position = hand_pos;
        // -----------------------------------------------
        auto it = controls.begin ();
        std::advance (it, controls_curr / 2);

        if ( !(controls_curr % 2) )
        { it->last -= velosity; }
        else
        { it->last += velosity; }
        velosity_prev = velosity;
        // -----------------------------------------------
        hand_act (store, hand, controls, aim,
                  hand_pos, hand_pos_base);
        ++rundown_complexity;
        // -----------------------------------------------
        next_distance = boost_distance (hand_pos, aim);
        // -----------------------------------------------
      } // while
    } // while
    // -----------------------------------------------
#ifdef _DEBUG_PRINT_RES
    tcout << _T ("prec: ") << distance << std::endl;
    tcout << _T ("rundown complexity: ")
          <<      rundown_complexity
          <<  std::endl << std::endl;
#endif // _DEBUG_PRINT_RES

    // -----------------------------------------------
    hand_position = hand_pos;
    return rundown_complexity;
  }


  size_t  weightedMean (IN  HandMoves::Store &store, IN Hand &hand,
                        IN  const Point &aim, IN double side, IN double precision,
                        OUT HandMoves::controling_t &controls)
  {
    size_t wmean_complexity = 0U;
    // -----------------------------------------------    
    hand.SET_DEFAULT;
    Point  hand_pos_base = hand.position;
    Point  hand_position;
    // -----------------------------------------------
    double  distance = boost_distance (hand_pos_base, aim);


    // -----------------------------------------------
    // Point  min (aim.x - side, aim.y - side),
    //        max (aim.x + side, aim.y + side);
    // -----------------------------------------------
    int isfsdf = 0;
    while ( isfsdf < 4 )
    {
      HandMoves::adjacency_refs_t  range;
      // size_t  count = store.adjacencyPoints (range, aim, side);
      // size_t  count = store.adjacencyRectPoints<adjacency_refs_t, ByP> (range, min, max);
      // size_t  count = store.adjacencyByPBorders (range, aim, side);

      // std::pair<Record, Record> x_min, y_min;
      // size_t  count = store.adjacencyByPBorders (aim, x_min, y_min);
      size_t count = store.adjacencyByPBorders (range, aim);
      if ( count == 0U )
      { return 0U; }

      // -----------------------------------------------
      double all_distance = 0.;
      for ( auto &pRec : range )
      { all_distance += boost_distance (aim, pRec->hit); }
      // -----------------------------------------------
      for ( auto m : hand.muscles_ )
      {
        Hand::Control  control (m, 0U, 0U);
        // -----------------------------------------------
        Hand::frames_t  n_control_summands = 0U;
        for ( auto &pRec : range )
          for ( auto &c : pRec->controls )
          {
            if ( c.muscle == m )
            {
              /* взвешенное НЕСМЕЩЁННОЕ cреднее арифметическое */
              double weight = boost_distance (aim, pRec->hit) / all_distance;
              control.start += weight * c.start;
              control.last += weight * c.last;
            } // end if
          } // end for-for
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
      // -----------------------------------------------

      hand_act (store, hand, controls, aim,
                hand_position, hand_pos_base);
      ++wmean_complexity;
      // -----------------------------------------------
      double next_distance = boost_distance (hand_position, aim);
      if ( next_distance < distance )
      { distance = next_distance; }
    }
    // -----------------------------------------------
#ifdef _DEBUG_PRINT_RES
    tcout << _T ("prec: ") << distance << std::endl;
    tcout << _T ("wmean complexity: ")
          <<      wmean_complexity
          << std::endl << std::endl;
#endif // _DEBUG_PRINT_RES

    return wmean_complexity;
  }

  void  LearnMovements::gradientMethod_admixture (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim)
  {
    controling_t  controls;
    weightedMean (store, hand, aim, 0.1, 0.004, controls);

    size_t  gradient_complexity = 0U;
    // -----------------------------------------------
    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;
    // -----------------------------------------------
    Point hand_position; // = rec.hit;

    std::set<size_t>  visited;
    // double d = boost_distance ((**it).hit, aim);
    // -----------------------------------------------
    double distance = boost_distance (hand_pos_base, aim);
    double new_distance = 0.;
    while ( precision < distance )
    {
      Record rec0, rec1, rec2;
      if ( !ClothestRecords (store, aim, &rec0, &rec1, &rec2, &visited) )
      { return; }

      new_distance = boost_distance (rec0.hit, aim);
      if ( new_distance < distance )
        distance = new_distance;

      HandMoves::controling_t  init_controls = rec0.controls;
      // -----------------------------------------------
      double  l_distance = boost_distance (aim, rec1.hit);
      double  g_distance = boost_distance (aim, rec2.hit);

      double  d_d = (g_distance - l_distance);
      // -----------------------------------------------
      /* нужна !!данная точка!!! + 4 - в каждую сторону */
      const auto &less_controls = rec1.controls;
      const auto &grtr_controls = rec2.controls;
      // -----------------------------------------------
      HandMoves::controling_t  controls;
      // -----------------------------------------------
      auto  iter = controls.begin ();
      for ( auto j : hand.joints_ )
      {
        auto mo = muscleByJoint (j, true);
        auto mc = muscleByJoint (j, false);

        auto it_mo_i = boost::range::find (init_controls, mo);
        auto it_mo_l = boost::range::find (less_controls, mo);
        auto it_mo_g = boost::range::find (grtr_controls, mo);

        auto it_mc_i = boost::range::find (init_controls, mc);
        auto it_mc_l = boost::range::find (less_controls, mc);
        auto it_mc_g = boost::range::find (grtr_controls, mc);

        int last_mo_i = (it_mo_i != init_controls.end ()) ? it_mo_i->last : 0U;
        int last_mo_l = (it_mo_l != less_controls.end ()) ? (it_mo_l->last - last_mo_i) : 0U;
        int last_mo_g = (it_mo_g != grtr_controls.end ()) ? (it_mo_g->last - last_mo_i) : 0U;

        int last_mc_i = (it_mc_i != init_controls.end ()) ? it_mc_i->last : 0U;
        int last_mc_l = (it_mc_l != less_controls.end ()) ? (it_mc_l->last - last_mc_i) : 0U;
        int last_mc_g = (it_mc_g != grtr_controls.end ()) ? (it_mc_g->last - last_mc_i) : 0U;

        const int  int_normalizer = 400;

        int  direction_o = 0U;
        int  direction_c = 0U;

        if ( last_mo_l || last_mo_g )
        {
          double  d = d_d / (last_mo_l + last_mo_g);
          // direction_o = ((d_d * int_normalizer) / (last_mo_l + last_mo_g));
          direction_o = d * int_normalizer;

          if ( new_distance >= distance )
          { direction_o = (direction_o < 0) ? -1 : 1; }

          // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          // BACKWARD MOVEMENT
          if ( direction_o == 0 && d < 0. )
          { direction_c = (direction_c) ? (direction_c + 1) : 50; }
        }

        if ( last_mc_l || last_mc_g )
        {
          double d = d_d / (last_mc_l + last_mc_g);
          // direction_c = ((d_d * int_normalizer) / (last_mc_l + last_mc_g));
          direction_c = d * int_normalizer;

          if ( new_distance >= distance )
          { direction_c = (direction_c < 0) ? -1 : 1; }

          // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          // BACKWARD MOVEMENT
          if ( direction_c == 0 && d < 0. )
          { direction_o = (direction_o) ? (direction_o + 1) : 50; }
        }

        Hand::frames_t last_o = 0U;
        Hand::frames_t last_c = 0U;
        if ( last_mo_i > direction_o )
        { last_o += (last_mo_i - direction_o); }
        else if ( last_mc_i > direction_o )
        { last_c += (last_mo_i - direction_o); }

        if ( last_mc_i > direction_c )
        { last_c += (last_mc_i - direction_c); }
        else if ( last_mo_i > direction_c )
        { last_o += (last_mc_i - direction_c); }

        Hand::frames_t start_o = (last_o > last_c) ? 0U : (last_c + 1U);
        Hand::frames_t start_c = (last_o < last_c) ? 0U : (last_o + 1U);

        if ( last_o )
          controls.insert (iter, Hand::Control (mo, start_o, last_o));
        if ( last_c )
          controls.insert (iter, Hand::Control (mc, start_c, last_c));
      }
      // -----------------------------------------------
      hand_act (store, hand, controls, aim, hand_position, hand_pos_base);
      ++gradient_complexity;

      double d = boost_distance (hand_position, aim);
      if ( d > side ) //== new_distance )
      {
        visited.clear ();

        Record rec0;
        ClothestRecords (store, aim, &rec0, NULL, NULL, &visited);
        if ( rec0.controls.size () )
        {
          Point hand_position = rec0.hit;
          rundownMethod (store, hand, aim, rec0.controls, hand_position);
          d = boost_distance (hand_position, aim);
        }

        if ( d == new_distance )
        { break; }
      }

      if ( new_distance > 0.1 )
      {
        visited.clear ();
        // continue;
      }

      if ( new_distance > distance && d > 0.2 ) // new_distance )
      { break; }

      new_distance = d;
      if ( distance > new_distance )
      { distance = new_distance; }
      // -----------------------------------------------
#ifdef _DEBUG_PRINT
      tcout << _T ("prec: ") << best_distance << std::endl;
#endif // _DEBUG_PRINT
    } // end while
      // -----------------------------------------------
    complexity += gradient_complexity;
#ifdef _DEBUG_PRINT_RES
    tcout << _T ("precision: ") << distance << std::endl;
    tcout << _T ("gradient complexity: ")
      << gradient_complexity
      << std::endl << std::endl;
#endif // _DEBUG_PRINT_RES
  }
};
