#include "Position.h"



namespace Positions
{
  using namespace std;
  using namespace HandMoves;

  void  testCover (HandMoves::Store &store, Hand &hand, HandMoves::trajectories_t &trajectories)
  {
    hand.reset ();
    hand.SET_DEFAULT;

    Point hand_base = hand.position;

    *
    *
    */

    for ( auto muscle_i : hand.muscles_ /* | boost::adaptors::sliced (3, 4) */ )
    {
      // if ( !(muscle_i & muscles) && muscleValidAtOnce(muscle_i | muscles) )
      {
        HandMoves::trajectory_t  trajectory;
        for ( Hand::frames_t last_i : boost::irange<Hand::frames_t> (1U, hand.maxMuscleLast (muscle_i) / 2, 10) )
        {
          Hand::Control control_i (muscle_i, 0U, last_i);
          // hand.move ({ control_i });

          for ( auto muscle_j : hand.muscles_ )
            if ( (muscle_i != muscle_j) && muscleValidAtOnce (muscle_i | muscle_j) )
            {
              for ( Hand::frames_t start_j : boost::irange<Hand::frames_t> (1U, last_i, 10) ) // ?? + inertial_lasts
              {
                for ( Hand::frames_t last_j : boost::irange<Hand::frames_t> (1U, hand.maxMuscleLast (muscle_j) / 2, 10) )
                {
                  Hand::Control control_j (muscle_j, 0U /* start_j */, last_j);
                  hand.move ({ control_i, control_j }, &trajectory);

                  // auto& index = store.get<Record::ByC> ();
                  // 
                  // auto& index = store.get<Record::ByP> ();
                  // index.equal_range (hand.position);

                  Point hand_pos = hand.position;
                  HandMoves::Record  rec (hand_pos, hand_base, hand_pos,
                  { control_i, control_j }, trajectory);

                  store.insert (rec);
                  // trajectories.push_back (make_shared<trajectory_t> (trajectory));
                }
              }
            }
        }
      }
      // for ( auto muscle_j : hand.muscles_ )
      // {
      //   if ( !(muscle_j & muscles) && muscleValidAtOnce (muscle_i | muscles) )
      //   {
      //     for ( Hand::frames_t last_i : boost::irange<Hand::frames_t> (1U, 126U, 10) ) //hand.maxMuscleLast (muscle_i)) )
      //     {
      //       hand.move (muscle_i | muscle_j, last_i, trajectory);
      //       store.insert (Record (hand.position, hand_base, hand.position,
      //                    { muscle_i }, { 0 }, { last_i, 126U - last_i }, 1U,
      //                    trajectory)
      //                    );
      //     }
      //   } // end if
      // } // end for
      // 
      //   // trajectories.push_back (make_shared<trajectory_t> (trajectory));
      //    */
      //   testCover (store, hand, muscles | muscle_i, trajectories);
      // 
      // } // end for
      //} // end if
    } // end for
  }
  //------------------------------------------------------------------------------
  struct counts_t
  {
    int count = 0;
    int count_TP = 0;
    int count_FP = 0;
    int count_TN = 0;
    int count_FN = 0;

    void incr (bool model, bool real)
    {
      ++count;
      if ( model & real )
        ++count_TP;
      else if ( !model & !real )
        ++count_TN;
      else if ( model & !real )
        ++count_FP;
      else if ( !model & real )
        ++count_FN;
    }
    void print ()
    {
      std::cout << "count = " << count << std::endl;
      // std::cout << "TP = " << count_TP << std::endl;
      // std::cout << "TN = " << count_TN << std::endl;
      // std::cout << "FP = " << count_FP << std::endl;
      // std::cout << "FN = " << count_FN << std::endl;

      std::cout << "\t< T >\t < F >" << std::endl;
      std::cout << "< P >\t" << count_TP << '\t' << count_FP << std::endl;
      std::cout << "< N >\t" << count_TN << '\t' << count_FN << std::endl;
    }
  };
  //------------------------------------------------------------------------------
  void  draftDistance (IN  Point  &hand_pos_prev,
                       IN  Point  &hand_pos,
                       OUT Hand::frames_t  &lasts_step,
                       IN  const Hand::frames_t  &last,
                       IN  const Hand::frames_t   lasts_max,
                       IN  const Hand::frames_t   lasts_incr_value,
                       IN double distance)
  {
    if ( boost_distance (hand_pos_prev, hand_pos) > distance )
    {
      if ( lasts_step > 2 * lasts_incr_value )
        lasts_step -= lasts_incr_value;
      // else if ( lasts_step > lasts_incr_value )
      //   lasts_step -= lasts_incr_value * 0.5;
      // else if ( lasts_step > 2 )
      //   --lasts_step;
      else if ( lasts_step == 1 && lasts_incr_value == 1U )
      {

      }
    }
    else if ( boost_distance (hand_pos_prev, hand_pos) < distance )
    {
      if ( (last + lasts_incr_value) < lasts_max )
        lasts_step += lasts_incr_value;
      // else if ( (last + 0.5 * lasts_incr_value) < last_max )
      //   lasts_step += 0.5 * lasts_incr_value;
      // else
      //   lasts_step = lasts_max - last;
    }

  }

  void  borders_insert (borders_t &borders_, const HandMoves::Record &rec)
  {
    for ( auto &ctrl : rec.controls () )
    {
      for ( auto m : muscles )
        if ( m & ctrl.muscle )
        {
          auto it = borders_.find (m);
          if ( it != borders_.end () )
          {
            if ( ctrl.last < it->second.first )
              it->second.first = ctrl.last;
            if ( ctrl.last > it->second.second )
              it->second.second = ctrl.last;
          }
          else
          { borders_[ctrl.muscle] = std::make_pair (ctrl.last, ctrl.last); }
        }
    } // end for
  }

  void  borders (borders_t &borders_, HandMoves::Store &store, RecTarget &target, double draft_distance)
  {
    for ( auto &rec : store )
    {
      if ( target.contain (rec.hit) )
      { borders_insert (borders_, rec); } // end if
    } // end for

    HandMoves::adjacency_refs_t range;
    store.adjacencyPoints (range, (target.min) (), draft_distance);
    for ( auto p_rec : range )
    {
      borders_insert (borders_, *p_rec);
    }

    range.clear ();
    store.adjacencyPoints (range, Point ((target.min) ().x, (target.max) ().y), draft_distance);
    for ( auto p_rec : range )
    {
      borders_insert (borders_, *p_rec);
    }

    range.clear ();
    store.adjacencyPoints (range, Point ((target.max) ().x, (target.min) ().y), draft_distance);
    for ( auto p_rec : range )
    {
      borders_insert (borders_, *p_rec);
    }

    range.clear ();
    store.adjacencyPoints (range, (target.max) (), draft_distance);
    for ( auto p_rec : range )
    {
      borders_insert (borders_, *p_rec);
    }

  }

  void  checking (Hand &hand, RecTarget &target,
                  HandMoves::controling_t &controls,
                  const Point &end, counts_t &stats)
  {
    //-------------------------------------------------------------
    HandMoves::trajectory_t  trajectory;
    hand.SET_DEFAULT;
    hand.move (controls.begin (), controls.end (), &trajectory);
    //-------------------------------------------------------------
    bool model = target.contain (end);
    bool real  = target.contain (hand.position);
    stats.incr (model, real);
  }
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
  class  ITERATE
  {
    HandMoves::Store &store;
    Hand &hand;
    RecTarget &target;

    std::vector<Hand::MusclesEnum>  ms;
    std::vector<Hand::frames_t>     ls;
    std::vector<Hand::frames_t>     ss;

    // std::vector<Point>           prevs;
    HandMoves::controling_t  controls;

    std::vector<std::list<Hand::Control>>    arr_controlings;

    borders_t borders_;
    DirectionPredictor  &pd;

    double step_distance;
    Hand::frames_t   lasts_step_init_value = 1U;
    Hand::frames_t   lasts_step_incr_value = 1U;

    Point hand_base;

    size_t complexity = 0U;
    counts_t stats;

  public:
    ITERATE (HandMoves::Store &store, Hand &hand, RecTarget &target,
             borders_t &borders, DirectionPredictor &pd,
             double step_distance,
             Hand::frames_t lasts_step_init_value,
             Hand::frames_t lasts_step_incr_value) :

      store (store), hand (hand), target (target),
      borders_ (borders), pd (pd),
      ms (hand.joints_.size ()), ls (hand.joints_.size ()), ss (hand.joints_.size ()),
      // controls (hand.joints_.size ()),
      arr_controlings (hand.joints_.size ()),
      step_distance (step_distance),
      lasts_step_init_value (lasts_step_init_value),
      lasts_step_incr_value (lasts_step_incr_value)
    {
      hand.SET_DEFAULT;
      hand_base = hand.position;
    }

    void  run ()
    {
      try
      {
        Point pt;
        runNestedFor_Muscle (0U, pt);
      }
      catch ( boost::thread_interrupted& )
      { return; } // end catch

      stats.print ();
      // ----------------------------------------------------
      std::wcout << _T ("\nStage 2 Complexity ") << complexity << std::endl;
      // ----------------------------------------------------
    }

  private:
    bool  runNestedFor_Muscle (int joint_index, Point &hand_pos_high)
    {
      bool  target_contain = true;
      Hand::frames_t start_i = 0U;
      Hand::frames_t lasts_step = lasts_step_init_value;

      Point hand_pos = hand_base,
        hand_pos_prev = hand_base;

      controls.push_back (Hand::Control ());
      Hand::Control &control_i = controls.back ();

      for ( auto muscle_i : { muscleByJoint (hand.joints_[joint_index], true),
                              muscleByJoint (hand.joints_[joint_index], false) } )
      {
        // ms[joint_index] = muscle_i;
        control_i.muscle = muscle_i;

        Hand::frames_t lasts_i_max = hand.maxMuscleLast (muscle_i);
        for ( Hand::frames_t last_i  = borders_[muscle_i].first;  (last_i > 0U) && (target_contain
                          || last_i <= borders_[muscle_i].second - (start_i));
                             last_i += lasts_step )
        {
          // ls[joint_index] = last_i;
          control_i.last = last_i;

          if ( 0 <= joint_index && (joint_index + 1) < hand.joints_.size () )
            target_contain = runNestedFor_Muscle (joint_index + 1, hand_pos);
          else
            target_contain = inserting (hand_pos);

          if ( last_i == borders_[muscle_i].first )
            hand_pos_high = hand_pos;

          if ( boost_distance (hand_pos_prev, hand_pos) > step_distance )
          {
            if ( lasts_step > 2 * lasts_step_incr_value )
              lasts_step -= lasts_step_incr_value;
            // else if ( lasts_step == 1 && lasts_step_incr_value == 1U )
            // {
            //   arr_controlings[joint_index].push_back (Hand::Control (muscle_i, start_i, last_i));
            //   start_i = last_i;
            //   // ss[joint_index] = start_i;
            //   control_i.start = start_i;
            //   last_i = 1;
            // }
          }
          else if ( boost_distance (hand_pos_prev, hand_pos) < step_distance )
          {
            if ( (last_i + lasts_step_incr_value) < lasts_i_max )
              lasts_step += lasts_step_incr_value;
          }
          hand_pos_prev = hand_pos;
        } // end for (last)

        arr_controlings[joint_index].clear ();
      } // end for (muscle)
      return  target_contain;
    }

    bool  inserting (Point &hand_pos)
    {
      bool  target_contain = true;
      Point end = hand_base;

      // for ( auto index : boost::irange (size_t{ 0U }, hand.joints_.size ()) )
      //   pd.predict (ms[index], ls[index], end);
      //----------------------------------------------
      // std::list<Hand::Control> controls;
      // for ( auto ii = 0U; ii < ms.size (); ++ii )
      //   controls.push_back (Hand::Control (ms[ii], ss[ii], ls[ii]));
      //----------------------------------------------

      bool  braking = boost::algorithm::any_of (arr_controlings,
                                                [](const controling_t &item)
                                                { return  item.size (); });
      controling_t  new_controling;
      if ( braking )
      {
        for ( const Hand::Control &c : controls )
          new_controling.push_back (c);
        // controling_t stop_controls;

        for ( auto &lst : arr_controlings )
          // controls.splice (controls.end (), lst);
          for ( auto &c : lst )
            // stop_controls.push_back (c);
            new_controling.push_back (c);

        // stop_controls.sort ();
        // controls.splice (controls.begin (), stop_controls);
        new_controling.sort ();
      }
      controling_t  &controling = (braking) ? new_controling : controls;
      //----------------------------------------------
      for ( auto &c : controls )
        pd.predict (c, end);
      //----------------------------------------------
      checking (hand, target, controling, end, stats);
      //----------------------------------------------
      if ( target.contain (end) )
      {
        //----------------------------------------------
        HandMoves::trajectory_t  trajectory;
        hand.SET_DEFAULT;
        hand.move (controling.begin (), controling.end (), &trajectory);
        hand_pos = hand.position;
        //----------------------------------------------
        target_contain = target.contain (hand_pos);
        //----------------------------------------------
        HandMoves::Record  rec (hand_pos, hand_base, hand_pos,
                                controling, trajectory);
        store.insert (rec);
        //----------------------------------------------
        ++complexity;
        boost::this_thread::interruption_point ();
        //----------------------------------------------
      } /* end if */
      else
      { target_contain = false; }
      //----------------------------------------------
      // while ( controls.size () > hand.joints_.size () )
      //   controls.pop_front ();
      //----------------------------------------------
      return  true; // target_contain;
    }
  };
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
  void  LearnMovements::STAGE2 (HandMoves::Store &store, Hand &hand, RecTarget &target)
  {
    borders (borders_, store, target, draft_distance);
    DirectionPredictor  pd (hand);

    ITERATE iter (store, hand, target, borders_, pd,
                  detail_distance, 1, 15);

    iter.run ();
  }
  //------------------------------------------------------------------------------
  void  LearnMovements::testStage1 (HandMoves::Store &store, Hand &hand, RecTarget &target)
  {
    size_t complexity = 0U;

    hand.reset ();
    hand.SET_DEFAULT;

    Point  hand_pos_base = hand.position;

    Point  hand_pos_prev_i = hand_pos_base;
    Point  hand_pos_prev_j = hand_pos_base;

    for ( auto muscle_i : { muscleByJoint (hand.joints_[0], false), muscleByJoint (hand.joints_[0], true) } ) // hand.muscles_
      for ( auto muscle_j : { muscleByJoint (hand.joints_[1], false), muscleByJoint (hand.joints_[1], true) } ) // hand.muscles_
                                                                                                                //for ( auto muscle_i : hand.muscles_ )
                                                                                                                //  for ( auto muscle_j : hand.muscles_ )
        if ( (muscle_i != muscle_j) && muscleValidAtOnce (muscle_i | muscle_j) )
        {
          Point  hand_pos_i;
          Hand::frames_t  lasts_step = lasts_init_value1;

          auto last_i_max = hand.maxMuscleLast (muscle_i),
            last_j_max = hand.maxMuscleLast (muscle_j);
          for ( Hand::frames_t last_i = lasts_init_value1; last_i < last_i_max; last_i += lasts_step )
          {
            for ( Hand::frames_t last_j = lasts_init_value1; last_j < last_j_max; last_j += lasts_step )
            {
              Hand::Control  control_i (muscle_i, 0U, last_i);
              // {
              //   trajectory_t  trajectory;
              //   hand.SET_DEFAULT;
              //   hand.move ({ control_i }, &trajectory);
              //   store.insert (Record (hand.position, hand_base, hand.position,
              //                          { muscle_i }, { 0U }, { last_i }, 1U,
              //                            trajectory));
              // }
              Hand::Control  control_j (muscle_j, 0U /* start_j */, last_j);

              HandMoves::trajectory_t  trajectory;
              hand.SET_DEFAULT;
              hand.move ({ control_i, control_j }, &trajectory);

              Point hand_pos_j = hand.position;
              if ( last_j == lasts_init_value1 )
                hand_pos_i = hand_pos_j;

              HandMoves::Record  rec (hand_pos_j, hand_pos_base, hand_pos_j,
              { control_i, control_j }, trajectory);
              store.insert (rec);

              draftDistance (hand_pos_prev_j, hand_pos_j, lasts_step, last_j, last_j_max, lasts_incr_value1, draft_distance);
              hand_pos_prev_j = hand_pos_j;

              ++complexity;

              try
              { boost::this_thread::interruption_point (); }
              catch ( boost::thread_interrupted& )
              { return; } // end catch

            } // end for

            draftDistance (hand_pos_prev_i, hand_pos_i, lasts_step, last_i, last_i_max, lasts_incr_value1, draft_distance);
            hand_pos_prev_i = hand_pos_i;

          } // end for 
        }
    // ----------------------------------------------------
    std::wcout << _T ("\nStage 1 Complexity ") << complexity << std::endl;
    // ----------------------------------------------------
  }
  void  LearnMovements::testStage2 (HandMoves::Store &store, Hand &hand, RecTarget &target)
  {
    /*         ~ - - - - *
    *       /
    *      /  +----------------+
    *    x \  |                |
    *   x   \ |                |
    * x       -                |
    *         | \              |
    *         |  *             |
    *         |                |
    *         +----------------+
    */
    size_t complexity = 0U;

    borders (borders_, store, target, draft_distance);
    DirectionPredictor  pd (hand);

    std::wcout << _T ("borders: ") << borders_.size () << std::endl;

    hand.reset ();
    hand.SET_DEFAULT;

    Point  hand_pos_base = hand.position;

    Point  hand_pos_prev_i = hand_pos_base;
    Point  hand_pos_prev_j = hand_pos_base;

    counts_t stats;

    draft_distance = 0.1;
    Hand::frames_t  last_step = 10U;

    for ( auto muscle_i : { muscleByJoint (hand.joints_[0], false), muscleByJoint (hand.joints_[0], true) } )
      for ( auto muscle_j : { muscleByJoint (hand.joints_[1], false), muscleByJoint (hand.joints_[1], true) } )
        if ( (muscle_i != muscle_j) && muscleValidAtOnce (muscle_i | muscle_j) )
        {
          Point  hand_pos_i;
          Hand::frames_t  last_step = lasts_incr_value2;

          auto  last_i_max = hand.maxMuscleLast (muscle_i),
            last_j_max = hand.maxMuscleLast (muscle_j);

          bool target_contain = true;

          // auto last_i_max = hand.maxMuscleLast (muscle_i);
          for ( Hand::frames_t last_i = borders_[muscle_i].first; target_contain || last_i <= borders_[muscle_i].second; last_i += last_step )
          {
            for ( Hand::frames_t last_j = borders_[muscle_j].first /*- 50*/; target_contain || last_j <= borders_[muscle_j].second /*+ 50*/; last_j += last_step )
            {
              Hand::Control  control_i (muscle_i, 0U, last_i);
              Hand::Control  control_j (muscle_j, 0U /* start_j */, last_j);

              Point end = hand_pos_base;

              pd.predict (control_i, end);
              pd.predict (control_j, end);

              checking (hand, target, controling_t{ control_i, control_j }, end, stats);

              if ( target.contain (end) )
              {
                HandMoves::trajectory_t  trajectory;
                hand.SET_DEFAULT;
                hand.move ({ control_i, control_j }, &trajectory);

                Point  hand_pos_j = hand.position;
                if ( last_j == borders_[muscle_j].first )
                  hand_pos_i = hand_pos_j;
                //----------------------------------------------
                target_contain = target.contain (hand_pos_j);
                // std::wcout << tstring(hand_pos_j) << std::endl;
                //----------------------------------------------
                HandMoves::Record  rec (hand_pos_j, hand_pos_base, hand_pos_j,
                { control_i, control_j }, trajectory);
                store.insert (rec);

                draftDistance (hand_pos_prev_j, hand_pos_j, last_step, last_j, last_j_max, lasts_incr_value2, detail_distance);
                hand_pos_prev_j = hand_pos_j;

                ++complexity;

                try
                { boost::this_thread::interruption_point (); }
                catch ( boost::thread_interrupted& )
                { return; } // end catch

              }
              else
              { target_contain = false; }
            } // end for

            draftDistance (hand_pos_prev_i, hand_pos_i, last_step, last_i, last_i_max, lasts_incr_value2, detail_distance);
            hand_pos_prev_i = hand_pos_i;
          } // end for
        } // end if

    stats.print ();
    // ----------------------------------------------------
    std::wcout << _T ("\nStage 2 Complexity ") << complexity << std::endl;
    // ----------------------------------------------------
    hand.SET_DEFAULT;
  }
  void  LearnMovements::testStage3 (HandMoves::Store &store, Hand &hand, RecTarget &target, std::list<Point> &uncovered)
  {
    // std::list<Point> uncovered;
    // store.uncoveredTargetPoints (target, uncovered);
    for ( auto &pt : target.coords () )
    {
      std::list<std::shared_ptr<HandMoves::Record>> range;
      store.adjacencyPoints (range, pt, 2. * target.thickness ());

      if ( range.empty () )
      {
        /* UNCOVERED */
        uncovered.push_back (pt);
      }
      else
      {
        double precision = target.precision ();
        if ( boost::algorithm::none_of (range, [&pt, precision](const std::shared_ptr<HandMoves::Record> &rec) { return  (boost_distance (pt, rec->hit) <= precision); }) )
        {
          /* UNCOVERED */
          uncovered.push_back (pt);

        } // end if

      } // end else
    } // end for
  }
  //------------------------------------------------------------------------------
  void  LearnMovements::testCoverTarget (HandMoves::Store &store, Hand &hand, RecTarget &target)
  {
    hand.reset ();
    hand.SET_DEFAULT;

    Point hand_base = hand.position;
    Point hand_prev = hand.position;

    *
    */
    try
    {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
      std::wcout << _T ("hand.muscles_.size = ") << (int) hand.muscles_.size () << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
      for ( auto muscle_i : hand.muscles_ )
      {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
        std::wcout << muscle_i << _T ("  ") << (int) hand.maxMuscleLast (muscle_i) << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
        for ( auto muscle_j : hand.muscles_ )
        {
          if ( (muscle_i != muscle_j) && muscleValidAtOnce (muscle_i | muscle_j) )
          {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
            std::wcout << muscle_j << _T ("  ") << (int) hand.maxMuscleLast (muscle_j) << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF

            // for ( Hand::frames_t last_i : boost::irange<Hand::frames_t> (5U, hand.maxMuscleLast (muscle_i), 2) )
            for ( Hand::frames_t last_i : boost::irange<Hand::frames_t> (5U, hand.maxMuscleLast (muscle_i)) )
            {
              Hand::Control control_i (muscle_i, 0U, last_i);
              // hand.move ({ control_i });

              // for ( Hand::frames_t last_j : boost::irange<Hand::frames_t> (3U, hand.maxMuscleLast (muscle_j), 2) )
              for ( Hand::frames_t last_j : boost::irange<Hand::frames_t> (3U, hand.maxMuscleLast (muscle_j)) )
              {
                // for ( Hand::frames_t start_j : boost::irange<Hand::frames_t> (5U, last_i, 2) ) // ?? + inertial_lasts
                for ( Hand::frames_t start_j : boost::irange<Hand::frames_t> (5U, last_i) )
                {
                  boost::this_thread::interruption_point ();

                  HandMoves::trajectory_t  trajectory;
                  Hand::Control control_j (muscle_j, start_j, last_j);

                  hand.SET_DEFAULT;
                  hand.move ({ control_i, control_j }, &trajectory);

                  // if ( hand.position.x > x_leftBorder && hand.position.x < x_rightBorder
                  //   && hand.position.y < y_topBorder && hand.position.y > y_bottomBorder )

                  const Point&  hand_pos = hand.position;
                  if ( target.contain (hand_pos) )
                  {
                    store.insert (HandMoves::Record (hand_pos, hand_base, hand_pos,
                    { muscle_i, muscle_j }, { 0, start_j }, { last_i, last_j }, 2U,
                                  trajectory));
                    // trajectories.push_back (make_shared<trajectory_t> (trajectory));

                    if ( store.size () == 200000 )
                    {
                      store.save (_T ("NewHand_200000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      std::wcout << _T ("NewHand_200000_moves_tightly_saved") << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                    else if ( store.size () == 400000 )
                    {
                      store.save (_T ("NewHand_400000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      std::wcout << _T ("NewHand_400000_moves_tightly_saved") << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                    else if ( store.size () == 600000 )
                    {
                      store.save (_T ("NewHand_600000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      std::wcout << _T ("NewHand_600000_moves_tightly_saved") << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                    else if ( store.size () == 1000000 )
                    {
                      store.save (_T ("NewHand_1000000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      std::wcout << _T ("NewHand_1000000_moves_tightly_saved") << std::endl;
                      std::wcout << muscle_i << ' ' << last_i << std::endl;
                      std::wcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                    else if ( store.size () == 1500000 )
                    {
                      store.save (_T ("NewHand_1500000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      std::wcout << _T ("NewHand_1500000_moves_tightly_saved") << std::endl;
                      std::wcout << muscle_i << ' ' << last_i << std::endl;
                      std::wcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                    else if ( store.size () == 2000000 )
                    {
                      store.save (_T ("NewHand_2000000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      std::wcout << _T ("NewHand_2000000_moves_tightly_saved") << std::endl;
                      std::wcout << muscle_i << ' ' << last_i << std::endl;
                      std::wcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                  }

#define   _HAND_TEST_BREAK
#ifdef    _HAND_TEST_BREAK
                  if ( boost_distance (hand_pos, target.center ())
            > boost_distance (hand_prev, target.center ()) )
                  {
                    // #ifdef    _HAND_TEST_CONSOLE_PRINTF
                    // std::wcout << _T ("break") << std::endl;
                    // #endif // _HAND_TEST_CONSOLE_PRINTF
                    break;
                  }
                  hand_prev = hand_pos;
#endif // _HAND_TEST_BREAK
                  try
                  { boost::this_thread::interruption_point (); }
                  catch ( boost::thread_interrupted& )
                  {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                    std::wcout << _T ("WorkingThread interrupted") << std::endl;
                    std::wcout << muscle_i << ' ' << last_i << std::endl;
                    std::wcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    return;
                  } // end catch
                } // end for
              } // end for
            } // end for
          } // end if
        } // end for
      } // end for
    } // end try
    catch ( std::exception &ex )
    {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
      std::cout << ex.what () << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
      return;
    }
  }
  //------------------------------------------------------------------------------
  struct Direction
  {
    double             angle; // with Ox
    Point              direction;
    Hand::MusclesEnum  control;
    Point              norm_dir;

    struct Ellipse
    {


    };

    Direction () :
      direction (0., 0.),
      control (Hand::EmptyMov),
      angle (0.)
    {}

    Direction (Point direction, Hand::MusclesEnum control) :
      direction (direction),
      control (control),
      angle (atan2 (direction.y, direction.x))
    {}

    double  cos_sim (const Direction &d) const
    {
      auto  ad = direction;
      auto  bd = d.direction;
      /* cosine similarity */
      return    (ad.x * bd.x + ad.y * bd.y)
        / (sqrt (ad.x * ad.x + ad.y * ad.y)
           *  sqrt (bd.x * bd.x + bd.y * bd.y));
    }

    void draw (HDC hdc)
    {
      if ( !norm_dir.x && !norm_dir.y )
        norm_dir = Point (0.1 * cos (angle), 0.1 * sin (angle));

      MoveToEx (hdc, Tx (0.), Ty (0.), NULL);
      LineTo (hdc, Tx (norm_dir.x), Ty (norm_dir.y));
    }

  };
  void  LearnMovements::getTargetCenter (HandMoves::Store &store, Hand &hand, Point &center)
  {
    hand.SET_DEFAULT;
    Point start = hand.position;



    std::vector<Direction>  directions;
    directions.reserve (hand.muscles_.size ());

    //hand.selectControl ()
    // for ( auto muscle_i : hand.muscles_ )
    for ( auto iCtrl = 0U; iCtrl < hand.controlsCount; ++iCtrl )
    {
      auto muscle_i = hand.selectControl (iCtrl);

      hand.move (muscle_i, 1U);
      // hand.step (muscle_i, 1U); hand.step ();

      directions.push_back (Direction (Point (hand.position.x - start.x,
                            hand.position.y - start.y), muscle_i));
    }
    // --------------------
    Direction   direct = Direction (Point (center.x - start.x,
                                    center.y - start.y), Hand::EmptyMov);

    Direction  &d = *std::min_element (directions.begin (), directions.end (),
                                       [&direct](const Direction &a, const Direction &b) { return  direct.cos_sim (a) < direct.cos_sim (b); });
    // { return  boost_distance (direct.direction, a.direction)
    //         > boost_distance (direct.direction, b.direction); });
    // --------------------

    auto max_last = hand.maxMuscleLast (d.control);
    hand.move (d.control, max_last);

    double center_distance = boost_distance (center, start);
    double maxdir_distance = boost_distance (hand.position, start);

    Hand::frames_t  last = static_cast<Hand::frames_t> ((center_distance * max_last) / maxdir_distance);
    hand.move (d.control, last);
  }
  //------------------------------------------------------------------------------
  bool  LearnMovements::tryToHitTheAim (HandMoves::Store &store, Hand &hand, const Point &aim,
                                        size_t tries, double precision)
  {
    size_t complexity = 0U;

    // borders (store, target);
    DirectionPredictor  pd (hand);

    hand.reset ();
    hand.SET_DEFAULT;

    bool result = 0;
    std::list<std::shared_ptr<HandMoves::Record>>  exact_range;
    /* While there is no exact trajectory */
    for ( size_t n_try = 0U; (result = exact_range.empty ()) || n_try < tries; ++n_try )
    {
      store.adjacencyPoints (exact_range, aim, precision);

    }
    return !result;
  }
  //------------------------------------------------------------------------------
  static void  testCoverTarget1 (HandMoves::Store &store, Hand &hand, RecTarget &target)
  {
    hand.SET_DEFAULT;
    // std::list<std::list<Point>> trajectories;

    /* Fixate the aim point */
    boost_point2_t  aim = target.coords ()[45];
    double prev_distance = boost::geometry::distance (aim, hand.pos);

    double             best_distance = prev_distance;
    Hand::MusclesEnum  best_muscle = Hand::EmptyMov;
    Hand::time_t       best_last;

    // while ( aim.y () < hand.position.y )
    {
      /* Try each muscle */
      for ( auto i : boost::irange<size_t> (1U, hand.controlsCount) )
      {
        auto muscle = hand.selectControl (i);
        auto last = hand.maxMuscleLast (muscle);

        hand.step (muscle);
        hand.step ();
        /* Find the best muscle by distance */
        double cur_distance = boost::geometry::distance (hand.pos, aim);
        if ( cur_distance < best_distance )
        {
          best_muscle = muscle;
          best_last = last;
          best_distance = cur_distance;
        }

        hand.reset ();
        hand.SET_DEFAULT;
      }

      hand.step (best_muscle);
      // while ( last-- )
      prev_distance = best_distance;
      double cur_distance = best_distance;
      auto last_ = 1U;

      do
        // for ( auto last_ = 1U; (prev_distance >= cur_distance) && (last_ <= best_last); ++last_ )
      {
        prev_distance = cur_distance;
        //-----------
        hand.step ();
        //-----------
        cur_distance = boost::geometry::distance (hand.pos, aim);
        ++last_;
      } while ( (prev_distance >= cur_distance) && (last_ <= best_last) );

      if ( !hand.moveEnd )
        hand.step (best_muscle);

      while ( !hand.moveEnd )
        hand.step ();
    }
    //=========================================================================
    // // const to target
    // const boost_point2_t  tlu (target.Min ().x, target.Max ().y),
    //                       tld (target.Min ().x, target.Min ().y),
    //                       trd (target.Max ().x, target.Min ().y),
    //                       tru (target.Max ().x, target.Max ().y);
    // 
    // const array<boost_point2_t, 4> target_corners = { tlu, tld, trd, tru };
    // const boost_point2_t *t_closest;
    // double distance = 0.;

    /* Create the tree of possible passes */
    //for ( Hand::MusclesEnum muscle_i : Hand::muscles )
    //{
    //  // auto h = hand.position;

    //  for ( uint_t last_i = 1U; last_i < hand.maxMuscleLast (muscle_i); ++last_i )
    //  {
    //    std::list<Point> trajectory;
    //    
    //    
    //    const boost_point2_t pos = hand.position;
    //    t_closest = &target_corners[0];
    //    distance = boost::geometry::distance (t_closest, pos);
    //    for ( size_t i = 1U; i < target_corners.size (); ++i )
    //    {
    //      double d = boost::geometry::distance (target_corners[i], pos);
    //      if ( distance > d )
    //      {
    //        distance = d;
    //        t_closest = &target_corners[i];
    //      }
    //    }

    //    // hand.move (muscle_i, last_i, trajectory);
    //    hand.step (muscle_i);
    //    // while ( last-- )
    //    for ( auto last_ = 1U; last_ <= last_i; ++last_ )
    //    {
    //      hand.step ();

    //      const boost_point2_t pos = hand.position;
    //      double d = boost::geometry::distance (pos, t_closest);
    //    }
    //    if ( !hand.isMoveEnd () )
    //      hand.step (muscle_i);

    //    while ( !hand.isMoveEnd () )
    //      hand.step ();

    //{
    //  const Point &aim = hand.position;
    //  Record rec (aim, aim,
    //              { muscle_i }, { 0 }, { last_i },
    //              1U, trajectory);
    //  store.insert (rec);
    //}
    //=========================================================================
    //// std::cout << muscle_i << ' ' << hi << ' ' << last_i << std::endl;
    //insert (trajectories, trajectory);

    //if ( nesting > 1U )
    //  for ( Hand::MusclesEnum muscle_j : Hand::muscles )
    //  {
    //    if ( (muscle_i == muscle_j) || !muscleValidAtOnce (muscle_i | muscle_j) )
    //      // if ( j == i )
    //      continue;

    //    Point cur1 = hand.position;

    //    for ( uint_t last_j = 1U; last_j < hand.maxMuscleLast (muscle_j) / 2; ++last_j )
    //    {
    //      std::list<Point>::iterator tail_j = trajectory.end (); --tail_j;

    //      hand.move (muscle_j, last_j, trajectory);
    //      {
    //        const Point &aim = hand.position;
    //        store.insert (Record (aim, aim,
    //        { muscle_i, muscle_j },
    //        { 0, last_i },
    //        { last_i, last_j },
    //                      2U,
    //                      trajectory)
    //                      );
    //      }

    //      // std::cout << muscles[j] << ' ' << hj << ' ' << last_j << std::endl;
    //      ++tail_j;

    //      // std::list::splice () allows you to concatenate two std::list's in constant time.
    //      insert (trajectories, trajectory);
    //      //=================================================
    //      if ( nesting > 2U )
    //        for ( Hand::MusclesEnum muscle_k : Hand::muscles )
    //        {
    //          if ( (muscle_i == muscle_k || muscle_k == muscle_j)
    //              || !muscleValidAtOnce (muscle_i | muscle_j | muscle_k) )
    //            // if ( k == i || k == j )
    //            continue;

    //          for ( auto last_k = 1U; last_k < hand.maxMuscleLast (muscle_k) / 2; ++last_k )
    //          {
    //            std::list<Point>::iterator tail_k = trajectory.end (); --tail_k;
    //            hand.move (muscle_k, last_k, trajectory);
    //            {
    //              const Point &aim = hand.position;
    //              store.insert (Record (aim, aim,
    //              { muscle_i, muscle_j, muscle_k },
    //              { 0, last_i, last_i + last_j },
    //              { last_i, last_j, last_k },
    //                            3U,
    //                            trajectory)
    //                            );
    //            }

    //            // std::cout << muscles[k] << ' ' << hk << ' ' << last_k << std::endl;
    //            ++tail_k;
    //            insert (trajectories, trajectory);
    //            trajectory.erase (tail_k, trajectory.end ());

    //            hand.set (jointByMuscle (muscle_k), { (jointByMuscle (muscle_k) == Hand::Elbow) ? 70. : 0. });
    //          }
    //        }
    //      //=================================================
    //      trajectory.erase (tail_j, trajectory.end ());
    //      //a.insert (a.end (), b.begin (), b.end ());

    //      hand.set (jointByMuscle (muscle_j), { (jointByMuscle (muscle_j) == Hand::Elbow) ? 70. : 0. });
    //    }
    //  }
    //     hand.set (jointByMuscle (muscle_i), { (jointByMuscle (muscle_i) == Hand::Elbow) ? 70. : 0. });
    //   }
    // }
    //hand.SET_DEFAULT;


  }
  //------------------------------------------------------------------------------
};
