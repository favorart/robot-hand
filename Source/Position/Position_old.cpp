#include "StdAfx.h"
#include "Position.h"
#include "DirectionPredictor.h"

namespace Positions
{
  using namespace std;
  using namespace HandMoves;
  //------------------------------------------------------------------------------
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // !!! Борьба с КОЛИЧЕСТВОМ, которая перетягивает начальную точку !!!
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  void  LearnMovements::Mean (IN const Point &aim,
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
      tcout << std::endl << _T ("LearnMovements::Mean") << std::endl;
      tcout << _T ("min=") << min << _T (" max=") << max << std::endl;
#endif // _DEBUG_PRINT
      // -----------------------------------------------
      HandMoves::adjacency_ptrs_t  range;
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
                control.start += Hand::frames_t(weight * c.start);
                control.last += Hand::frames_t(weight * c.last);
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
          if ( n_control_summands ) control.last /= n_control_summands;
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
      { rundownFull (aim, hand_position); }

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

  void  LearnMovements::Close (IN const Point &aim,
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
    HandMoves::adjacency_ptrs_t  range;
    // auto count = store.adjacencyPoints (range, aim, side);
    auto  count = store.adjacencyRectPoints<adjacency_ptrs_t, ByP> (range, min, max);
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
    { rundownFull (aim, hand_position); }

    // -----------------------------------------------
#ifdef _DEBUG_PRINT_RES
    tcout << _T ("ResPrec: ") << distance << std::endl;
    tcout << _T ("Complexity: ") << complexity << std::endl << std::endl;
#endif // _DEBUG_PRINT_RES
  }

  //------------------------------------------------------------------------------
  void  LearnMovements::rundownMethod_old (IN const Point &aim,
                                           IN const HandMoves::controling_t &init_controls,
                                           IN Point &hand_position)
  {
    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;
    Point hand_pos = hand_position;
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
    std::list<std::shared_ptr<Hand::Control>>  ordered_controls;
    for ( auto &c : controls )
    { ordered_controls.push_back (std::make_shared<Hand::Control> (c)); }
    ordered_controls.sort ([](const shared_ptr<Hand::Control> &sca,
                              const shared_ptr<Hand::Control> &scb)
    {
      return sca->muscle < scb->muscle;
    });
    // -----------------------------------------------
    Hand::frames_t lasts_step = 1U;
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

          handAct (aim, controls, hand_pos);
          next_distance = boost_distance (hand_pos, aim);

          if ( next_distance >= prev_distance )
          {
            (**it_straight).last -= lasts_step;
            (**it_opposite).start -= lasts_step;

            do
            {
              prev_distance = next_distance;
              (**it_opposite).last -= lasts_step;
              handAct (aim, controls, hand_pos);
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

          handAct (aim, controls, hand_pos);
          next_distance = boost_distance (hand_pos, aim);

          if ( next_distance >= prev_distance )
          {
            (**it_straight).last += lasts_step;
            (**it_opposite).start += lasts_step;
            do
            {
              prev_distance = next_distance;
              (**it_opposite).last += lasts_step;

              handAct (aim, controls, hand_pos);
              next_distance = boost_distance (hand_pos, aim);

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
  void  testCover (IN  HandMoves::Store &store, IN Hand &hand,
                   OUT HandMoves::trajectories_t &trajectories)
  {
    hand.reset ();
    hand.SET_DEFAULT;

    Point hand_base = hand.position;

    /*    Покрыть всё в одно движение
    *
    *    Начать двигать какой-то мышцой (варьировать длительность),
    *    К её движению сложить все комбинации остальных мышц.
    *
    */

    /* Возьмём первый мускул наугад */
    for ( auto muscle_i : hand.muscles_ /* | boost::adaptors::sliced (3, 4) */ )
    {
      // if ( !(muscle_i & muscles) && musclesValidUnion(muscle_i | muscles) )
      {
        HandMoves::trajectory_t  trajectory;
        /* Попробуем его варьировать по длительности */
        for ( Hand::frames_t last_i : boost::irange<Hand::frames_t> (1U, hand.maxMuscleLast (muscle_i) / 2, 10) )
        {
          Hand::Control control_i (muscle_i, 0U, last_i);
          // hand.move ({ control_i });

          for ( auto muscle_j : hand.muscles_ )
            if ( (muscle_i != muscle_j) && musclesValidUnion (muscle_i | muscle_j) )
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
      //   if ( !(muscle_j & muscles) && musclesValidUnion (muscle_i | muscles) )
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
      //   /*  Теперь в каждый такт длительности первого мускула,
      //    *  включим второй и будет варьировать его длительность
      //    */
      //   testCover (store, hand, muscles | muscle_i, trajectories);
      // 
      // } // end for
      //} // end if
    } // end for
  }
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

      // else if ( lasts_step == 1 && lasts_incr_value == 1U )
      // { }
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
  //------------------------------------------------------------------------------
  /* грубое покрытие всего рабочего пространства */
  void  LearnMovements::testStage1 (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target)
  {
    size_t complexity = 0U;

    hand.reset ();
    hand.SET_DEFAULT;

    Point  hand_pos_base = hand.position;

    Point  hand_pos_prev_i = hand_pos_base;
    Point  hand_pos_prev_j = hand_pos_base;

    /* Возьмём первый мускул наугад */
    for ( auto muscle_i : { muscleByJoint (hand.joints_[0], false), muscleByJoint (hand.joints_[0], true) } )
      for ( auto muscle_j : { muscleByJoint (hand.joints_[1], false), muscleByJoint (hand.joints_[1], true) } )
        // if ( (muscle_i != muscle_j) && musclesValidUnion (muscle_i | muscle_j) )
      {
        Point  hand_pos_i;
        /* Нужен уменьшающийся шаг в зависимости от пройдённой дистанции */
        Hand::frames_t  lasts_step = lasts_init_value1;

        auto last_i_max = hand.maxMuscleLast (muscle_i),
          last_j_max = hand.maxMuscleLast (muscle_j);
        /* Попробуем его варьировать по длительности */
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
    tcout << _T ("\nStage 1 Complexity ") << complexity << std::endl;
    // ----------------------------------------------------
  }
  /* Покрытие всей мишени не слишком плотно */
  void  LearnMovements::testStage2 (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target)
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

    borders_t borders;
    defineBorders (borders, target, store, draft_distance);
    DirectionPredictor  pd (hand);

    tcout << _T ("borders: ") << borders.size () << std::endl;

    hand.reset ();
    hand.SET_DEFAULT;

    Point  hand_pos_base = hand.position;

    Point  hand_pos_prev_i = hand_pos_base;
    Point  hand_pos_prev_j = hand_pos_base;

    counts_t stats;

    draft_distance = 0.1;
    Hand::frames_t  last_step = 1U;

    for ( auto muscle_i : { muscleByJoint (hand.joints_[0], false), muscleByJoint (hand.joints_[0], true) } )
      for ( auto muscle_j : { muscleByJoint (hand.joints_[1], false), muscleByJoint (hand.joints_[1], true) } )
        // if ( (muscle_i != muscle_j) && musclesValidUnion (muscle_i | muscle_j) )
      {
        Point  hand_pos_i;
        /* Нужен уменьшающийся шаг в зависимости от пройдённой дистанции */
        Hand::frames_t  last_step = lasts_incr_value2;

        auto  last_i_max = hand.maxMuscleLast (muscle_i),
          last_j_max = hand.maxMuscleLast (muscle_j);

        bool target_contain = true;
        for ( Hand::frames_t last_i = borders[muscle_i].first;
        last_i > 0 && /* target_contain || */ last_i <= borders[muscle_i].second;
          last_i += last_step )
        {
          for ( Hand::frames_t last_j = borders[muscle_j].first /*- 50*/;
          last_j > 0 && /* target_contain || */ last_j <= borders[muscle_j].second /* + 50 */;
            last_j += last_step )
          {
            Hand::Control  control_i (muscle_i, 0U, last_i);
            Hand::Control  control_j (muscle_j, 0U /* start_j */, last_j);

            Point end = hand_pos_base;

            pd.predict (control_i, end);
            pd.predict (control_j, end);

            stats.fill (hand, target, controling_t{ control_i, control_j }, end);

            /* типо на мишени */
            if ( target.contain (end) )
            {
              HandMoves::trajectory_t  trajectory;
              hand.SET_DEFAULT;
              hand.move ({ control_i, control_j }, &trajectory);

              Point  hand_pos_j = hand.position;
              if ( last_j == borders[muscle_j].first )
                hand_pos_i = hand_pos_j;
              //----------------------------------------------
              target_contain = target.contain (hand_pos_j);
              // tcout << tstring(hand_pos_j) << std::endl;
              //----------------------------------------------
              HandMoves::Record  rec (hand_pos_j, hand_pos_base, hand_pos_j,
              { control_i, control_j }, trajectory);
              store.insert (rec);

              draftDistance (hand_pos_prev_j, hand_pos_j, last_step,
                             last_j, last_j_max, lasts_incr_value2,
                             detail_distance);
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
    tcout << _T ("\nStage 2 Complexity ") << complexity << std::endl;
    // ----------------------------------------------------
    hand.SET_DEFAULT;
  }
  /* Попадание в оставшиеся непокрытыми точки мишени */
  void  LearnMovements::testStage3 (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target, OUT std::list<Point> &uncovered)
  {
    // std::list<Point> uncovered;
    // store.uncoveredTargetPoints (target, uncovered);
    for ( auto &pt : target.coords () )
    {
      adjacency_ptrs_t  range;
      store.adjacencyPoints (range, pt, 2. * target.thickness ());

      if ( range.empty () )
      {
        /* UNCOVERED */
        uncovered.push_back (pt);
      }
      else
      {
        double precision = target.precision ();
        if ( boost::algorithm::none_of (range, [&pt, precision](const Record *rec)
                                               // (const std::shared_ptr<HandMoves::Record> &rec)
                                               { return  (boost_distance (pt, rec->hit) <= precision); }) )
        {
          /* UNCOVERED */
          uncovered.push_back (pt);

        } // end if

      } // end else
    } // end for
  }
  //------------------------------------------------------------------------------
  void  LearnMovements::testCoverTarget (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target)
  {
    hand.reset ();
    hand.SET_DEFAULT;

    Point hand_base = hand.position;
    Point hand_prev = hand.position;

    /*    Покрыть всё в одно движение
    *
    *    Начать двигать какой-то мышцой (варьировать длительность),
    *    К её движению сложить все комбинации остальных мышц.
    */
    try
    {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
      tcout << _T ("hand.muscles_.size = ") << (int) hand.muscles_.size () << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
      /* Возьмём первый мускул наугад */
      for ( auto muscle_i : hand.muscles_ )
      {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
        tcout << muscle_i << _T ("  ") << (int) hand.maxMuscleLast (muscle_i) << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
        for ( auto muscle_j : hand.muscles_ )
        {
          if ( (muscle_i != muscle_j) && musclesValidUnion (muscle_i | muscle_j) )
          {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
            tcout << muscle_j << _T ("  ") << (int) hand.maxMuscleLast (muscle_j) << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
            /* Попробуем его варьировать по длительности */

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
                      tcout << _T ("NewHand_200000_moves_tightly_saved") << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                    else if ( store.size () == 400000 )
                    {
                      store.save (_T ("NewHand_400000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      tcout << _T ("NewHand_400000_moves_tightly_saved") << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                    else if ( store.size () == 600000 )
                    {
                      store.save (_T ("NewHand_600000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      tcout << _T ("NewHand_600000_moves_tightly_saved") << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                    else if ( store.size () == 1000000 )
                    {
                      store.save (_T ("NewHand_1000000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      tcout << _T ("NewHand_1000000_moves_tightly_saved") << std::endl;
                      tcout << muscle_i << ' ' << last_i << std::endl;
                      tcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                    else if ( store.size () == 1500000 )
                    {
                      store.save (_T ("NewHand_1500000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      tcout << _T ("NewHand_1500000_moves_tightly_saved") << std::endl;
                      tcout << muscle_i << ' ' << last_i << std::endl;
                      tcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                    else if ( store.size () == 2000000 )
                    {
                      store.save (_T ("NewHand_2000000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                      tcout << _T ("NewHand_2000000_moves_tightly_saved") << std::endl;
                      tcout << muscle_i << ' ' << last_i << std::endl;
                      tcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                    }
                  }

#define   _HAND_TEST_BREAK
#ifdef    _HAND_TEST_BREAK
                  if ( boost_distance (hand_pos, target.center ())
      > boost_distance (hand_prev, target.center ()) )
                  {
                    // #ifdef    _HAND_TEST_CONSOLE_PRINTF
                    // tcout << _T ("break") << std::endl;
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
                    tcout << _T ("WorkingThread interrupted") << std::endl;
                    tcout << muscle_i << ' ' << last_i << std::endl;
                    tcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
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
#ifdef    _HAND_TEST_CONSOLE_PRINTF
    catch ( std::exception &ex )
    {
      tcout << ex.what () << std::endl;
#else
    catch ( ... )
    {
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
  void  LearnMovements::getTargetCenter (IN HandMoves::Store &store, IN Hand &hand, OUT Point &center)
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
  bool  LearnMovements::tryToHitTheAim (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim)
  {
    size_t complexity = 0U;

    // borders (store, target);
    DirectionPredictor  pd (hand);

    hand.reset ();
    hand.SET_DEFAULT;

    bool result = 0;
    std::list<std::shared_ptr<HandMoves::Record>>  exact_range;
    /* While there is no exact trajectory */
    for ( size_t n_try = 0U; (result = exact_range.empty ()) || n_try < hit_tries; ++n_try )
    {
      store.adjacencyPoints (exact_range, aim, hit_precision);

    }
    return !result;
  }
  //------------------------------------------------------------------------------
  static void  testCoverTarget1 (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target)
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
    //// tcout << muscle_i << ' ' << hi << ' ' << last_i << std::endl;
    //insert (trajectories, trajectory);

    //if ( nesting > 1U )
    //  for ( Hand::MusclesEnum muscle_j : Hand::muscles )
    //  {
    //    if ( (muscle_i == muscle_j) || !musclesValidUnion (muscle_i | muscle_j) )
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

    //      // tcout << muscles[j] << ' ' << hj << ' ' << last_j << std::endl;
    //      ++tail_j;

    //      // std::list::splice () allows you to concatenate two std::list's in constant time.
    //      insert (trajectories, trajectory);
    //      //=================================================
    //      if ( nesting > 2U )
    //        for ( Hand::MusclesEnum muscle_k : Hand::muscles )
    //        {
    //          if ( (muscle_i == muscle_k || muscle_k == muscle_j)
    //              || !musclesValidUnion (muscle_i | muscle_j | muscle_k) )
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

    //            // tcout << muscles[k] << ' ' << hk << ' ' << last_k << std::endl;
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

  //void  /*HandMoves::*/testCover (Store &store, Hand &hand,
  //                            Hand::MusclesEnum muscles /* recursive */,
  //                            int recursive,
  //                            std::list<int> step,
  //                            std::list<Point> trajectory1)
  //{
  //  // hand.reset ();
  //  hand.SET_DEFAULT;
  //
  //  Point hand_base = hand.position;
  //
  //  for ( int i = step[recursive]; i < hand.muscles_.size (); ++i )
  //  {
  //    auto muscle_i = hand.muscles_[i];
  //    if ( !(muscle_i & muscles) )
  //    {
  //      trajectory_t  trajectory;
  //
  //      /* Попробуем его варьировать по длительности */
  //      for ( Hand::frames_t last_i : boost::irange (1U, hand.maxMuscleLast (muscle_i)) )
  //      {
  //        hand.move (muscle_i, last_i, trajectory);
  //        store.insert (Record (hand.position, hand_base, hand.position,
  //                             { muscle_i }, { 0 }, { last_i }, 1U,
  //                             trajectory)
  //                     );
  //
  //        std::copy (trajectory.begin (), trajectory.end (), trajectory1.begin ());
  //        if ( step.size < recursive + 1 )
  //        {}
  //
  //        testCover (store, hand, muscles & muscle_i, recursive + 1, step, trajectory1);
  //
  //      } // end for
  //    } // end if
  //  } // end for
  //}

  //------------------------------------------------------------------------------
};
