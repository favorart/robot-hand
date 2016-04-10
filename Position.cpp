#include "StdAfx.h"
#include "Position.h"

namespace Positions
{

  void testGradientMethod (HandMoves::Store &store, Hand &hand, const Point &aim)
  {
    return;
  }

  using namespace std;
  using namespace HandMoves;
  //------------------------------------------
        DirectionPredictor::DirectionPredictor (IN Hand &hand)
  {
    hand.SET_DEFAULT;
    hand_base = hand.position;

    for ( auto j : hand.joints_ )
    {
      Hand::MusclesEnum muscle;

      hand.set (Hand::JointsSet{ {j, 100.} });
      muscle = muscleByJoint (j, true);
      main_directions[muscle] = MainDirection (muscle, hand);

      hand.set (Hand::JointsSet{ {j, 0.} });
      muscle = muscleByJoint (j, false);
      main_directions[muscle] = MainDirection (muscle, hand);

      hand.SET_DEFAULT;
    }
    // for ( auto m : hand.muscles_ )
    // { mainDirections[m] = MainDirection (m, hand); }
  }
  void  DirectionPredictor::predict (IN Hand::Control control, OUT Point &end)
  {
    for ( Hand::MusclesEnum m : muscles )
    {
      const MainDirection  &md = main_directions[m & control.muscle];
      if ( control.last < md.shifts.size () )
      {
        end.x += md.shifts[control.last].x;
        end.y += md.shifts[control.last].y;
      } // end if
    } // end for
  }

  bool  DirectionPredictor::shifting_gt (Hand::MusclesEnum m, unsigned int &inx, const Point &aim)
  {
    bool changes = false;
    Point  prev, curr;

    auto &shifts = main_directions[m].shifts;

    curr.x = hand_base.x + shifts[inx].x;
    curr.y = hand_base.y + shifts[inx].y;
    do
    {
      prev = curr;

      curr.x = hand_base.x + shifts[inx + 1].x;
      curr.y = hand_base.y + shifts[inx + 1].y;

      ++inx;
    } while ( (inx < (shifts.size () - 1U))
           && (boost_distance (aim, curr) < boost_distance (aim, prev))
           && (changes = true) );

    return changes;
  }
  bool  DirectionPredictor::shifting_ls (Hand::MusclesEnum m, unsigned int &inx, const Point &aim)
  {
    bool changes = false;
    Point  prev, curr;

    auto &shifts = main_directions[m].shifts;

    curr.x = hand_base.x + shifts[inx].x;
    curr.y = hand_base.y + shifts[inx].y;
    do
    {
      prev = curr;

      curr.x = hand_base.x + shifts[inx - 1].x;
      curr.y = hand_base.y + shifts[inx - 1].y;

      --inx;
    } while ( (inx > 0)
             && (boost_distance (aim, curr) > boost_distance (aim, prev))
             && (changes = true) );

    return changes;
  }

  void  DirectionPredictor::measure (IN Hand &hand, IN  const Point &aim,
                                     OUT HandMoves::controling_t &controls)
  { /*
     *  У нас есть 4 линейки, их надо сопоставить так,
     *  чтобы приблизить aim.
     *
     *  Система линейных уравнений
     */

    std::vector<int> indices (hand.joints_.size ());
    // std::fill (indices.begin (), indices.end (), 1U);

    bool changes = true;
    while ( changes )
    {
      for ( size_t ij = 0; ij < hand.joints_.size (); ++ij )
      {
        auto mo = muscleByJoint (hand.joints_[ij], true);
        auto mc = muscleByJoint (hand.joints_[ij], false);

        changes = false;
        // wcout << hand.joints_[ij] << endl;

        if ( indices[ij] >= 0 )
        {
          unsigned int inx = indices[ij];

          changes = shifting_gt (mo, inx, aim);
          if ( !changes )
            changes = shifting_ls (mo, inx, aim);

          indices[ij] = inx;
        }
        
        if ( indices[ij] <= 0 )
        {
          unsigned int inx = -indices[ij];

          changes = shifting_gt (mo, inx, aim);
          if ( !changes )
            changes = shifting_ls (mo, inx, aim);

          indices[ij] = -(int)inx;
        } // end if
      } // end for
    } // end while

    for ( size_t ij = hand.joints_.size (); ij > 0; --ij )
    {
      auto mo = muscleByJoint (hand.joints_[ij - 1U], true);
      auto mc = muscleByJoint (hand.joints_[ij - 1U], false);

      if ( indices[ij - 1U] > 0 )
        controls.push_back (Hand::Control( mo, 0U, size_t(  indices[ij - 1U] ) /* - stopping ? */ ));
      else if ( indices[ij - 1U] < 0 )
        controls.push_back (Hand::Control( mc, 0U, size_t( -indices[ij - 1U] ) /* - stopping ? */ ));
    }

    // Point  base;
    // if ( base.x > aim.x )
    // {}
    // if ( base.y > aim.y )
    // {}
    // 
    // while ( base.x < aim.x )
    // {}
    // while ( base.y < aim.y )
    // {}
    // 
    // for ( auto &md : main_directions )
    // {
    //   int  j = 0, const_t = 5;
    //   Hand::Control  control (md.muscle, 0U, j - const_t);
    // }
  }
  //------------------------------------------
  void  testCover (HandMoves::Store &store, Hand &hand,
                   HandMoves::trajectories_t &trajectories)
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
  struct counts_t
  {
    int count = 0;
    int count_TP = 0;
    int count_FP = 0;
    int count_TN = 0;
    int count_FN = 0;

    void  incr  (bool model, bool real)
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
    void  fill (Hand &hand, RecTarget &target,
                HandMoves::controling_t &controls,
                const Point &end)
    {
      //-------------------------------------------------------------
      HandMoves::trajectory_t  trajectory;
      hand.SET_DEFAULT;
      hand.move (controls.begin (), controls.end (), &trajectory);
      //-------------------------------------------------------------
      bool model = target.contain (end);
      bool real = target.contain (hand.position);
      incr (model, real);
    }
    void  print ()
    {
      tcout << _T("count = ") << count << std::endl;
      tcout << _T("\t< T >\t < F >") << std::endl;
      tcout << _T("< P >\t") << count_TP << _T("\t") << count_FP << std::endl;
      tcout << _T("< N >\t") << count_TN << _T("\t") << count_FN << std::endl;
    }
    void  clear ()
    {
      count = 0;
      count_TP = 0;
      count_FP = 0;
      count_TN = 0;
      count_FN = 0;
    }
  };
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
    HandMoves::controling_t  controls;
    //---braking----------------------------------------------
    std::vector</*std::list<*/Hand::Control/*>*/>  arr_controlings;
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

    Point last_point;

    const Hand::frames_t INIT = 100U;


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
                              (last_i  < lasts_i_max) && (target_contain) && // || /* only last */ !was_on_target) &&
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
          for ( size_t ji = 0; ji <= joint_index; ++ji)
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
                                (last_i  - lasts_step) < lasts_i_max && target_contain;
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
                                                                 [](const Hand::Control &item)
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
  void  LearnMovements::STAGE_1 (HandMoves::Store &store, Hand &hand, RecTarget &target)
  {
    borders_t  borders;
    defineBorders (borders, hand, 70U);
    DirectionPredictor  pd (hand);

    TourWorkSpace  tour (store, hand, target);
    tour.run (borders, pd, false, false, 0.05, 5U);
  }
  void  LearnMovements::STAGE_2 (HandMoves::Store &store, Hand &hand, RecTarget &target)
  {
    borders_t  borders;
    defineBorders (borders, target, store, 0.005);
    DirectionPredictor  pd (hand);

    TourWorkSpace  tour (store, hand, target);
    tour.run (borders, pd, /* target */ true, /* braking */ true, 0.005, 1U);
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
  void  LearnMovements::testStage1 (HandMoves::Store &store, Hand &hand, RecTarget &target)
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
        if ( boost::algorithm::none_of (range, [&pt, precision](const std::shared_ptr<HandMoves::Record> &rec)
                                               { return  (boost_distance (pt, rec->hit) <= precision); }) )
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
    { tcout << ex.what () << std::endl;
#else
    catch (...)
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
};
