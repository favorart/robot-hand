#include "StdAfx.h"
#include "Position.h"

namespace Positions
{
  using namespace std;
  using namespace HandMoves;
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
    //---flags------------------------------------------------
    bool  b_distance, b_target, b_braking, b_checking;
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
    double  max_distance;
    const Point &center;
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
      max_distance = (target.max ().x - target.min ().x) / 2.3; // !!!!!!!!!!!!!!!!!!!!!!!!!!!
    }

    void  run (IN borders_t &borders, IN DirectionPredictor &pd,
               IN bool distance, IN bool target, IN bool braking, IN bool checking,
               IN double step_distance, IN Hand::frames_t lasts_step_increment,
               IN bool verbose=false)
    {
      this->borders = &borders;
      this->pd = &pd;
      // ----------------------------------------------------
      this->b_target = target;
      this->b_braking = braking;
      this->b_distance = distance;
      this->b_checking = checking;
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
      { 
        /* tcout << _T("WorkingThread interrupted") << std::endl; */
        return;
      } // end catch
      // ----------------------------------------------------
      if ( verbose )
      { stats.print ();
        tcout << _T ("\nComplexity: ") << complexity << std::endl;
      }
    }

  private:
    bool  runNestedForMuscle (OUT Point &hand_pos_high, IN int joint_index)
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
          Hand::frames_t lasts_step = lasts_step_increment;
          Hand::frames_t lasts_i_max = hand.maxMuscleLast (muscle_i);

          start_i = 0U;
          target_contain = true;

          Hand::frames_t last_i = 0U;
          for ( last_i = (board.first);
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
              else
              { lasts_step += lasts_step_increment; }
            }
            //-----------------------------
            hand_pos_prev = hand_pos_curr;
          } // end for (last)

          if ( b_target )
          { board.second = last_i; }
          //------------------------------------------
          for ( size_t ji = 0; ji <= joint_index; ++ji )
          { arr_controlings[ji].last = 0U; }
          //------------------------------------------
          if ( b_target )
          {
            start_i = 0U;
            target_contain = true;
            //------------------------------------------
            const Hand::frames_t  lasts_step_increment_thick = 10U;

            Hand::frames_t lasts_step_prev = lasts_step;
            lasts_step = lasts_step_increment_thick + 1U;
            //------------------------------------------
            for ( last_i = board.first;
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
              //------------------------------------------
            lasts_step = lasts_step_prev;
          } // if (target)
        } // end if (border)
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
      bool  has_braking = b_braking
        && boost::algorithm::any_of (arr_controlings,
                                     [](const Hand::Control &item)
                                     { return  item.last; });
      //----------------------------------------------
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
        { pd->predict (c, end); }
        //----------------------------------------------
        if ( b_distance )
        {
          end.x += 0.1;
          end.y -= 0.05;
        }
        //----------------------------------------------
        if ( b_checking )
        { stats.fill (hand, target, controling, end); }
      }
      //----------------------------------------------
      /* типо на мишени */
      if ( (!b_target || target.contain (end))
          && (!b_distance || boost_distance (center, end) < max_distance) )
      {
        HandMoves::trajectory_t  trajectory;
        //----------------------------------------------
        hand.SET_DEFAULT;
        /* двигаем рукой */
        hand.move (controling.begin (), controling.end (), &trajectory);
        hand_position = hand.position;
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
  void  LearnMovements::STAGE_1 (IN bool verbose)
  {
    borders_t  borders;
    defineBorders (borders, hand, 70U);
    DirectionPredictor  dp (hand);

    TourWorkSpace  tour (store, hand, target);
    tour.run (borders, dp, 
              /* distance */ true,
              /* target   */ false,
              /* braking  */ false,
              /* checking */ false,
              0.03, 3U,
              verbose);
  }
  /* Покрытие всей мишени не слишком плотно */
  void  LearnMovements::STAGE_2 (IN bool verbose)
  {
    borders_t  borders;
    defineBorders (borders, target, store, 0.005);
    DirectionPredictor  dp (hand);

    TourWorkSpace  tour (store, hand, target);
    tour.run (borders, dp,
              /* distance */ false,
              /* target   */ true,
              /* braking  */ true,
              /* checking */ false,
              0.01, 2U,
              verbose);
  }
  /* Попадание в оставшиеся непокрытыми точки мишени */
  void  LearnMovements::STAGE_3 (OUT HandMoves::trajectory_t &uncovered,
                                 OUT size_t &complexity, IN bool verbose)
  {
    size_t count = 0U;
    // -------------------------------------------------
    for ( const auto &pt : target.coords () )
    {
      visited.clear ();
      // -------------------------------------------------
      // Close (pt, NULL, NULL);

      int tries = 10;
      // -------------------------------------------------
      const Record &rec = store.ClothestPoint (pt, side);
      while ( tries > 0 && boost_distance (rec.hit, pt) > target.precision () )
      {
        deep = 0U;
        complexity += gradientMethod_admixture (pt, verbose);
        // -------------------------------------------------
        const Record &rec = store.ClothestPoint (pt, side);
        // if ( boost_distance (rec.hit, pt) > target.precision () )
        // { uncovered.push_back (pt); }
        --tries;
      }
      ++count;
    }
    // -------------------------------------------------
    tcout << _T ("TOTAL Complexity: ")   << complexity << std::endl;
    tcout << _T ("AVERAGE Complexity: ") << complexity / count << std::endl;
  }
  //------------------------------------------------------------------------------  
  void  LearnMovements::uncover (OUT HandMoves::trajectory_t &uncovered)
  {
    for ( const auto &pt : target.coords () )
    {
      const Record &rec = store.ClothestPoint (pt, side);
      // -------------------------------------------------------
      if ( boost_distance (rec.hit, pt) > target.precision () )
      { uncovered.push_back (pt); }
    }
  }
  //------------------------------------------------------------------------------  
};
