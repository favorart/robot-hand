#include "StdAfx.h"
#include "Position.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

namespace Positions
{
  using namespace std;
  using namespace HandMoves;
  //------------------------------------------------------------------------------
  class  NewTourWorkSpace /* Non-recursion */
  {
    tstring  config_file_name;
    bool end_ = false;
    
    Hand &hand;
    Store &store;
    RecTarget &target;
    DirectionPredictor dp;
    Point   hand_pos_base;
    
    boost::property_tree::ptree  config;

    // --- iterating ---
    HandMoves::controling_t     controls;

    borders_t  indices_borders; /* min--max */
    std::vector<Hand::Control>  indices;
    size_t max_nested;

    // --- braking ---
    // std::vector<Hand::Control>  arr_controlings;

    // --- flags ---
    bool  b_distance, b_target,
          b_braking, b_checking;
    //--------------------------------------------------------
    borders_t  borders_;
    //--------------------------------------------------------
   
    // --- distance --- 
    Point  pd_shift{ 0.15, -0.05 };
    double target_distance;

    //--------------------------------------------------------
    double                 step_distance;
    Hand::frames_t   lasts_step_increment;
    Hand::frames_t   lasts_step_increment_thick = 20U;
    Hand::frames_t   lasts_step_initiate = 25U;
    Hand::frames_t   lasts_step_braking = 5U;

    
  public:
    // NewTourWorkSpace () // : max_nested (hand.joints_.size ())
    // {}

    NewTourWorkSpace (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target,
                      IN const tstring &config_file_name) :
        store (store), hand (hand), target (target), dp (hand),
        max_nested (hand.joints_.size ()),
        // arr_controlings (max_nested),
        target_distance ((target.max ().x - target.min ().x) * 2.)
    {
      hand.SET_DEFAULT;
      hand_pos_base = hand.position;  

      // // Pick your poison
      // boost::property_tree::ini_parser::read_ini (config_file_name, config);
      // 
      // // for ( auto &child : config.get_child ("some_collection") )
      // // {
      // //   // Mutate children, perhaps.
      // // }
      // config.put ("metadata.modified", true);
      // 
      // // More poison
      // boost::property_tree::ini_parser::write_ini (config_file_name, config);
    }

    // NewTourWorkSpace (const tstring &config_file_name) :
    //   /* NewTourWorkSpace (), */ config_file_name (config_file_name)
    // {}
    // ~NewTourWorkSpace ()
    // {}
    // NewTourWorkSpace (borders_t borders) :
    //   max_nested (max_nested),
    //   indices_borders (borders),
    //   indices (max_nested)
    // {}
    // void Run (int max_nested)
    // {
    //   for ( ; !end (); ++(*this) )
    //   {
    //     // now (*this)[i] holds index of i-th loop
    //   }
    // }

    NewTourWorkSpace& operator++ ()
    {
      // increase first index i for which ind[i] < max_ind[i] 
      // set all indices j < i equal to min_ind[j] 
      // if no index found to increase, set end = true
      return *this;
    }
    NewTourWorkSpace& operator++ (int i)
    {
      // increase first index i for which ind[i] < max_ind[i] 
      // set all indices j < i equal to min_ind[j] 
      // if no index found to increase, set end = true
      return *this;
    }

    const Hand::Control&  operator[] (int i) const
    { return indices[i]; }

    bool  end () const { return end_; }
    
  public:
    //------------------------------------------------------------------------------
    void  run (IN bool distance, IN bool target, IN bool braking, IN bool checking,
               IN double step_distance, IN Hand::frames_t lasts_step_increment,
               IN bool verbose=false)
    {
      this->b_target = target;
      this->b_braking = braking;
      this->b_distance = distance;
      this->b_checking = checking;
      // ----------------------------------------------------
      this->step_distance = step_distance;
      this->lasts_step_increment = lasts_step_increment;
      // ----------------------------------------------------
      size_t  complexity = 0U;
      // ----------------------------------------------------
      try
      { runNestedForMuscle (NULL, 0U); }
      catch ( boost::thread_interrupted& )
      { /* tcout << _T("WorkingThread interrupted") << std::endl; */
        return;
      } // end catch
      // ----------------------------------------------------
      if ( verbose )
      { tcout << _T ("\nComplexity: ") << complexity << std::endl; }
    }
    //------------------------------------------------------------------------------

  private:
    //------------------------------------------------------------------------------
    bool  runNestedForMuscle (OUT Point &hand_pos_high, IN int joint_index)
    {
      bool  target_contain = true;
      //------------------------------------------
      Hand::frames_t start_i = 0U;
      // Hand::frames_t last_lasts = 0U;
      Hand::frames_t lasts_step; // = lasts_step_initiate;
      //------------------------------------------
      bool  was_on_target = false;
      //------------------------------------------
      Point hand_pos_curr = hand_pos_base,
            hand_pos_prev = hand_pos_base;
      //------------------------------------------
      int     high = 0;
      Point   max_hand_pos_high{ 0., 0. };
      //------------------------------------------
      controls.push_back (Hand::Control ());
      Hand::Control &control_i = controls.back ();
      //------------------------------------------
      for ( auto muscle_i : { muscleByJoint (hand.joints_[joint_index], true),
                              muscleByJoint (hand.joints_[joint_index], false) } )
      {
        control_i.muscle = muscle_i;
        //------------------------------------------
        auto &board = borders_[muscle_i];
        if (  board.first > 0
           && board.first < board.second )
        {
          lasts_step = lasts_step_increment;
          Hand::frames_t lasts_i_max = hand.maxMuscleLast (muscle_i);
          //------------------------------------------
          start_i = 0U;
          target_contain = true;
          //------------------------------------------
          Hand::frames_t last_i = 0U;
          for ( last_i  = board.first;
               (last_i <  lasts_i_max) && (target_contain || !was_on_target) // &&
           /* ((last_i <= board.second - start_i || target_contain)) */;
                last_i += lasts_step )
          {
            control_i.last = last_i;

            if ( (last_i > board.second - start_i) && ((!b_target || !target_contain) ||
                 hand_pos_prev.x > target.max ().x || hand_pos_prev.y < target.min ().y) )
            { break; }
            //------------------------------------------
            if ( 0 <= joint_index && (joint_index + 1) < max_nested )
            {
              //===============================================================
              target_contain = runNestedForMuscle (/*&*/hand_pos_curr, joint_index + 1) || !b_target;
              //===============================================================
            }
            else
            {
              // if ( b_braking )
              // { auto it = controls.begin ();
              //   for ( size_t ji = 0; ji <= joint_index; ++ji, ++it )
              //     if ( arr_controlings[ji].last )
              //       arr_controlings[ji].start = it->last + 1U;
              // }
              //===============================================================
              target_contain = runNestedInserting (hand_pos_curr) || !b_target;
              //===============================================================              
            } // end else (insert)

            //------------------------------------------
            if ( b_target && (target_contain && !was_on_target) )
            {
              board.first = last_i;
              was_on_target = true;
            }
            //------------------------------------------
            {
              max_hand_pos_high.x += hand_pos_curr.x;
              max_hand_pos_high.y += hand_pos_curr.y;
              ++high;
            }
            //------------------------------------------
            double  d = boost_distance (hand_pos_prev, hand_pos_curr);
            //------------------------------------------
                 if ( d > step_distance )
            {
              if ( lasts_step >= 2 * lasts_step_increment )
                lasts_step -= lasts_step_increment;
              else if ( lasts_step > lasts_step_increment )
                lasts_step -= lasts_step_increment;
              else if ( (lasts_step_increment > 1) && (lasts_step > lasts_step_increment / 2) )
                lasts_step -= lasts_step_increment / 2;
              // else if ( b_braking && (lasts_step_increment == 1U) )
              // {
              //   auto it = controls.begin ();
              //   for ( size_t ji = 0; ji <= joint_index; ++ji, ++it )
              //   {
              //     auto opposite_muscle = muscleOpposite (it->muscle);
              //     auto prev_last = arr_controlings[ji].last;
              //     arr_controlings[ji] = (Hand::Control (opposite_muscle, it->last + 1,
              //                                           (prev_last ? prev_last : 30) +
              //                                           lasts_step_braking));
              //   }
              // }
            }
            else if ( d < step_distance )
            {
              // if ( arr_controlings[0].last )
              // {
              //   if ( arr_controlings[0].last > lasts_step_braking )
              //     for ( size_t ji = 0; ji <= joint_index; ++ji )
              //       arr_controlings[ji].last -= lasts_step_braking;
              // }
              // else
              { lasts_step += lasts_step_increment; }
            }
            //------------------------------------------
            hand_pos_prev = hand_pos_curr;
            //------------------------------------------
          } // end for (last)
          //------------------------------------------
          if ( b_target ) { board.second = last_i; }
          //------------------------------------------
          // for ( size_t ji = 0; ji <= joint_index; ++ji )
          // { arr_controlings[ji].last = 0U; }
          //------------------------------------------
          if ( b_target )
          {
            start_i = 0U;
            target_contain = true;
            //------------------------------------------
            // Hand::frames_t lasts_step_prev = lasts_step;
            lasts_step = lasts_step_increment_thick; // lasts_step_initiate;
            //------------------------------------------
            for ( last_i = board.first;
                 (last_i - lasts_step) < lasts_i_max && target_contain;
                  last_i -= lasts_step )
            {
              control_i.last = last_i;
              //------------------------------------------
              if ( 0 <= joint_index && (joint_index + 1) < max_nested )
              {
                //===============================================================
                target_contain = runNestedForMuscle (/* & */hand_pos_curr, joint_index + 1) || !b_target;
                //===============================================================
              }
              else
              {
                //===============================================================
                target_contain = runNestedInserting (hand_pos_curr) || !b_target;
                //===============================================================
              } // end (insert)
              //------------------------------------------
              double  d = boost_distance (hand_pos_prev, hand_pos_curr);
              //------------------------------------------
                   if ( d > step_distance )
              {
                if ( lasts_step > lasts_step_increment_thick )
                  lasts_step -= lasts_step_increment_thick;
              }
              else if ( d < step_distance )
              { lasts_step += lasts_step_increment_thick; }
              //-----------------------------
              hand_pos_prev = hand_pos_curr;
            } // end for (lasts)
            //------------------------------------------
            // lasts_step = lasts_step_prev;
          } // if (target)
        } // end if (border)
      } // end for (muscle)
      //------------------------------------------
      controls.pop_back ();
      //------------------------------------------
      if ( /*hand_pos_high &&*/ high )
      { hand_pos_high =  Point{ max_hand_pos_high.x / high,
                                max_hand_pos_high.y / high };
      }
      //------------------------------------------
      return true;
    }
    bool  runNestedInserting (OUT Point &hand_position)
    {
      // const Record &rec = store.ClothestPoint (hand_position, 0.1);
      // if ( boost_distance (rec.hit, hand_position) < step_distance )
      // {
      //   hand_position = rec.hit;
      //   return target.contain (hand_position);
      // }

      bool  target_contain = true;
      Point end = hand_pos_base;
      //----------------------------------------------
      // bool  has_braking = b_braking
      //   && boost::algorithm::any_of (arr_controlings,
      //                                [](const Hand::Control &item)
      //                                { return  item.last; });
      //----------------------------------------------
      // controling_t  new_controling;
      // if ( has_braking )
      // {
      //   for ( const Hand::Control &c : controls )
      //   { new_controling.push_back (c); }
      //   /* записываем все разрывы */
      //   for ( auto &c : arr_controlings )
      //   { new_controling.push_back (c); }
      //   new_controling.sort ();
      // }
      // controling_t  &controling = (has_braking) ? new_controling : controls;
      controling_t  &controling = controls;
      //----------------------------------------------
      if ( b_target || b_distance )
      {
        for ( auto &c : controls )
        { dp.predict (c, end); }
        //----------------------------------------------
        // if ( b_distance && b_target )
        // { end.x += predict_shift.x;
        //   end.y += predict_shift.y;
        // }
      }
      //----------------------------------------------
      if ( (!b_target   || target.contain (end))
        && (!b_distance || boost_distance (target.center (), end) < target_distance) )
      {
        /* ближе к мишени */
        HandMoves::trajectory_t  trajectory;
        //----------------------------------------------
        hand.SET_DEFAULT;
        /* двигаем рукой */
        hand.move (controling.begin (), controling.end (), &trajectory);
        // ++complexity; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //----------------------------------------------
        hand_position  = hand.position;
        target_contain = target.contain (hand_position);
        //----------------------------------------------
        HandMoves::Record  rec (hand_position, hand_pos_base,
                                hand_position, controling,
                                trajectory);
        store.insert (rec);
        //----------------------------------------------
      } /* end if */
      else
      { 
        if ( !b_target )
        { hand_position = end; }
        target_contain = false;
      }
      //----------------------------------------------
      boost::this_thread::interruption_point ();
      //----------------------------------------------
      return  target_contain;
    }

    bool  runNestedForMuscle (OUT std::vector<Point> *hand_pos_highs, IN int joint_index)
    {
      bool  target_contain = true;
      //------------------------------------------
      Hand::frames_t start_i = 0U;
      // Hand::frames_t last_lasts = 0U;
      Hand::frames_t lasts_step; // = lasts_step_initiate;
      //------------------------------------------
      bool  was_on_target = false;
      //------------------------------------------
      Point hand_pos_curr = hand_pos_base,
            hand_pos_prev = hand_pos_base;
      //------------------------------------------
      // int     high = 0;
      // Point   max_hand_pos_high{ 0., 0. };
      std::vector<Point>  hand_pos_currs;
      std::vector<Point>  hand_pos_prevs;

      auto m = muscleByJoint (hand.joints_[joint_index], true);
      hand_pos_currs.reserve (2 * hand.maxMuscleLast (m));
      hand_pos_prevs.reserve (2 * hand.maxMuscleLast (m));
      //------------------------------------------
      controls.push_back (Hand::Control ());
      Hand::Control &control_i = controls.back ();
      //------------------------------------------
      for ( auto muscle_i : { muscleByJoint (hand.joints_[joint_index], true),
                              muscleByJoint (hand.joints_[joint_index], false) } )
      {
        control_i.muscle = muscle_i;
        //------------------------------------------
        auto &board = borders_[muscle_i];
        if ( board.first > 0
            && board.first < board.second )
        {
          lasts_step = lasts_step_increment;
          Hand::frames_t lasts_i_max = hand.maxMuscleLast (muscle_i);
          //------------------------------------------
          start_i = 0U;
          target_contain = true;
          //------------------------------------------
          Hand::frames_t last_i = 0U;
          for ( last_i = board.first;
          (last_i <  lasts_i_max) && (target_contain || !was_on_target) // &&
            /* ((last_i <= board.second - start_i || target_contain)) */;
            last_i += lasts_step )
          {
            control_i.last = last_i;

            if ( (last_i > board.second - start_i) && ((!b_target || !target_contain) ||
                hand_pos_prev.x > target.max ().x || hand_pos_prev.y < target.min ().y) )
            { break; }
            //------------------------------------------
            double  d;
            //------------------------------------------
            if ( 0 <= joint_index && (joint_index + 1) < max_nested )
            {
              //===============================================================
              target_contain = runNestedForMuscle (&hand_pos_currs, joint_index + 1) || !b_target;
              //===============================================================
              if ( !hand_pos_prevs.empty () )
              {
                d = 0.;
                size_t count = 0U;

                auto itc = hand_pos_currs.begin ();
                auto itp = hand_pos_prevs.begin ();
                for ( ; itc != hand_pos_currs.end ()
                     && itp != hand_pos_prevs.end (); ++itc, ++itp )
                {
                  d += boost_distance (*itc, *itp);
                  ++count;
                }
                d /= (count);

                // d *= 2.8;
              }
              else
              { d = step_distance; }

              std::swap (hand_pos_currs, hand_pos_prevs);
              hand_pos_currs.clear ();
            }
            else
            {
              // if ( b_braking )
              // {
              //   auto it = controls.begin ();
              //   for ( size_t ji = 0; ji <= joint_index; ++ji, ++it )
              //     if ( arr_controlings[ji].last )
              //       arr_controlings[ji].start = it->last + 1U;
              // }
              //===============================================================
              target_contain = runNestedInserting (hand_pos_curr) || !b_target;
              //===============================================================  
              if ( hand_pos_highs )
              { hand_pos_highs->push_back (hand_pos_curr); }

              d = boost_distance (hand_pos_prev, hand_pos_curr);
            } // end else (insert)

              //------------------------------------------
            if ( b_target && (target_contain && !was_on_target) )
            {
              board.first = last_i;
              was_on_target = true;
            }
            //------------------------------------------
            // {
            //   max_hand_pos_high.x += hand_pos_curr.x;
            //   max_hand_pos_high.y += hand_pos_curr.y;
            //   ++high;
            // }

            //------------------------------------------
            // double  d = boost_distance (hand_pos_prev, hand_pos_curr);
            //------------------------------------------
            if ( d > step_distance )
            {
              if ( lasts_step >= 2 * lasts_step_increment )
                lasts_step -= lasts_step_increment;
              else if ( lasts_step > lasts_step_increment )
                lasts_step -= lasts_step_increment;
              else if ( (lasts_step_increment > 1) && (lasts_step > lasts_step_increment / 2) )
                lasts_step -= lasts_step_increment / 2;
              // else if ( b_braking )
              // {
              //   auto it = controls.begin ();
              //   for ( size_t ji = 0; ji <= joint_index; ++ji, ++it )
              //   {
              //     auto opposite_muscle = muscleOpposite (it->muscle);
              //     auto prev_last = arr_controlings[ji].last;
              //     arr_controlings[ji] = (Hand::Control (opposite_muscle, it->last + 1,
              //                                           (prev_last ? prev_last : 30) +
              //                                           lasts_step_braking));
              //   }
              // }
            }
            else if ( d < step_distance )
            {
              // if ( arr_controlings[0].last )
              // {
              //   if ( arr_controlings[0].last > lasts_step_braking )
              //     for ( size_t ji = 0; ji <= joint_index; ++ji )
              //       arr_controlings[ji].last -= lasts_step_braking;
              // }
              // else if ( d < 10 * step_distance )
              // { lasts_step += 10 * lasts_step_increment; }
              // else
              { lasts_step += lasts_step_increment; }
            }
            //------------------------------------------
            hand_pos_prev = hand_pos_curr;
            //------------------------------------------
          } // end for (last)
            //------------------------------------------
          if ( b_target ) { board.second = last_i; }
          //------------------------------------------
          // for ( size_t ji = 0; ji <= joint_index; ++ji )
          // { arr_controlings[ji].last = 0U; }
          //------------------------------------------
          if ( 0 ) // b_target ) // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          {
            start_i = 0U;
            target_contain = true;
            //------------------------------------------
            // Hand::frames_t lasts_step_prev = lasts_step;
            lasts_step = lasts_step_increment_thick; // lasts_step_initiate;
                                                     //------------------------------------------
            for ( last_i = board.first;
            (last_i - lasts_step) < lasts_i_max && target_contain;
              last_i -= lasts_step )
            {
              control_i.last = last_i;
              //------------------------------------------
              if ( 0 <= joint_index && (joint_index + 1) < max_nested )
              {
                //===============================================================
                // target_contain = runNestedForMuscle (&hand_pos_curr, joint_index + 1) || !b_target;
                //===============================================================
              }
              else
              {
                //===============================================================
                target_contain = runNestedInserting (hand_pos_curr) || !b_target;
                //===============================================================
              } // end (insert)
                //------------------------------------------
              double  d = boost_distance (hand_pos_prev, hand_pos_curr);
              //------------------------------------------
              if ( d > step_distance )
              {
                if ( lasts_step > lasts_step_increment_thick )
                  lasts_step -= lasts_step_increment_thick;
              }
              else if ( d < step_distance )
              { lasts_step += lasts_step_increment_thick; }
              //-----------------------------
              hand_pos_prev = hand_pos_curr;
            } // end for (lasts)
              //------------------------------------------
              // lasts_step = lasts_step_prev;
          } // if (target)
        } // end if (border)
      } // end for (muscle)
        //------------------------------------------
      controls.pop_back ();
      //------------------------------------------
      // if ( hand_pos_high && high )
      // {
      //   *hand_pos_high = Point{ max_hand_pos_high.x / high,
      //     max_hand_pos_high.y / high };
      // }
      //------------------------------------------
      return true;
    }

    //------------------------------------------------------------------------------
  public:
    /* грубое покрытие всего рабочего пространства */
    void  STAGE_1 (IN bool verbose)
    {
      defineBorders (indices_borders, hand, 70U);

      run (/* distance */  true,
           /* target   */ false,
           /* braking  */  true,
           /* checking */ false,
           0.05, 3U,
           verbose);
    }
    /* Покрытие всей мишени не слишком плотно */
    void  STAGE_2 (IN bool verbose)
    {
      defineBorders (indices_borders, target, store, /* side */ 0.05);

      run (/* distance */ false,
           /* target   */ true,
           /* braking  */ true,
           /* checking */ false,
           0.015, 2U,
           verbose);
    }
  };
  //------------------------------------------------------------------------------
};
