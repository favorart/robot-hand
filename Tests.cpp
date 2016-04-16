#include "StdAfx.h"
#include "HandMovesStore.h"

using namespace std;
using namespace HandMoves;
//------------------------------------------------------------------------------
bool  trajectories_concat (HandMoves::trajectories_t  &trajectories,
                           HandMoves::trajectory_t    &trajectory)
{
  // --------------------------------------------
  // ???
  auto back = boost_point2_t (trajectory.back ());
  for ( auto &t : trajectories )
  {
    auto fin = boost_point2_t (t.back ()); 
    if ( boost::geometry::distance (fin, back) < EPS )
      return false;
  }
  // --------------------------------------------
  // trajectories.splice (it, trajectory);
  trajectories.push_back (trajectory);
  return true;
}
//------------------------------------------------------------------------------
void  HandMoves::test_random (Store &store, Hand &hand, size_t tries)
{
  try
  {
    /* Для нового потока нужно снова переинициализировать rand */
    std::srand ((unsigned int) clock ());

    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;

    for ( size_t i = 0U; i < tries; ++i )
    {
      controling_t controls;

      Hand::MusclesEnum  muscle = Hand::EmptyMov;
      Hand::frames_t       last = 0U;
      Hand::frames_t      start = 0U;
      // trajectory_t      visited;

      size_t  moves_count = random (1, 2); // Record::maxControlsCount);
      for ( size_t j = 0U; j < moves_count; ++j )
      {
        muscle = hand.selectControl ();
        last = random (1U, hand.maxMuscleLast (muscle));
        
        controls.push_back (Hand::Control (muscle, start, last));
        start = last;
        // start /* actual_last */ = hand.move (muscle, last, visited);
        // hand.SET_DEFAULT;

        /*
           start_times[0] = 0U
           start_times[1] = random (1U, lasts[0])

           lasts[0] = random (1U, maxFrames_activeMuscles_correctedByCurrentPosition)
           lasts[1] = random (1U, maxFrames_activeMuscles_correctedByCurrentPosition)
        */

        // muscles[j] = muscle;
        // start_times[j] = (j) ? actual_last : 0;
        // lasts[j] = last;

        // // times_stop[j]  = (j) ? (times_stop[j - 1] + last) : last;
        boost::this_thread::interruption_point ();
      }

      trajectory_t visited;
      hand.move (controls.begin (), controls.end (), &visited);
      hand.SET_DEFAULT;

      const Point &hand_pos = hand.position;
      store.insert (Record (hand_pos, hand_pos_base,
                            hand_pos, controls,
                            visited) );

      boost::this_thread::interruption_point ();
    } // for tries
  }
  catch ( boost::thread_interrupted& )
  { /* tcout << _T("WorkingThread interrupted") << std::endl; */ }
}
void  HandMoves::test_cover  (Store &store, Hand &hand, size_t nesting)
{
  try
  {
    /* Create the tree of possible passes */
    for ( Hand::MusclesEnum muscle_i : hand.muscles_ )
    {
      hand.SET_DEFAULT;
      Point hand_base = hand.position;
      for ( Hand::frames_t last_i : boost::irange<Hand::frames_t> (1U, hand.maxMuscleLast (muscle_i)) )
      {
        std::list<Point> trajectory;

        hand.move (muscle_i, last_i, trajectory);
        store.insert (Record (hand.position, hand_base, hand.position,
                             { muscle_i }, { 0 }, { last_i },
                             1U, trajectory) );

        boost::this_thread::interruption_point ();

        if ( nesting > 1U )
          for ( Hand::MusclesEnum muscle_j : hand.muscles_ )
          {
            if ( (muscle_i == muscle_j) || !musclesValidUnion (muscle_i | muscle_j) )
              continue;

            for ( uint_t last_j = 1U; last_j < hand.maxMuscleLast (muscle_j); ++last_j )
            {
              std::list<Point>::iterator tail_j = trajectory.end ();
              --tail_j;

              hand.move (muscle_j, last_j, trajectory);
              store.insert (Record (hand.position, hand_base, hand.position,
                                   { muscle_i, muscle_j },
                                   { 0, last_i },
                                   { last_i, last_j },
                                   2U, trajectory) );

              boost::this_thread::interruption_point ();

              ++tail_j;
              //=================================================
              if ( nesting > 2U )
                for ( Hand::MusclesEnum muscle_k : hand.muscles_ )
                {
                  if ( (muscle_i == muscle_k || muscle_k == muscle_j)
                      || !musclesValidUnion (muscle_i | muscle_j | muscle_k) )
                    continue;

                  for ( auto last_k = 1U; last_k < hand.maxMuscleLast (muscle_k); ++last_k )
                  {
                    std::list<Point>::iterator tail_k = trajectory.end ();
                    --tail_k;

                    hand.move (muscle_k, last_k, trajectory);
                    store.insert (Record (hand.position, hand_base, hand.position,
                                         { muscle_i, muscle_j, muscle_k },
                                         { 0, last_i, last_i + last_j },
                                         { last_i, last_j, last_k },
                                         3U, trajectory) );

                    boost::this_thread::interruption_point ();

                    ++tail_k;

                    trajectory.erase (tail_k, trajectory.end ());
                    auto joint_k = jointByMuscle (muscle_k);
                    hand.set (Hand::JointsSet{ {joint_k, (joint_k == Hand::Elbow) ? 70. :
                                                         (joint_k == Hand::Wrist) ? 50. : 0.} });
                  }
                }
              //=================================================
              trajectory.erase (tail_j, trajectory.end ());
              auto joint_j = jointByMuscle (muscle_j);
              hand.set (Hand::JointsSet{ {joint_j, (joint_j == Hand::Elbow) ? 70. :
                                                   (joint_j == Hand::Wrist) ? 50. : 0.} });

              boost::this_thread::interruption_point ();
            }
          }
        auto joint_i = jointByMuscle (muscle_i);
        hand.set (Hand::JointsSet{ {joint_i, (joint_i == Hand::Elbow) ? 70. :
                                             (joint_i == Hand::Wrist) ? 50. : 0.} });

        boost::this_thread::interruption_point ();
      }
    }
    hand.SET_DEFAULT;
  }
  catch ( exception )
  { MessageBox (NULL, _T ("Error"), _T ("Error"), MB_OK); }
  catch ( boost::thread_interrupted& )
  { /* tcout << _T("WorkingThread interrupted" )<< std::endl; */ }
}
//------------------------------------------------------------------------------



