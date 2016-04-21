#include "StdAfx.h"
#include "Store.h"

using namespace std;
using namespace HandMoves;
//------------------------------------------------------------------------------
void  HandMoves::test_random (Store &store, Hand &hand, size_t tries)
{
  const Hand::frames_t last_min = 50U;

  try
  {
    /* Для нового потока нужно снова переинициализировать rand */
    std::srand ((unsigned int) clock ());

    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;

    for ( size_t i = 0U; i < tries; ++i )
    {
      controling_t  controls;

      Hand::MusclesEnum  muscle = Hand::EmptyMov;
      Hand::frames_t       last = 0U;
      Hand::frames_t      start = 0U;

      size_t  moves_count = random (1, 3); // Record::maxControlsCount);
      for ( size_t j = 0U; j < moves_count; ++j )
      {
        muscle = hand.selectControl ();
        if ( !last )
        { last = random (last_min, hand.maxMuscleLast (muscle) / 2U); }
        else
        { last = random (last_min, last); }

        controls.push_back (Hand::Control (muscle, start, last));
        start += (last + 1U);
      }
      
      trajectory_t  trajectory;
      hand.move (controls.begin (), controls.end (), &trajectory);

      const Point&  hand_position = hand.position;
      store.insert (Record (hand_position, hand_pos_base, hand_position,
                            controls, trajectory));
      hand.SET_DEFAULT;

      boost::this_thread::interruption_point ();
    } // for tries
  }
  catch ( boost::thread_interrupted& )
  { /* tcout << _T("WorkingThread interrupted") << std::endl; */ }
}
void  HandMoves::test_cover  (Store &store, Hand &hand, size_t nesting)
{
  const Hand::frames_t last_min  = 50U;
  const Hand::frames_t last_step = 10U;

  try
  {
    hand.SET_DEFAULT;
    Point hand_base = hand.position;

    /* Create the tree of possible passes */
    for ( Hand::MusclesEnum muscle_i : hand.muscles_ )
    {

      for ( Hand::frames_t last_i = last_min; last_i < hand.maxMuscleLast (muscle_i); last_i += last_step )
      {
        trajectory_t  trajectory;

        hand.move (muscle_i, last_i, trajectory);
        store.insert (Record (hand.position, hand_base, hand.position,
                              { muscle_i }, { 0 }, { last_i },
                              1U, trajectory) );

        //=================================================
        if ( nesting == 1U )
        { continue; }
        //=================================================
        for ( Hand::MusclesEnum muscle_j : hand.muscles_ )
          {
            if ( (muscle_i == muscle_j) || !musclesValidUnion (muscle_i | muscle_j) )
              continue;

            for ( Hand::frames_t last_j = last_min; last_j < hand.maxMuscleLast (muscle_j); last_j += last_step )
            {
              trajectory_t::iterator tail_j = trajectory.end ();
              --tail_j;

              hand.move (muscle_j, last_j, trajectory);
              store.insert (Record (hand.position, hand_base, hand.position,
                                   { muscle_i, muscle_j },
                                   { 0, last_i },
                                   { last_i, last_j },
                                   2U, trajectory) );

              ++tail_j;
              //=================================================
              if ( nesting == 2U )
              { continue; }
              //=================================================
              for ( Hand::MusclesEnum muscle_k : hand.muscles_ )
              {
                if ( (muscle_i == muscle_k || muscle_k == muscle_j)
                  || !musclesValidUnion (muscle_i | muscle_j | muscle_k) )
                { continue; }

                for ( Hand::frames_t last_k = last_min; last_k < hand.maxMuscleLast (muscle_k); last_k += last_step )
                {
                  trajectory_t::iterator  tail_k = trajectory.end ();
                  --tail_k;

                  hand.move (muscle_k, last_k, trajectory);
                  store.insert (Record (hand.position, hand_base, hand.position,
                                        { muscle_i, muscle_j, muscle_k },
                                        { 0, last_i, last_i + last_j },
                                        { last_i, last_j, last_k },
                                        3U, trajectory));

                  ++tail_k;

                  trajectory.erase (tail_k, trajectory.end ());
                  auto joint_k = jointByMuscle (muscle_k);
                  hand.set (Hand::JointsSet{ {joint_k, (joint_k == Hand::Elbow) ? 70. :
                                                       (joint_k == Hand::Wrist) ? 50. : 0.} });

                  boost::this_thread::interruption_point ();
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
  catch ( boost::thread_interrupted& )
  { /* tcout << _T("WorkingThread interrupted" )<< std::endl; */ }
  catch ( ... )
  { MessageBox (NULL, _T ("Error"), _T ("Error"), MB_OK); }
}
//------------------------------------------------------------------------------



