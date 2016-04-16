#include "StdAfx.h"
#include "Position.h"

namespace Positions
{
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

      hand.set (Hand::JointsSet{ { j, 100. } });
      muscle = muscleByJoint (j, true);
      main_directions[muscle] = MainDirection (muscle, hand);

      hand.set (Hand::JointsSet{ { j, 0. } });
      muscle = muscleByJoint (j, false);
      main_directions[muscle] = MainDirection (muscle, hand);

      hand.SET_DEFAULT;
    }
    // for ( auto m : hand.muscles_ )
    // { mainDirections[m] = MainDirection (m, hand); }
  }
  //------------------------------------------
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
  //------------------------------------------
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
  //------------------------------------------
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

          indices[ij] = -(int) inx;
        } // end if
      } // end for
    } // end while

    for ( size_t ij = hand.joints_.size (); ij > 0; --ij )
    {
      auto mo = muscleByJoint (hand.joints_[ij - 1U], true);
      auto mc = muscleByJoint (hand.joints_[ij - 1U], false);

      if ( indices[ij - 1U] > 0 )
        controls.push_back (Hand::Control (mo, 0U, size_t (indices[ij - 1U]) /* - stopping ? */));
      else if ( indices[ij - 1U] < 0 )
        controls.push_back (Hand::Control (mc, 0U, size_t (-indices[ij - 1U]) /* - stopping ? */));
    }
  }
  //------------------------------------------
};
