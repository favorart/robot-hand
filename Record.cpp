#include "StdAfx.h"
#include "HandMovesStore.h"

using namespace std;
using namespace HandMoves;
//---------------------------------------------------------
Record::Record (const Point         &aim,
                const Point         &hand_begin,
                const Point         &hand_final,
                const muscles_array &muscles,
                const times_array   &times,
                const times_array   &lasts,
                size_t               controls_count,
                const trajectory_t  &visited) :
  aim_ (aim), hand_begin_ (hand_begin), hand_final_ (hand_final),
  muscles_ (Hand::EmptyMov), visited_ (visited)
{
  if ( controls_count > maxControlsCount )
    throw new exception ("Incorrect number of muscles in constructor Record"); // _T( ??

  auto time_first = times[0];
  for ( auto i = 0U; i < controls_count; ++i )
  {
    muscles_ = muscles_ | muscles[i];
    hand_controls_.push_back (HandControl (muscles[i],
                                           static_cast<time_t>(times[i] - time_first),
                                           static_cast<time_t>(lasts[i])));
  }

  if ( !validateMusclesTimes () )
    throw new std::exception ("Invalid muscles constructor Record parameter"); // _T( ??

  // elegance_ = eleganceMove (aim_);
  // distance_ = boost_distance (hand_, aim_);
}

Record::Record (const Point         &aim,
                const Point         &hand_begin,
                const Point         &hand_final,
                const std::initializer_list<HandControl> controls,
                const trajectory_t  &visited) :
  aim_ (aim), hand_begin_ (hand_begin), hand_final_ (hand_final),
  muscles_ (Hand::EmptyMov), visited_ (visited)
{
  if ( controls.size () > maxControlsCount )
    throw new exception ("Incorrect number of muscles in constructor Record"); // _T( ??

  auto time_first = controls.begin ()->time;
  for ( auto hc : controls )
  {
    muscles_ = muscles_ | hc.muscle;
    hand_controls_.push_back (HandControl        (hc.muscle,
                              static_cast<time_t>(hc.time - time_first),
                              static_cast<time_t>(hc.last)));
  }

  if ( !validateMusclesTimes () )
    throw new std::exception ("Invalid muscles constructor Record parameter"); // _T( ??

  // elegance_ = eleganceMove   (aim_);
  // distance_ = boost_distance (aim_, hand_begin_);
}
//---------------------------------------------------------
bool    Record::validateMusclesTimes () const
{
  // if ( times_.size () > 1U )
  if ( hand_controls_.size () > 1U )
  {
    /* Каждый с каждым - n^2 !!! TODO !!!  */

    // for ( muscle_times_t::iterator iti = times_.begin (); iti != times_.end (); ++iti )
    //   for ( muscle_times_t::iterator itj = std::next (iti); itj != times_.end (); ++itj )
    for ( auto iti = hand_controls_.begin (); iti != hand_controls_.end (); ++iti )
      for ( auto itj = std::next (iti); itj != hand_controls_.end (); ++itj )
        /* Если есть перекрытие по времени */
        // if ( ((iti->second.first <= itj->second.first) && (iti->second.second >= itj->second.first))
        //   || ((itj->second.first <= iti->second.first) && (itj->second.second >= iti->second.first)) )
        if ( ((iti->time <= itj->time) && ((iti->time + iti->last) >= itj->time))
            || ((itj->time <= iti->time) && ((itj->time + itj->last) >= iti->time)) )
        {
          for ( auto j : joints )
          {
            Hand::MusclesEnum  Opn = muscleByJoint (j, true);
            Hand::MusclesEnum  Сls = muscleByJoint (j, false);
            /* Одновременно работающие противоположные мышцы */
            if ( (Opn & iti->muscle) && (Сls & itj->muscle) )
            { return false; } // end if
          } // end for
        } // end if
  } //end if

    // if ( (Opn & muscles_) && (Сls & muscles_) )
    // {
    //   if ( ((times_[Opn].first <= times_[Сls].first) && (times_[Opn].second >= times_[Сls].first))
    //     || ((times_[Сls].first <= times_[Opn].first) && (times_[Сls].second >= times_[Opn].first)) )
    //     return false;
    // }

  return true;
}
//---------------------------------------------------------
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/linestring.hpp>
double  Record::eleganceMove (/* const Point &aim */) const
{
  if ( 0 )
  {
    /* Количество движений */
    double controls_count_ratio = 1. / controlsCount;

    /* Количество задействованных мышц */
    size_t muscles_count = 0U;
    for ( auto i : boost::irange<size_t> (0U, Hand::MusclesCount) )
      if ( muscles_ & (1 << i) )
        ++muscles_count;
    double muscles_count_ratio = 1. / muscles_count;

    /* Время работы двигателей */
    auto sum_time = 0.;
    for ( auto hc : hand_controls_ )
    { sum_time += hc.last; }

    /* max.отклонение */
    typedef boost::geometry::model::d2::point_xy<double> bpt;
    boost::geometry::model::linestring<bpt>  line;

    auto start = visited_.front ();
    line.push_back (bpt (start.x, start.y));
    line.push_back (bpt (aim.x, aim.y));

    double max_divirgence = 0.;
    for ( auto pt : visited_ )
    {
      double dist = boost::geometry::distance (bpt (pt.x, pt.y), line);
      if ( dist > max_divirgence )
        max_divirgence = dist;
    }
  }

  /* Длина траектории по сравнениею с дистанцией */
  double visited_disance = 0.;
  for ( auto curr = visited_.begin (), next = std::next (curr); next != visited_.end (); ++next )
  { visited_disance += boost_distance (*curr, *next); }
  double  distance_ratio = (visited_disance) ? (boost_distance (aim_, hand_begin_) / visited_disance) : 0.;
  
  return  distance_ratio; /* + max_divirgence + sum_time + muscles_count_ratio + controls_count_ratio */
}
//---------------------------------------------------------
void    Record::repeatMove (Hand &hand) const
{
  hand.reset ();
  hand.SET_DEFAULT;

  // std::list<Point> visited;
  for ( auto mp : hand_controls_ )
  { hand.move (mp.muscle, mp.last); // , visited);
  }
}
//---------------------------------------------------------
