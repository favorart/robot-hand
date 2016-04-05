#include "StdAfx.h"

#ifndef  _RECORD_H_
#define  _RECORD_H_
//------------------------------------------------------------------------------
#define HAND_VER 2
#if   HAND_VER == 1
#include "Hand.h"
using namespace OldHand;
#elif HAND_VER == 2
#include "NewHand.h"
using namespace NewHand;
#include "HandMuscles.h"
#endif
//------------------------------------------------------------------------------
namespace HandMoves
{
  // !!! tree types of point !!!
  //-------------------------------
  // Point  aim_;
  // Point  hand_;
  // Point  close_network_;
  //------------------------------------------------------------------------------
  typedef std::list<Point>          trajectory_t;
  typedef std::list<trajectory_t>   trajectories_t;
  typedef std::list<Hand::Control>  controling_t;
  //------------------------------------------------------------------------------
  class Record
  {
  public:
    const static size_t  maxControlsCount = 50U; // !!!!!!!!!!!!!

    const static size_t  arrays_size = 4U;
    typedef std::array<Hand::frames_t, arrays_size> times_array;
    typedef std::array<Hand::MusclesEnum, arrays_size> muscles_array;

  private:
    // ----------------------------------------
    Point  aim_, hand_begin_, hand_final_;
    // ----------------------------------------
    Hand::MusclesEnum   muscles_;
    trajectory_t        visited_;
    controling_t  hand_controls_;

    // ----------------------------------------
    friend class boost::serialization::access;
    BOOST_SERIALIZATION_SPLIT_MEMBER ()

    template <class Archive>
    void  save (Archive & ar, const unsigned int version) const
    {
      ar << aim_ << hand_begin_ << hand_final_;
      ar << muscles_ << hand_controls_ << visited_;
    }
    template <class Archive>
    void  load (Archive & ar, const unsigned int version)
    {
      ar >> aim_ >> hand_begin_ >> hand_final_;
      ar >> muscles_ >> hand_controls_ >> visited_;
    }

  public:
    // ----------------------------------------
    struct ChangeAimPoint : public std::unary_function<Record, void>
    {
      ChangeAimPoint (const Point &aim) : aim_ (aim) {}
      void operator() (Record rec) { rec.aim_ = aim_; }
    private:
      Point aim_;
    };

    double hit_x () const { return hand_final_.x; }
    double hit_y () const { return hand_final_.y; }

    double aim_x () const { return aim_.x; }
    double aim_y () const { return aim_.y; }

    Hand::MusclesEnum  muscles () const
    { return muscles_; }
    // ----------------------------------------
    Record () {}

    Record (const Point         &aim,
            const Point         &hand_begin,
            const Point         &hand_final,
            const muscles_array &muscles,
            const times_array   &times,
            const times_array   &lasts,
            size_t               controls_count,
            const trajectory_t  &visited);

    Record (const Point         &aim,
            const Point         &hand_begin,
            const Point         &hand_final,
            const controling_t  &controls,
            const trajectory_t  &visited);

    // ----------------------------------------
    operator tstring () const
    { return  str (boost::wformat (_T ("rec<x=%1%, y=%2%>")) % hit.x % hit.y); }

    // ----------------------------------------
    /* Microsoft specific: C++ properties */
    __declspec(property(get = get_aim)) const Point &aim;
    const Point&  get_aim () const { return aim_; }

    __declspec(property(get = get_final)) const Point &hit;
    const Point&  get_final () const { return hand_final_; }

    __declspec(property(get = get_trajectory)) const trajectory_t &trajectory;
    const trajectory_t  &get_trajectory () const { return visited_; }

    __declspec(property(get = get_controls_count)) size_t  controlsCount;
    size_t  get_controls_count () const { return hand_controls_.size (); }

    size_t   controls_array (OUT std::array<Hand::Control,Record::maxControlsCount> &controls) const
    {
      size_t i = 0U;
      for ( auto &hc : hand_controls_ )
      { controls[i++] = hc; }
      return controlsCount;
    }
    const controling_t&  controls () const { return hand_controls_; }
    // ----------------------------------------
    bool    validateMusclesTimes () const;
    void    repeatMove (Hand &hand) const
    {
      hand.SET_DEFAULT;
      trajectory_t visited;
      hand.move (hand_controls_.begin (), hand_controls_.end (), &visited);
      
      auto it1 = visited.begin ();
      auto it2 = visited_.begin ();
      auto index = 0U;
      for ( ; it1 != visited.end () && it2 != visited_.end (); ++it1, ++it2 )
      {
        if ( *it1 != *it2 )
        {
          auto  &j1 = *it1;
          auto  &j2 = *it2;
        }
        ++index;
      }
    }

    double  eleganceMove (/* const Point &aim */) const;
    double  distanceCovered () const
    { return boost_distance (hand_final_, hand_begin_); }
    // ----------------------------------------
    double  ratioDistanceByTrajectory () const;
    double  ratioTrajectoryDivirgence () const;

    double  ratioUsedMusclesCount () const;
    double  ratioTrajectoryBrakes () const;
    
    Hand::frames_t  longestMusclesControl () const;
    // ----------------------------------------
  };
  //------------------------------------------------------------------------------
  class RecordHasher
  {
    std::size_t operator()(const Record& rec) const
    {
      std::size_t  seed = 0U;
      // modify seed by xor and bit-shifting
      // of the key members
      boost::hash_combine (seed, boost::hash_value (rec.hit.x));
      boost::hash_combine (seed, boost::hash_value (rec.hit.y));
      // the result.
      return seed;
    }
  };
  //------------------------------------------------------------------------------
}
BOOST_CLASS_VERSION (HandMoves::Record, 1)
//------------------------------------------------------------------------------
#endif // _RECORD_H_
