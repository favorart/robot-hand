#include "StdAfx.h"

#ifndef  _RECORD_H_
#define  _RECORD_H_
//------------------------------------------------------------------------------
#define HAND_VER 2
#if   HAND_VER == 1
#include "Hand.h"
using namespace OldHand;
#elif HAND_VER == 2
#include "Hand.h"
using namespace NewHand;
#include "HandMuscles.h"
#endif
//------------------------------------------------------------------------------
namespace HandMoves
{
  //------------------------------------------------------------------------------
  typedef std::list<Hand::Control>             controling_t;
  typedef std::list<Point>                     trajectory_t;
  typedef std::list<trajectory_t>            trajectories_t;
  typedef std::list<std::shared_ptr<Point>>  trajectory_refs_t;
  //------------------------------------------------------------------------------
  class Record
  {
  public:
    // ----------------------------------------
    typedef std::vector<Hand::frames_t>       times_array;
    typedef std::vector<Hand::MusclesEnum>  muscles_array;

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
    struct ChangeHitPoint : public std::unary_function<Record, void>
    {
      ChangeHitPoint (const Point &hit) : hit_ (hit) {}
      void operator() (Record rec) { rec.hand_final_ = hit_; }
    private:
      Point hit_;
    };

    double  hit_x () const { return hand_final_.x; }
    double  hit_y () const { return hand_final_.y; }

    double  aim_x () const { return aim_.x; }
    double  aim_y () const { return aim_.y; }

    // ----------------------------------------
    Record () {}

    Record (IN const Point         &aim,
            IN const Point         &hand_begin,
            IN const Point         &hand_final,
            IN const muscles_array &muscles,
            IN const times_array   &times,
            IN const times_array   &lasts,
            IN size_t               controls_count,
            IN const trajectory_t  &visited);

    Record (IN const Point         &aim,
            IN const Point         &hand_begin,
            IN const Point         &hand_final,
            IN const controling_t  &controls,
            IN const trajectory_t  &visited);

    // ----------------------------------------
    operator tstring () const
    {
      tstringstream ss;
      ss << _T ("rec<x=") << hit.x << _T (", y=") << hit.y << _T (">");
      return  ss.str ();
    }
    // ----------------------------------------
    bool  operator== (const Record &rec) const
    {
      if ( this != &rec )
      { return   boost::range::equal (hand_controls_, rec.hand_controls_); }
      return true;
    }
    bool  operator!= (const Record &rec) const
    {
      if (this != &rec)
      { return  !boost::range::equal (hand_controls_, rec.hand_controls_); }
      return false;
    }
    // ----------------------------------------
    /* Microsoft specific: C++ properties */
    __declspec(property(get = _get_aim))      const Point&         aim;
    __declspec(property(get = _get_hit))      const Point&         hit;
    __declspec(property(get = _get_traj))     const trajectory_t&  trajectory;
    __declspec(property(get = _get_muscles))  Hand::MusclesEnum    muscles;
    __declspec(property(get = _get_controls)) const controling_t&  controls;
    __declspec(property(get = _get_n_ctrls))  size_t             n_controls;

    // ----------------------------------------
    const Point&         _get_aim () const { return aim_; }
    const Point&         _get_hit () const { return hand_final_; }
    const trajectory_t&  _get_traj () const { return visited_; }
    Hand::MusclesEnum    _get_muscles () const { return  muscles_; }
    size_t               _get_n_ctrls () const { return hand_controls_.size (); }
    const controling_t&  _get_controls () const { return hand_controls_; }
    // ----------------------------------------
    size_t  controls_copy (OUT controling_t &controls) const
    {
      boost::range::copy (hand_controls_, std::back_inserter (controls));
      return n_controls;
    }

    // ----------------------------------------
    double  distanceCovered () const
    { return boost_distance (hand_final_, hand_begin_); }
    double  eleganceMove    () const;

    bool    validateMusclesTimes () const;
    // ----------------------------------------
    double  ratioDistanceByTrajectory () const;
    double  ratioTrajectoryDivirgence () const;

    double  ratioUsedMusclesCount () const;
    double  ratioTrajectoryBrakes () const;
    
    Hand::frames_t  longestMusclesControl () const;
    // ----------------------------------------
  };
  //------------------------------------------------------------------------------
  struct RecordHasher
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
  tostream&  operator << (tostream &out, const HandMoves::controling_t  &controls);
//------------------------------------------------------------------------------
}
BOOST_CLASS_VERSION (HandMoves::Record, 2)
//------------------------------------------------------------------------------
#endif // _RECORD_H_
