#pragma once

#include "StdAfx.h"
#include "Robo.h"

#ifndef  _ROBO_MOVES_RECORD_H_
#define  _ROBO_MOVES_RECORD_H_
//------------------------------------------------------------------------------
namespace RoboMoves
{

typedef std::vector<Robo::frames_t>   frames_array;
typedef std::vector<Robo::muscle_t>  muscles_array;

class Record
{
    Point  aim_, move_begin_, move_final_;
    //Robo::muscle_t muscles_;
    Robo::Control    control_;
    Robo::Trajectory visited_;

    // ----------------------------------------
    friend class boost::serialization::access;
    BOOST_SERIALIZATION_SPLIT_MEMBER()

    template <class Archive>
    void  save(Archive & ar, const unsigned int version) const
    {
        ar << aim_ << move_begin_ << move_final_;
        ar << /*muscles_ <<*/ control_ << visited_;
    }
    template <class Archive>
    void  load(Archive & ar, const unsigned int version)
    {
        ar >> aim_ >> move_begin_ >> move_final_;
        ar >> /*muscles_ >>*/ control_ >> visited_;
    }

public:
    // ----------------------------------------
    struct ChangeAimPoint : public std::unary_function<Record, void>
    {
        ChangeAimPoint(const Point &aim) : aim_(aim) {}
        void operator()(Record rec) { rec.aim_ = aim_; }
    private:
        Point aim_;
    };
    struct ChangeHitPoint : public std::unary_function<Record, void>
    {
        ChangeHitPoint(const Point &hit) : hit_(hit) {}
        void operator()(Record rec) { rec.move_final_ = hit_; }
    private:
        Point hit_;
    };

    double  hit_x() const { return move_final_.x; }
    double  hit_y() const { return move_final_.y; }

    double  aim_x() const { return aim_.x; }
    double  aim_y() const { return aim_.y; }

    // ----------------------------------------
    Record() {}

    Record(IN const Point            &aim,
           IN const Point            &move_begin,
           IN const Point            &move_final,
           IN const muscles_array    &muscles,
           IN const frames_array     &starts,
           IN const frames_array     &lasts,
           IN size_t                  controls_count,
           IN const Robo::Trajectory &visited);

    Record(IN const Point            &aim,
           IN const Point            &move_begin,
           IN const Point            &move_final,
           IN const Robo::Control    &controls,
           IN const Robo::Trajectory &visited);

    // ----------------------------------------
    operator tstring() const
    {
        tstringstream ss;
        ss << _T("rec<x=") << hit.x << _T(", y=") << hit.y << _T(">");
        return  ss.str();
    }
    // ----------------------------------------
    bool  operator== (const Record &rec) const
    { return (this == &rec) || (control_ == rec.control_); }
    bool  operator!= (const Record &rec) const
    { return (this != &rec) && (control_ != rec.control_); }
    // ----------------------------------------
    /* Microsoft specific: C++ properties */
    __declspec(property(get = _get_aim))      const Point&            aim;
    __declspec(property(get = _get_hit))      const Point&            hit;
    __declspec(property(get = _get_traj))     const Robo::Trajectory& trajectory;
    //__declspec(property(get = _get_muscles)) Robo::muscle_t         muscles;
    __declspec(property(get = _get_controls)) const Robo::Control&    controls;
    __declspec(property(get = _get_n_ctrls))  size_t                n_controls;

    // ----------------------------------------
    const Point&            _get_aim() const { return aim_; }
    const Point&            _get_hit() const { return move_final_; }
    const Robo::Trajectory& _get_traj() const { return visited_; }
    //Robo::muscle_t        _get_muscles() const { return muscles_; }
    size_t                  _get_n_ctrls() const { return control_.size(); }
    const Robo::Control&    _get_controls() const { return control_; }

    // ----------------------------------------
    double  distanceCovered() const
    { return boost_distance(move_final_, move_begin_); }

    double  eleganceMove() const;
    // ----------------------------------------
    double  ratioDistanceByTrajectory() const;
    double  ratioTrajectoryDivirgence() const;

    double  ratioUsedMusclesCount() const;
    double  ratioTrajectoryBrakes() const;

    Robo::frames_t  longestMusclesControl() const;
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
        boost::hash_combine(seed, boost::hash_value(rec.hit.x));
        boost::hash_combine(seed, boost::hash_value(rec.hit.y));
        // the result.
        return seed;
    }
};

}
BOOST_CLASS_VERSION (RoboMoves::Record, 2)
//------------------------------------------------------------------------------
#endif // _RECORD_H_
