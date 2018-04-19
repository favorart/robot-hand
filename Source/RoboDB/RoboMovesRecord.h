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
    Point aim_{}; // TODO: REMOVE
    Point move_begin_{}, move_final_{};
    Robo::Control    control_{};
    Robo::Trajectory visited_{};

    //double _error_distance{};
    //bool _outdated{};

    // ----------------------------------------
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned version)
    { ar & aim_ & move_begin_ & move_final_ & control_ & visited_; }

public:
    Record() = default;
    Record(Record&&) = default;
    Record(const Record&) = default;
    // ----------------------------------------
    Record& operator=(Record&&) = default;
    Record& operator=(const Record&) = default;
    // ----------------------------------------
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
    bool  operator== (const Record &rec) const
    { return (this == &rec) || (control_ == rec.control_); }
    bool  operator!= (const Record &rec) const
    { return (this != &rec) && (control_ != rec.control_); }
    // ----------------------------------------
    /* Container index keys  */
    double  hit_x() const { return move_final_.x; }
    double  hit_y() const { return move_final_.y; }

    double  aim_x() const { return aim_.x; }
    double  aim_y() const { return aim_.y; }

    double  distanceCovered() const
    { return boost_distance(move_final_, move_begin_); }
    // ----------------------------------------
    /* Microsoft specific: C++ properties */
    __declspec(property(get = getAim))       const Point&            aim;
    __declspec(property(get = getHit))       const Point&            hit;
    __declspec(property(get = getVisited))   const Robo::Trajectory& trajectory;
    __declspec(property(get = getControls))  const Robo::Control&    controls;
    __declspec(property(get = getNControls)) size_t                n_controls;

    // ----------------------------------------
    const Point&            getAim() const { return aim_; }
    const Point&            getHit() const { return move_final_; }
    const Robo::Trajectory& getVisited() const { return visited_; }
    size_t                  getNControls() const { return control_.size(); }
    const Robo::Control&    getControls() const { return control_; }
    
    // ----------------------------------------
    void clear()
    {
        aim_ = { 0.,0. };
        move_begin_ = { 0.,0. };
        move_final_ = { 0.,0. };
        control_.clear();
        visited_.clear();
    }
    // ----------------------------------------
    double  eleganceMove() const;
    // ----------------------------------------
    double  ratioDistanceByTrajectory() const;
    double  ratioTrajectoryDivirgence() const;

    double  ratioUsedMusclesCount() const;
    double  ratioTrajectoryBrakes() const;

    Robo::frames_t  longestMusclesControl() const;
    // ----------------------------------------
    friend tostream& operator<<(tostream &s, const RoboMoves::Record &rec);
    friend tistream& operator>>(tistream &s, RoboMoves::Record &rec);
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
//------------------------------------------------------------------------------
BOOST_CLASS_VERSION (RoboMoves::Record, 2)
//------------------------------------------------------------------------------
#endif // _RECORD_H_
