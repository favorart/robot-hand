#pragma once

#include "Robo.h"
/// TODO: !!! UPDATE TIME & SIM_TIME when request the Record !!!

namespace RoboMoves {
//------------------------------------------------------------------------------
using SimTime = uint64_t;

typedef std::vector<Robo::frames_t>   frames_array;
typedef std::vector<Robo::muscle_t>  muscles_array;

#define  CHECK_BIT(var,pos)   ((var)&(1<<(pos)))
#define  BITSINBYTE           8

//------------------------------------------------------------------------------
class Strategy final
{
    Robo::muscle_t _nmuscles{};
    using Value = uint64_t;
    Value _number{};
    static const size_t NPOS = (sizeof(_number)*BITSINBYTE);
    Value chunk(size_t pos) const;
    size_t nchunks() const { return (NPOS / _nmuscles); }
public:
    auto number() const { return _number; }
    Strategy(Robo::muscle_t nmuscles) : _nmuscles(nmuscles) {}
    Strategy(Strategy&&) = default;
    Strategy(const Strategy&) = default;
    Strategy& operator=(const Strategy&) = default;
    bool operator==(const Strategy&) const;
    bool operator!=(const Strategy&s) const { return !(*this == s); }
    bool almost_eq(const Strategy&) const;
    static Strategy get(const Robo::Control&);
    static Strategy get(const Robo::Control&, Robo::muscle_t nmuscles);
    static Strategy getEmpty() { return Strategy(0); }
};

//------------------------------------------------------------------------------
class Record final
{
    using Traj = Robo::StateTrajectory;
    using Control = Robo::Control;
    Point aim_{}; // TODO: ??? REMOVE
    Point move_begin_{}, move_final_{};
    Control control_{};
    Traj visited_{};

    Strategy _strategy{ Robo::MInvalid }; ///< набор зайствованных мускулов в управлении
    Robo::frames_t _lasts_step{};
    mutable double _error_distance{0.};
    
    //bool _outdated{false};
    mutable std::time_t _update_time{0}; ///< geographic time
    mutable SimTime     _update_traj{0}; ///< time in used trajectories

    // ----------------------------------------
    friend class boost::serialization::access;
    BOOST_SERIALIZATION_SPLIT_MEMBER()
    template <class Archive>
    void save(Archive &ar, unsigned /*version*/) const
    {
        ar & aim_ & move_begin_ & move_final_ & control_ & visited_;
        //ar & _lasts_step & _strategy;
        ar &(_update_time - std::time(NULL)) & _update_traj & _error_distance;
    }
    template <class Archive>
    void load(Archive &ar, unsigned /*version*/)
    {
        ar & aim_ & move_begin_ & move_final_ & control_ & visited_;
        //ar & _lasts_step & _strategy;
        ar & _update_time & _update_traj & _error_distance;
        _update_time += std::time(NULL);
    }

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
           IN const Traj             &visited);

    Record(IN const Point            &aim,
           IN const Point            &move_begin,
           IN const Point            &move_final,
           IN const Control          &controls,
           IN const Traj             &visited,
           IN Robo::frames_t lasts_step=Robo::LastsInfinity);
    // ----------------------------------------
    bool  operator== (const Record &rec) const
    { return (this == &rec) || (control_ == rec.control_); }
    bool  operator!= (const Record &rec) const
    { return (this != &rec) && (control_ != rec.control_); }

    bool operator< (const Record &rec) const
    { return (hit < rec.hit); }
    bool operator< (const Point &p) const
    { return (hit < p); }
    // ----------------------------------------
    /* Container index keys  */
    Robo::distance_t hit_x() const { return move_final_.x; }
    Robo::distance_t hit_y() const { return move_final_.y; }
    Robo::distance_t aim_x() const { return aim_.x; }
    Robo::distance_t aim_y() const { return aim_.y; }
    Robo::distance_t distanceCovered() const
    { return bg::distance(move_final_, move_begin_); }
    // ----------------------------------------
    /* Microsoft specific: C++ properties */
    __declspec(property(get = getAim))       const Point&            aim;
    __declspec(property(get = getHit))       const Point&            hit;
    __declspec(property(get = getVisited))   const Traj&      trajectory;
    __declspec(property(get = getControls))  const Control&     controls;
    __declspec(property(get = getNControls)) size_t           n_controls;

    // ----------------------------------------
    const Point&            getAim() const { return aim_; }
    const Point&            getHit() const { return move_final_; }
    const Traj&             getVisited() const { return visited_; }
    size_t                  getNControls() const { return control_.size(); }
    const Control&          getControls() const { return control_; }

    void updateErrDistance(const Point &hit) { _error_distance = boost_distance(aim, hit); }
    void updateTimeTraj(SimTime sim_time) const { _update_traj = sim_time; }
    void updateTimeAim(const Point &aim) { aim_ = aim; }
    // ----------------------------------------
    void clear();
    // ----------------------------------------
    Robo::distance_t ratioDistanceByTrajectory() const;
    Robo::distance_t ratioTrajectoryDivirgence() const;
    Robo::distance_t ratioUsedMusclesCount() const;
    Robo::distance_t ratioTrajectoryBrakes() const;
    Robo::distance_t ratioSumOfWorkingTime() const;
    Robo::distance_t optimalMove() const; ///< less value is better
    // ----------------------------------------
    Robo::frames_t controlsDense() const { return _lasts_step; }
    Robo::frames_t longestMusclesControl() const;
    // ----------------------------------------
    friend tostream& operator<<(tostream &s, const RoboMoves::Record &rec);
    friend tistream& operator>>(tistream &s, RoboMoves::Record &rec);
};
//------------------------------------------------------------------------------
struct RecordHasher
{
    size_t operator()(const Record& rec) const { return PointHasher{}(rec.hit); }
};
} // end RoboMoves
//------------------------------------------------------------------------------
BOOST_CLASS_VERSION (RoboMoves::Record, 2)
//------------------------------------------------------------------------------
