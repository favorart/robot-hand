#pragma once
#include "Point.h"
#include "RoboControl.h"
#include "RoboEnviroment.h"
#include "RoboMotionLaws.h"

//-------------------------------------------------------------------------------
namespace rl_problem {
struct ObservationRobo;
}
namespace Robo {
//-------------------------------------------------------------------------------
using distance_t = double;
using Trajectory = std::vector<Point>;
using Trajectories = std::list<Trajectory>;
using joint_t = uint8_t;
const joint_t JInvalid = 0xFF;
//-------------------------------------------------------------------------------
struct State final
{
    int special_no{ 0 };
    Trajectory positions{};
    Trajectory velosities{};
    Trajectory accelerations{};
    //------------------------------------------
    State() = default;
    State(State&&) = default;
    State(const State&) = default;
    //------------------------------------------
    State& operator=(const State&) = default;
    //------------------------------------------
    template <typename Points>
    State(const Points &p, const Points &v, const Points &a, int special=0) :
        positions(p.begin(),p.end()), 
        velosities(v.begin(),v.end()), 
        accelerations(a.begin(),a.end()), 
        special_no(special)
    {}
    const Point& spec() const { return positions[special_no]; }
    //------------------------------------------
    template <class Archive>
    void serialize(Archive &ar, unsigned /*version*/)
    { ar & special_no & positions & velosities & accelerations; }
    //------------------------------------------
    bool operator==(const State &state) const
    {
        return (this == &state) ||
            (special_no == state.special_no &&
             positions == state.positions &&
             velosities == state.velosities &&
             accelerations == state.accelerations);
    }
    bool operator!=(const State &state) const { return !(*this == state); }
    //------------------------------------------
    friend tostream& operator<<(tostream &s, const State &state);
};
//------------------------------------------
using StateTrajectory = std::vector<State>;
using StateTrajectories = std::list<StateTrajectory>;
//-------------------------------------------------------------------------------
struct JointInput
{
    MotionLaws::JointMotionLaw frames{};
    bool show = true;
    Point base{};
    size_t joint{}; // not in order of `joint_t`, but Type::Joints order
    bool operator<(const JointInput &ji) const { return (joint < ji.joint); }
    JointInput() = default;
    JointInput(joint_t joint, bool show, const Point &base, const MotionLaws::JointMotionLaw &frames) :
        joint(joint), frames(frames), show(show), base(base)
    {}
    virtual ~JointInput() {}
    friend tostream& operator<<(tostream &s, const JointInput &ji)
    { return s << "{ " << ji.joint << ' ' << ji.base << ' ' << ji.show << " +MLaw }"; }
    virtual void save(tptree &root) const;
    virtual void load(tptree &root);
};
//-------------------------------------------------------------------------------
using JointInputPtr = std::shared_ptr<JointInput>;
using JointsInputsPtrs = std::list<JointInputPtr>;
using JointsInputs = std::list<JointInput>;
//-------------------------------------------------------------------------------
/*  jointsOpenPercent={ joint, 0.% <= value <= 100.% } */
using JointsOpenPercent = std::initializer_list<std::pair<joint_t, double>>;
//-------------------------------------------------------------------------------
template <typename T> void forceIncludeMethodMake() { T::name(); T::make(); }
//-------------------------------------------------------------------------------
/// Robotic Interface Interaction
class RoboI
{
protected:
    frames_t _frame{ 0 };
    StateTrajectory _trajectory{};
    bool _trajectory_save{ true };
    void _reset()
    {
        _frame = 0;
        _trajectory.clear();
        _trajectory_save = true;
    }

public:
    static const distance_t minFrameMove;
    static const muscle_t musclesPerJoint = 2;

    static const muscle_t musclesMaxCount = 8;// !!! 32;
    static const muscle_t jointsMaxCount = musclesMaxCount / musclesPerJoint;

    static muscle_t muscleOpposite(IN muscle_t muscle)
    {
        //if (muscle >= musclesCount())
        //    throw std::exception{"Incorrect muscle"};
        return ((muscle % musclesPerJoint) ? (muscle - 1): (muscle + 1));
    }
    static muscle_t muscleByJoint(IN joint_t joint, IN bool open)
    { return (joint * musclesPerJoint + !open); }
    static joint_t  jointByMuscle(IN muscle_t muscle)
    { return (muscle / musclesPerJoint); }

    //----------------------------------------------------
    RoboI(const Point &base, const JointsInputsPtrs &joint_inputs) : _base(base), _joint_inputs(joint_inputs) {}
    RoboI(RoboI&&) = delete;
    RoboI(const RoboI&) = delete;
    virtual ~RoboI() {}

//protected:
    virtual void setJoints(const Robo::JointsOpenPercent&) = 0;
    virtual bool isCollision() const = 0;

#ifdef DEBUG_RM
    virtual frames_t muscleStatus(muscle_t m) const = 0;
    virtual frames_t lastsStatus(muscle_t m) const = 0;
    virtual TCHAR lastsStatusT(muscle_t m) const = 0;
#endif
//public:
    //----------------------------------------------------
    virtual joint_t  jointsCount() const = 0;
    virtual muscle_t musclesCount() const = 0;

    virtual frames_t muscleMaxLasts(IN muscle_t) const = 0;
    virtual frames_t muscleMaxLasts(IN const Control&) const = 0;

    virtual void draw(IN HDC, IN HPEN, IN HBRUSH) const = 0;
    virtual void getWorkSpace(OUT Trajectory&) = 0;
    
    virtual frames_t move(IN const Control &controls, IN frames_t max_frames = LastsInfinity) = 0;
    virtual frames_t move(IN const std::bitset<musclesMaxCount> &bits, IN frames_t lasts, IN frames_t max_frames = LastsInfinity) = 0;

    virtual frames_t move(IN frames_t max_frames = LastsInfinity) = 0;
    virtual frames_t move(IN const std::vector<muscle_t>&, IN frames_t max_frames = LastsInfinity) = 0;
    virtual frames_t move(IN const BitsControl<musclesMaxCount + 1>&, IN frames_t max_frames = LastsInfinity) = 0;

    virtual void step() = 0;
    virtual void step(IN const Robo::Control &control) = 0;
    virtual void step(IN const Control &control, OUT size_t &control_curr) = 0;

    virtual void step(IN const std::bitset<musclesMaxCount> &muscles, IN frames_t lasts) = 0;
    using bitwise = std::bitset<musclesMaxCount + 1>;
    virtual void step(const bitwise &muscles) = 0;

    bool operator!=(const RoboI &r) const
    { return !(*this == r); }
    bool operator==(const RoboI &r) const;
    //----------------------------------------------------
    virtual void reset() = 0;
    virtual bool moveEnd() const = 0;
    virtual const Point& position() const = 0;
    virtual const Point& jointPos(joint_t) const = 0;
    virtual const StateTrajectory& trajectory() const { return _trajectory; };

    virtual State currState() const = 0;
    virtual rl_problem::ObservationRobo getCurrState() const = 0;

    //----------------------------------------------------
    virtual void setTrajectorySave(bool save) { _trajectory_save = save; };
    virtual frames_t frame() const { return _frame; };

    virtual Enviroment getEnvCond() const = 0;
    virtual void setEnvCond(Enviroment conditions) = 0;
    //----------------------------------------------------
    static tstring name() { return _T("robo"); };
    virtual tstring getName() const = 0;

    /*virtual*/ void save(tptree &root) const;

protected:
    /* save-load data */
    Point _base{};
    JointsInputsPtrs _joint_inputs{};

private:
    static std::shared_ptr<RoboI> make(const tstring &/*type*/, tptree &/*node*/) { throw std::logic_error("Depricated"); };
    //----------------------------------------------------
    //friend class EnvEdges;
    friend void forceIncludeMethodMake<RoboI>();
};
//-------------------------------------------------------------------------------
using bitset_t = std::bitset<Robo::RoboI::musclesMaxCount>;
using pRoboI = std::shared_ptr<Robo::RoboI>;
//-------------------------------------------------------------------------------
template<typename JInput>
std::list<JInput> JInputs(const std::list<std::shared_ptr<Robo::JointInput>> &joints)
{
    std::list<JInput> inputs;
    for (auto &j : joints) inputs.push_back(*dynamic_cast<JInput*>(j.get()));
    return inputs;
}
//-------------------------------------------------------------------------------
inline Robo::frames_t musclesMaxLasts(const Robo::RoboI &robo)
{
    frames_t l = 0;
    for (muscle_t m = 0; m < robo.musclesCount(); ++m)
        if (l < robo.muscleMaxLasts(m))
            l = robo.muscleMaxLasts(m);
    return l;
}
} // end namespace Robo

//------------------------------------------------------------------------------
BOOST_CLASS_VERSION(Robo::State, 1)
//------------------------------------------------------------------------------
