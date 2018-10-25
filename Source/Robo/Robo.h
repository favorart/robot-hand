﻿#pragma once
#include "Point.h"
#include "RoboControl.h"
#include "RoboMotionLaws.h"


namespace Robo
{
namespace Learn
{
class Act
{
    //static unsigned maxActorsNumber = 32;
    unsigned actors;
    unsigned b_acts;
public:
    Act(int actors) : actors(actors), b_acts(0) {}
    int64_t nActions() const { return static_cast<int64_t>(std::pow(2, actors)); }
    bool setActs(unsigned acts) { b_acts = acts; }
    bool getAct(unsigned i) const
    {
        i = ((i >= actors) ? (actors - 1) : i);
        return ((b_acts & (1 << i)) != 0);
    }
};

//template <int N>
struct State : public Point // !!! REMOVE Temporary
{
    struct SState
    {
        Point pos{};
        Point velosity{};
        Point accel{}; //acceleration
    };
    SState joints[4] = {};
    Act act{4};

    State() {}
    State(const Point& p) : Point(p) {}
    //State(const Point& pos, const Point& velosity, const Point& accel) : Point(p) {}
};
}
//-------------------------------------------------------------------------------
//struct TStringSerializableI { virtual operator tstring() const = 0; };
//-------------------------------------------------------------------------------
using distance_t = double;
using Trajectory = std::vector<Point/*Learn::State*/>;
using Trajectories = std::list<Trajectory>;
using joint_t = uint8_t;
const joint_t JInvalid = 0xFF;
//-------------------------------------------------------------------------------
struct JointInput
{
    MotionLaws::JointMotionLaw frames{ nullptr, nullptr };
    bool show = true;
    Point base{};
    size_t nMoveFrames{};
    distance_t maxMoveFrame{};
    size_t joint{}; // not in order of `joint_t`, but Type::Joints order

    bool operator<(const JointInput &ji) const { return (joint < ji.joint); }

    JointInput() = default;
    JointInput(joint_t joint, const MotionLaws::JointMotionLaw &frames, bool show,
               const Point &base, size_t nMoveFrames, double maxMoveFrame) :
        joint(joint), frames(frames), show(show), base(base),
        nMoveFrames(nMoveFrames), maxMoveFrame(maxMoveFrame)
    {}
    virtual ~JointInput() {}
    friend tostream& operator<<(tostream &s, const JointInput &ji)
    {
        return s << "{ " << ji.joint << ' ' << ji.base << ' ' 
                 << ji.nMoveFrames << ' ' << ji.maxMoveFrame << ' ' << ji.show << " +MLaw }";
    }
    virtual void save(tptree &root) const;
    virtual void load(tptree &root);
};
//-------------------------------------------------------------------------------
using JointInputPtr = std::shared_ptr<JointInput>;
using JointsInputsPtrs = std::list<JointInputPtr>;
using JointsInputs = std::list<JointInput>;
//-------------------------------------------------------------------------------
template <typename T> void forceIncludeMethodMake() { T::name(); T::make(); }
//-------------------------------------------------------------------------------
/// Robotic Interface Interaction
class RoboI
{
protected:
    //static const double minFrameMove;
    
    frames_t _frame{ 0 };
    Trajectory _trajectory{};
    bool _trajectory_save{ true };

    void _reset()
    {
        _frame = 0;
        _trajectory.clear();
        _trajectory_save = true;
    }

public:
    static const double minFrameMove;
    static const muscle_t musclesPerJoint = 2;

    static const muscle_t musclesMaxCount = 8;// !!! 32;
    static const muscle_t jointsMaxCount = musclesMaxCount / musclesPerJoint;

    static muscle_t muscleOpposite(IN muscle_t muscle)
    {
        //if (muscle >= musclesCount())
        //    throw std::exception{"Incorrect muscle"};
        return ((muscle % 2) ? (muscle - 1): (muscle + 1));
    }
    static muscle_t muscleByJoint(IN joint_t joint, IN bool open)
    { return (joint * 2 + !open); }
    static joint_t  jointByMuscle(IN muscle_t muscle)
    { return (muscle / 2); }

    //----------------------------------------------------
    RoboI(const JointsInputsPtrs &joint_inputs) : _joint_inputs(joint_inputs) {}
    RoboI(RoboI&&) = delete;
    RoboI(const RoboI&) = delete;

    // RM !!! DEBUG
    virtual frames_t muscleStatus(muscle_t m) const = 0;
    virtual frames_t lastsStatus(muscle_t m) const = 0;
    virtual TCHAR lastsStatusT(muscle_t m) const = 0;

    //----------------------------------------------------
    virtual joint_t  jointsCount() const = 0;
    virtual muscle_t musclesCount() const = 0;

    virtual frames_t muscleMaxLasts(IN muscle_t) const = 0;
    virtual frames_t muscleMaxLasts(IN const Control&) const = 0;

    virtual void draw(IN HDC, IN HPEN, IN HBRUSH) const = 0;
    virtual void getWorkSpace(OUT Trajectory&) = 0;
    
    virtual frames_t move(IN const Control &controls) = 0;
    virtual frames_t move(IN const Control &controls, IN frames_t max_frames) = 0;
    virtual frames_t move(IN const std::bitset<musclesMaxCount> &muscles, IN frames_t lasts) = 0;
    virtual frames_t move(IN const std::bitset<musclesMaxCount> &muscles, IN frames_t lasts, IN frames_t max_frames) = 0;

    virtual frames_t move(IN frames_t max_frames = LastsInfinity) = 0;
    virtual frames_t move(IN const std::vector<muscle_t> &ctrls) = 0;

    virtual void step() = 0;
    virtual void step(IN const Robo::Control &control) = 0;
    virtual void step(IN const Control &control, OUT size_t &control_curr) = 0;
    virtual void step(IN const std::bitset<musclesMaxCount> &muscles, IN frames_t lasts) = 0;

    using bitwise = std::bitset<musclesMaxCount + 1>;
    virtual void step(const bitwise &muscles) = 0;

    //----------------------------------------------------
    virtual void reset() = 0;
    virtual void resetJoint(joint_t) = 0;
    //virtual void setJoints(const JointsOpenPercent&) = 0;

    //----------------------------------------------------
    virtual bool moveEnd() const = 0;
    virtual const Point& position() const = 0;
    virtual Point jointPos(joint_t) const = 0;
    virtual /*const*/ Trajectory& trajectory() /*const*/ { return _trajectory; };

    //----------------------------------------------------
    virtual void setTrajectorySave(bool save) { _trajectory_save = save; };
    virtual frames_t frame() const { return _frame; };

    virtual unsigned getVisitedRarity() const = 0;
    virtual void     setVisitedRarity(unsigned rarity) = 0;

    virtual unsigned getWindy() const = 0;
    virtual void     setWindy(bool windy) = 0;
    //----------------------------------------------------
    static tstring name() { return _T("robo"); };
    virtual tstring getName() const = 0;
    /*virtual*/ void save(tptree &root) const;

protected:
    /* save-load data */
    virtual Point _base() const = 0;
    JointsInputsPtrs _joint_inputs{};

private:
    static std::shared_ptr<RoboI> make(const tstring &type, tptree &node)
    { throw std::logic_error("Depricated"); };
    //----------------------------------------------------
    friend class EnvEdges;
    friend void forceIncludeMethodMake<RoboI>();
};
//-------------------------------------------------------------------------------
using bitset_t = std::bitset<Robo::RoboI::musclesMaxCount>;
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
//-------------------------------------------------------------------------------
}
