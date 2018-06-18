#pragma once
#include "RoboControl.h"
#include "RoboMotionLaws.h"


namespace Robo
{
//-------------------------------------------------------------------------------
//struct TStringSerializableI { virtual operator tstring() const = 0; };
//-------------------------------------------------------------------------------
using Trajectory = std::list<Point>;
using Trajectories = std::list<Trajectory>;
using joint_t = uint8_t;
const joint_t JInvalid = 0xFF;
//-------------------------------------------------------------------------------
struct JointInput
{
    MotionLaws::JointMotionLaw frames{ nullptr, nullptr };
    bool show = true;

    JointInput() = default;
    JointInput(const MotionLaws::JointMotionLaw &frames, bool show) :
        frames(frames), show(show) {}
    virtual ~JointInput() {}
};
//-------------------------------------------------------------------------------
/// Robotic Interface Interaction
class RoboI
{
protected:
    //static const double minFrameMove;
    
    frames_t _frame{ 0 };
    Trajectory _trajectory{};
    bool _trajectory_show{ true };

    void _reset()
    {
        _frame = 0;
        _trajectory.clear();
        _trajectory_show = true;
    }

    virtual void realMove() = 0;

    virtual void step(IN muscle_t muscle /*= Robo::MInvalid*/, IN frames_t lasts /*= 0*/);

    virtual void muscleDriveStop(muscle_t) = 0;
    virtual bool muscleDriveFrame(muscle_t) = 0;
    virtual void muscleDriveMove(frames_t frame, muscle_t muscle, frames_t last) = 0;

    virtual bool somethingMoving() = 0;

public:
    static const double minFrameMove;
    static const muscle_t musclesMaxCount = 32;

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
    RoboI() {}
    RoboI(RoboI&&) = delete;
    RoboI(const RoboI&) = delete;
    RoboI(const Point&, const std::list<Robo::JointInput>&) {}
    RoboI(const Point&, const std::list<std::shared_ptr<Robo::JointInput>>&) {}

    //----------------------------------------------------
    virtual joint_t  jointsCount() const = 0;
    virtual muscle_t musclesCount() const = 0;

    virtual frames_t muscleMaxLasts(IN muscle_t) const = 0;
    virtual frames_t muscleMaxLasts(IN const Control&) const = 0;

    //----------------------------------------------------
    virtual void     draw(IN HDC, IN HPEN, IN HBRUSH) const = 0;    
    virtual void     getWorkSpace(OUT Trajectory&) = 0;

    virtual frames_t move(IN const Control &controls);
    virtual frames_t move(IN const Control &controls, IN frames_t max_frames);
    virtual frames_t move(IN const std::bitset<musclesMaxCount> &muscles, IN frames_t lasts);
    virtual frames_t move(IN const std::bitset<musclesMaxCount> &muscles, IN frames_t lasts, IN frames_t max_frames);
    virtual frames_t move(IN frames_t max_frames = LastsInfinity);

    virtual void      step();
    virtual void      step(IN const Robo::Control &control);
    virtual void      step(IN const Control &control, OUT size_t &control_curr);
    virtual void      step(IN const std::bitset<musclesMaxCount> &muscles, IN frames_t lasts);

    using bitwise = std::bitset<musclesMaxCount + 1>;
    virtual void step(const bitwise &muscles);

    virtual frames_t muscleStatus(muscle_t m) const = 0;
    virtual TCHAR    lastsStatusT(muscle_t m) const = 0;
    virtual frames_t lastsStatus(muscle_t m) const = 0;

    //----------------------------------------------------
    using JointsOpenPercent = std::initializer_list<std::pair<joint_t, double>>;
    virtual void setJoints(IN const JointsOpenPercent&) = 0;

    virtual void resetJoint(IN joint_t) = 0;
    virtual void reset() = 0;
    
    //----------------------------------------------------
    virtual bool moveEnd() const = 0;
    virtual const Point& position() const = 0;
    virtual Point jointPos(IN joint_t) const = 0;
    virtual /*const*/ Trajectory& trajectory() /*const*/ { return _trajectory; };
    //----------------------------------------------------

    virtual void     setTrajectoryShow(bool show) { _trajectory_show = show; };

    virtual unsigned getVisitedRarity() const = 0;
    virtual void     setVisitedRarity(unsigned rarity) = 0;

    virtual unsigned getWindy() const = 0;
    virtual void     setWindy(bool windy) = 0;
    //----------------------------------------------------
    virtual tstring name() const = 0;
    //----------------------------------------------------
    friend class EnvEdges;

    frames_t frame() const { return _frame; };
    //----------------------------------------------------
    /* сериализация */
    //virtual void save(IN const tstring &filename) const = 0;
    //virtual void load(IN const tstring &filename) = 0;
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
