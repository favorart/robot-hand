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
    static const double minFrameMove;

public:
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

    virtual frames_t muscleMaxLast(IN muscle_t) const = 0;
    virtual frames_t muscleMaxLast(IN const Control&) const = 0;

    //----------------------------------------------------
    virtual frames_t move(IN const Control&, OUT Trajectory&) = 0;
    virtual void     step(IN frames_t frame, IN muscle_t muscle = MInvalid, IN frames_t last = 0) = 0;

    virtual void     draw(IN HDC, IN HPEN, IN HBRUSH) const = 0;

    virtual void     getWorkSpace(OUT Trajectory&) = 0;

    //----------------------------------------------------
    using JointsOpenPercent = std::initializer_list<std::pair<joint_t, double>>;
    virtual void setJoints(IN const JointsOpenPercent&) = 0;
    virtual void resetJoint(IN joint_t) = 0;
    virtual void reset() = 0;

    virtual Point jointPos(IN joint_t) const = 0;
    //----------------------------------------------------
    virtual void controlsValidate(const Control&) const = 0;
    //----------------------------------------------------
    virtual bool moveEnd() const = 0;
    virtual const Point& position() const = 0;
    //----------------------------------------------------

    virtual unsigned getVisitedRarity() const = 0;
    virtual void     setVisitedRarity(unsigned rarity) = 0;

    virtual unsigned getWindy() const = 0;
    virtual void     setWindy(bool windy) = 0;
    //----------------------------------------------------
    virtual tstring name() const = 0;
    //----------------------------------------------------
    friend class EnvEdges;
    
    //----------------------------------------------------
    /* сериализация */
    //virtual void save(IN const tstring &filename) const = 0;
    //virtual void load(IN const tstring &filename) = 0;
};
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
        if (l < robo.muscleMaxLast(m))
            l = robo.muscleMaxLast(m);
    return l;
}
//-------------------------------------------------------------------------------
}
