#pragma once

#include "RoboPhysics.h"

namespace Robo {
class EnvEdgesTank;
namespace Mobile {

#define TANK_VER 2
//#define TANK_DEBUG

//------------------------------------------------------------------------------
class Tank : public RoboPhysics
{
public:
    enum class Muscle : uint8_t
    {
        LTrackFrw = 0, // Frw = forward
        LTrackBck = 1, // Bck = backward
        RTrackFrw = 2,
        RTrackBck = 3,
        MCount = 4,
        MInvalid = 5
    };
    enum class Joint : uint8_t
    {
        LTrack = 0,
        RTrack = 1,
        JCount = 2,
        Center = 2,
        JInvalid = 3
    };

    Tank::Muscle MofJ(Tank::Joint joint, bool open) const
    { return Tank::Muscle(size_t(joint) * 2 + !open); }
    Tank::Joint  JofM(Tank::Muscle muscle) const
    { return Tank::Joint(size_t(muscle) / 2); }

    Tank::Muscle M(muscle_t muscle) const
    { return (muscle >= muscles) ? Tank::Muscle::MInvalid : params.musclesUsed[muscle]; }
    Tank::Joint  J(joint_t joint) const
    { return (joint >= joints) ? Tank::Joint::JInvalid : params.jointsUsed[joint]; }

    struct JointInput;

protected:
    static const size_t muscles = (size_t)(Tank::Muscle::MCount);
    static const size_t joints = (size_t)(Tank::Joint::JCount);

    struct Params
    {
        std::array<Tank::Muscle, muscles> musclesUsed{ Tank::Muscle::MInvalid };
        std::array<Tank::Joint, joints> jointsUsed{ Tank::Joint::JInvalid };
        //--- draw constants
        double   trackWidth{};
        double  trackHeight{};
        double   bodyHeight{};
        double centerRadius{};

        Params(const JointsInputsPtrs&, const Tank&);
    };
    const Params params;

#ifdef TANK_DEBUG
    Point center_;
    double r1_, r2_;
#endif
    void realMove();
    
public:
    Tank(const Point &baseCenter, const JointsInputsPtrs &joints);

    frames_t muscleMaxLasts(muscle_t muscle) const;
    frames_t muscleMaxLasts(const Robo::Control &control) const;

    void getWorkSpace(OUT Trajectory &workSpace);
    void draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const;
    
    void resetJoint(IN joint_t);
    void setJoints(IN const Robo::JointsOpenPercent&);
    
    const Point& position() const;

    tstring name() const { return _T("Tank-v2"); }
    friend class Robo::EnvEdgesTank;
};

}
}
//------------------------------------------------------------------------------

