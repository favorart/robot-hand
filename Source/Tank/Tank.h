#pragma once

#include "Robo.h"
#include "RoboEdges.h"
#include "RoboPhysics.h"


namespace Robo {
class EnvEdges;
class EnvEdgesTank;
namespace Mobile {

#define TANK_VER 2
#define TANK_DEBUG

//------------------------------------------------------------------------------
class Tank : public RoboPhysics
{
public:
    // Frw - forward, Bck - backward
    enum Muscle : uint8_t
    {
        LTrackFrw = 0,
        LTrackBck = 1,
        RTrackFrw = 2,
        RTrackBck = 3,
        MCount = 4,
        MInvalid = 5
    };
    enum Joint : uint8_t
    { LTrack = 0, RTrack = 1, JCount = 2, JCenter = 2, JInvalid = 3 };

    Tank::Muscle MofJ(Tank::Joint joint, bool open) const
    { return Tank::Muscle(size_t(joint) * 2 + !open); }
    Tank::Joint  JofM(Tank::Muscle muscle) const
    { return Tank::Joint(size_t(muscle) / 2); }
    //----------------------------------------------------
    Tank::Muscle M(muscle_t muscle) const
    {
        if (muscle >= Tank::MCount)
            return Muscle::MInvalid;
        return params.musclesUsed[muscle];
    }
    Tank::Joint  J(joint_t joint) const
    {
        if (joint >= Tank::JCount)
            return Joint::JInvalid;
        return params.jointsUsed[joint];
    }
    //----------------------------------------------------
    frames_t muscleMaxLast(const Robo::Control &control) const;
    frames_t muscleMaxLast(muscle_t muscle) const;
    //----------------------------------------------------
    struct JointInput;

protected:
    struct Params
    {
        //--- draw constants
        double   trackWidth;
        double  trackHeight;
        double   bodyHeight;
        double centerRadius;

        std::array<Tank::Muscle, Tank::MCount> musclesUsed;
        std::array<Tank::Joint, Tank::JCount>  jointsUsed;

        Params(const JointsPInputs&, const Tank&);
    };
    const Params params;

#ifdef TANK_DEBUG
    Point center_;
    double r1_, r2_;
#endif
    void realMove();
    
public:
    Tank(const Point &baseCenter, const JointsPInputs &joints);

    void getWorkSpace(OUT Trajectory &workSpace);
    void draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const;
    
    void reset();
    void resetJoint(IN joint_t);
    void setJoints(IN const Robo::JointsOpenPercent&);
    
    const Point&  position() const
    { 
        //static Point center = (status.curPos[Joint::LTrack] + status.curPos[Joint::RTrack]) / 2.;
        //return center;

        /* return status.curPos[Joint::JCount]; */
        return status.curPos[Joint::JCenter];
    }

    tstring name() const { return _T("Tank-v2"); }
    friend class Robo::EnvEdgesTank;
};

}
}
//------------------------------------------------------------------------------

