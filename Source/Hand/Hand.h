#pragma once

#include "RoboPhysics.h"


namespace Robo {
class EnvEdgesHand;
namespace NewHand {

#define HAND_VER 4

class Hand : public RoboPhysics
{
public:
    enum Muscle : uint8_t
    {   
        WristOpn  = 0, // Opn - open
        WristCls  = 1, // Cls - close
        ElbowOpn  = 2,
        ElbowCls  = 3,
        ShldrOpn  = 4,
        ShldrCls  = 5,
        ClvclOpn  = 6,
        ClvclCls  = 7,
        MCount    = 8,
        MInvalid  = 9
    };
    enum Joint : uint8_t
    {
        Wrist     = 0, // запястье:  wrist (carpus)
        Elbow     = 1, // локоть:    elbow
        Shldr     = 2, // плечо:     shoulder
        Clvcl     = 3, // ключица:   clavicle
        JCount    = 4,
        JBase     = 4,
        JInvalid  = 5
    };

    Hand::Muscle MofJ(Joint joint, bool open) const
    { return Muscle(uint8_t(joint) * 2 + !open); }
    Hand::Joint  JofM(Muscle muscle) const
    { return Joint(uint8_t(muscle) / 2); }
    //----------------------------------------------------
    Hand::Muscle M(muscle_t muscle) const
    {
        if (muscle >= musclesCount())
            return Hand::MInvalid;
        return params.musclesUsed[muscle];
    }
    Hand::Joint  J(joint_t joint) const
    {
        if (joint >= jointsCount())
            return Hand::JInvalid;
        return params.jointsUsed[joint];
    }

    frames_t muscleMaxLast(muscle_t) const;
    frames_t muscleMaxLast(const Control&) const;
    //----------------------------------------------------
    struct JointInput;

protected:
    // текущие смещения каждого сочления
    std::array<double, Hand::Joint::JCount> angles{}; // angle_Wrist, angle_Elbow, angle_Shldr, shift_Clvcl
    // jointsBases{}; // jointsOpenCoords{}; // palm, hand, arm, shoulder (clavicle fixed)

    struct Params
    {
        std::array<Hand::Muscle, Hand::MCount> musclesUsed;
        std::array<Hand::Joint,  Hand::JCount>  jointsUsed;

        std::array<double, Hand::JCount> defOpen;

        std::array<double, Hand::Joint::JCount> maxAngles{}; // maxWristAngle, maxElbowAngle, maxShldrAngle, maxClvclShift
        std::array<frames_t, Hand::Joint::JCount> nMoveFrames{}; // число кадров от полного раскрытия сустава до полного закрытия
        std::array<frames_t, Hand::Joint::JCount> nStopFrames{}; // число кадров для остановки сустава при полном ускорении

        //--- draw constants ---
        bool drawPalm;
                
        double   palmRadius;
        double  jointRadius;
        double sectionWidth;

        Params(const JointsPInputs &jointInputs, const Hand &hand);
    };
    const Params params;
    
    void realMove();
    void jointMove(joint_t, double);
        
public:
    Hand(const Point &base, const JointsPInputs &joints);

    void getWorkSpace(OUT Trajectory&);
    void draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const;
    
    void resetJoint(IN joint_t);
    void reset();

    /*  jointsOpenPercent={ j={ Clvcl, Shldr, Elbow, Wrist }, val <= 100.0% } */
    void setJoints(IN const Robo::JointsOpenPercent&);
    //const Point& position() const { return status.curPos[(params.drawPalm ? Joint::Wrist : Joint::Elbow)]; }

    tstring name() const { return _T("Hand-v4"); }
    friend class Robo::EnvEdgesHand;
};

}
}
