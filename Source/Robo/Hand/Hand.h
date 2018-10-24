#pragma once

#include "RoboPhysics.h"

namespace Robo {
class EnvEdgesHand;
namespace NewHand {

#define HAND_VER 4

class Hand : public RoboPhysics
{
public:
    enum class Muscle : uint8_t
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
    enum class Joint : uint8_t
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
    { return Hand::Muscle(uint8_t(joint) * 2 + !open); }
    Hand::Joint  JofM(Muscle muscle) const
    { return Hand::Joint(uint8_t(muscle) / 2); }

    Hand::Muscle M(muscle_t muscle) const
    {
        if (muscle >= musclesCount())
            return Hand::Muscle::MInvalid;
        return params.musclesUsed[muscle];
    }
    Hand::Joint  J(joint_t joint) const
    {
        if (joint >= jointsCount())
            return Hand::Joint::JInvalid;
        return params.jointsUsed[joint];
    }

    struct JointInput;

protected:
    static const size_t joints = (size_t)(Hand::Joint::JCount);
    static const size_t muscles = (size_t)(Hand::Muscle::MCount);

    // текущие смещения каждого сочления
    std::array<distance_t, joints> angles{}; // angle_Wrist, angle_Elbow, angle_Shldr, shift_Clvcl
    // jointsBases{}; // jointsOpenCoords{}; // palm, hand, arm, shoulder (clavicle fixed)

    struct Params
    {
        std::array<Hand::Muscle, muscles> musclesUsed{};
        std::array<Hand::Joint, joints> jointsUsed{};
        std::array<distance_t, joints> defOpen{};

        std::array<distance_t, joints> maxAngles{}; // maxWristAngle, maxElbowAngle, maxShldrAngle, maxClvclShift
        std::array<frames_t, joints> nMoveFrames{}; // число кадров от полного раскрытия сустава до полного закрытия
        std::array<frames_t, joints> nStopFrames{}; // число кадров для остановки сустава при полном ускорении

        //--- draw constants ---
        bool drawPalm{};
                
        double   palmRadius{};
        double  jointRadius{};
        double sectionWidth{};

        Params(const JointsInputsPtrs &jointInputs, const Hand &hand);
    };
    const Params params;
    
    distance_t maxJointAngle(joint_t joint) const
    { return params.maxAngles[joint]; }
    
    void realMove();
    void jointMove(joint_t, double offset);
        
public:
    Hand(const Point &base, const JointsInputsPtrs &joints);

    frames_t muscleMaxLasts(muscle_t) const;
    frames_t muscleMaxLasts(const Control&) const;

    void getWorkSpace(OUT Trajectory&);
    void draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const;
    
    void resetJoint(IN joint_t);
    void setJoints(IN const Robo::JointsOpenPercent&);
    //const Point& position() const { return status.curPos[(params.drawPalm ? Joint::Wrist : Joint::Elbow)]; }

    static tstring name() { return _T("Hand-v4"); }
    static std::shared_ptr<RoboI> make(const tstring &type, tptree &node);
    friend class Robo::EnvEdgesHand;
};

}
}
