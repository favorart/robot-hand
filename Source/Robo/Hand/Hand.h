#pragma once

#include "RoboPhysics.h"

namespace Robo {
class EnvEdgesHand;
namespace NewHand {

#define HAND_VER 4

class Hand : public RoboPhysics
{
public:
    struct JointInput;
    enum class Muscle : uint8_t
    {   
        WristCls  = 0, // Cls - close
        WristOpn  = 1, // Opn - open
        ElbowCls  = 2,
        ElbowOpn  = 3,
        ShldrCls  = 4,
        ShldrOpn  = 5,
        ClvclCls  = 6,
        ClvclOpn  = 7,
        MCount    = 8,
        MInvalid  = 9
    };
    enum class Joint : uint8_t
    {
        SpecP     = 0,
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
    
protected:
    static const size_t joints = (size_t)(Hand::Joint::JCount);
    static const size_t muscles = (size_t)(Hand::Muscle::MCount);    
    std::array<distance_t, joints> angles{}; ///< текущие смещения каждого сочления (palm, hand, arm, shoulder, clavicle)

    struct Params
    {
        std::array<Hand::Muscle, muscles> musclesUsed{};
        std::array<Hand::Joint, joints> jointsUsed{};
        std::array<distance_t, joints> defOpen{};

        std::array<distance_t, joints> maxAngles{}; // maxWristAngle, maxElbowAngle, maxShldrAngle, maxClvclShift
        std::array<frames_t, joints> nMoveFrames{}; // число кадров от полного раскрытия сустава до полного закрытия
        std::array<frames_t, joints> nStopFrames{}; // число кадров для остановки сустава при полном ускорении

        //--- draw constants ---
        bool       drawPalm{};
        double   palmRadius{};
        double  jointRadius{};
        double sectionWidth{};

        Params(const JointsInputsPtrs &jointInputs, const Hand &hand);
    };
    const Params params;
        
    void realMove();
    void jointMove(joint_t, double offset);
    distance_t maxJointAngle(joint_t joint) const { return params.maxAngles[joint]; }
        
public:
    Hand(const Point &base, const JointsInputsPtrs &joints);

    frames_t muscleMaxLasts(muscle_t) const;
    frames_t muscleMaxLasts(const Control&) const;

    void getWorkSpace(OUT Trajectory&);
    void draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const;
    int specPoint() const { return static_cast<int>(Joint::SpecP); }
    
    void resetJoint(IN joint_t);
    void setJoints(IN const Robo::JointsOpenPercent&);

    tstring getName() const { return Hand::name(); }
    static tstring name() { return _T("Hand-v4"); }
    static std::shared_ptr<RoboI> make(const tstring &type, tptree &node);
    friend class Robo::EnvEdgesHand;
};

}
}
