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

    Hand::Muscle MofJ(Joint j, bool frwd) const { return Hand::Muscle(muscle_t(j) * musclesPerJoint + !frwd); }
    Hand::Joint  JofM(Muscle m)           const { return Hand::Joint(joint_t(m) / musclesPerJoint); }
    Hand::Muscle M(muscle_t m) const { return (m >= musclesCount()) ? Hand::Muscle::MInvalid : params.musclesUsed[m]; }
    Hand::Joint  J(joint_t j)  const { return (j >= jointsCount())  ? Hand::Joint::JInvalid  : params.jointsUsed[j]; }
    
    frames_t muscleMaxLasts(muscle_t) const override;
    frames_t muscleMaxLasts(const Control&) const override;

    Hand(const Point &base, const JointsInputsPtrs &joints);
    void getWorkSpace(OUT Trajectory&) override;
    void draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const override;
    int specPoint() const override { return static_cast<int>(Joint::SpecP); }

    void reset() override;
    void resetJoint(IN joint_t) override;
    void setJoints(IN const Robo::JointsOpenPercent&) override;

    tstring getName() const override { return Hand::name(); }

    static tstring name() { return _T("Hand-v4"); }
    static std::shared_ptr<RoboI> make(const tstring &type, tptree &node);
    
protected:
    static const size_t joints = (size_t)(Hand::Joint::JCount);
    static const size_t muscles = (size_t)(Hand::Muscle::MCount);    

    struct Params final
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
    distance_t maxAngleJoint(joint_t joint) const { return params.maxAngles[joint]; }
    distance_t prismaticFactor(joint_t j) const override { return ((J(j) == Joint::Clvcl) ? (3 * M_PI) : 1.); }

    std::array<distance_t, joints> angles{}; ///< текущие смещения каждого сочления (palm, hand, arm, shoulder, clavicle)
    bool realMove() override;
    void realMoveJoint(joint_t, double offset);

    friend class Robo::EnvEdgesHand;
};

}
}
