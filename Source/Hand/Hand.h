#pragma once

#include "Robo.h"


namespace Robo {
class EnvEdgesHand;
namespace NewHand {

#define HAND_VER 3

class Hand : public RoboI
{
public:
    //----------------------------------------------------
    enum Muscle : uint8_t
    {   /* Opn - open, Cls - close */
        WristOpn  = 0,
        WristCls  = 1,
        ElbowOpn  = 2,
        ElbowCls  = 3,
        ShldrOpn  = 4,
        ShldrCls  = 5,
        ClvclOpn  = 6,
        ClvclCls  = 7,
        MCount    = 8,
        MInvalid  = 9
    };

    //----------------------------------------------------
    enum Joint : uint8_t
    {
        Wrist     = 0, // запястье:  wrist (carpus)
        Elbow     = 1, // локоть:    elbow
        Shldr     = 2, // плечо:     shoulder
        Clvcl     = 3, // ключица:   clavicle
        JCount    = 4,
        JInvalid  = 5
    };

    static const size_t MusclesMaxCount = (size_t)(Hand::Muscle::MCount);
    static const size_t JointsMaxCount = (size_t)(Hand::Joint::JCount);
    //----------------------------------------------------
    frames_t muscleMaxLast(IN muscle_t) const;
    frames_t muscleMaxLast(IN const Control&) const;
    //----------------------------------------------------
    static Muscle MofJ(Joint joint, bool open) { return Muscle(size_t(joint) * 2 + !open); }
    static Joint  JofM(Muscle muscle) { return Joint(size_t(muscle) / 2); }
    //----------------------------------------------------
    Muscle M(muscle_t muscle) const
    {
        if (muscle >= params.musclesUsedCount)
            return MInvalid;
        return params.musclesUsed[muscle];
    }
    Joint  J(joint_t joint) const
    {
        if (joint >= params.jointsUsedCount)
            return JInvalid;
        return params.jointsUsed[joint];
    }
    //----------------------------------------------------
    joint_t   jointsCount() const { return params.jointsUsedCount; }
    muscle_t musclesCount() const { return params.musclesUsedCount; }
    //----------------------------------------------------
    bool musclesValidUnion(Muscle m1, Muscle m2)
    { return (m1 < MCount && m2 < MCount && (m1 / 2) != (m2 / 2)); }
    //----------------------------------------------------
    struct JointInput : public Robo::JointInput
    {
        Joint             type{ Joint::JInvalid };
        Point       openCoords{};
        size_t        maxAngle{};
        frames_t maxMoveFrames{};
        double     defaultPose{};
        //MotionLaws::JointMotionLaw  frames{ nullptr, nullptr };
        //bool show;
        //[ &palm, &hand, &arm, &shoulder ]
        JointInput() : Robo::JointInput() {}
        JointInput(Joint type, const Point &openCoords, size_t maxAngle,
                   frames_t  maxMoveFrames, double defaultPose,
                   const MotionLaws::JointMotionLaw &frames, bool show) :
            type(type), openCoords(openCoords), maxAngle(maxAngle),
            maxMoveFrames(maxMoveFrames), defaultPose(defaultPose),
            Robo::JointInput(frames, show) {}
    };
    //----------------------------------------------------
    Hand(IN const Point &baseClavicle, IN const std::list<JointInput> &joints);
    Hand(IN const Point &baseClavicle, IN const std::list<std::shared_ptr<Robo::JointInput>> &joints);

protected:
    //----------------------------------------------------
    struct Params
    {
        std::array<Muscle, MusclesMaxCount> musclesUsed;
        std::array<Joint,   JointsMaxCount>  jointsUsed;
        std::array<double,  JointsMaxCount>     defOpen;

        uint8_t musclesUsedCount;
        uint8_t jointsUsedCount;

        bool dynamics;
        bool oppHandle;

        //--- draw constants ---
        bool drawPalm;
                
        double   palmRadius;
        double  jointRadius;
        double sectionWidth;

        Params(IN const std::list<JointInput> &jointInputs);
    };

    //----------------------------------------------------
    struct Status
    {
        // ---current position---
        std::array<Point, JointsMaxCount>  curPos{};  // curPos_Palm, curPos_Hand, curPos_Arm, curPos_Shldr (curPosClvcl fixed)
        std::array<double, JointsMaxCount> angles{};  // angle_Wrist, angle_Elbow, angle_Shldr, shift_Clvcl

        bool moveEnd = false; // флаг окончания движения - полной остановки

        // ---parameters of hydraulic force---
        std::array<frames_t, MusclesMaxCount> lastsMove{}; // текущая продолжительность основного движения данного мускула (индекс кадра в массиве moveFrames)
        std::array<frames_t, MusclesMaxCount> lastsStop{}; // текущая продолжительность торможения данного мускула (индекс кадра в массиве moveFrames)
        std::array<frames_t, MusclesMaxCount> lasts{};     // индекс кадра в массиве moveFrames 

        // ---frames---
        std::array<frames_t, MusclesMaxCount> musclesMove{}; // задействованные в движениии двители
        std::array<double,   MusclesMaxCount>   prevFrame{}; // величина смещения сочленения в предыдущий такт для данного мускула
        //std::array<double, JointsMaxCount>     velosity{}; // скорость сочленения в данный такт
        //std::array<double, JointsMaxCount> acceleration{}; // ускорение сочленения в данный такт

        unsigned visitedRarity = 10;
        bool windy = false;

        Status(IN const std::list<Hand::JointInput> &jointInputs)
        {
            for (auto &input : jointInputs)
                if (input.show)
                    curPos[input.type] = input.openCoords;
        }
    };

    //----------------------------------------------------
    struct Physics
    {
        //---internal phisical parameters---
        std::array<Point, JointsMaxCount + 1> jointsOpenCoords{}; // palm, hand, arm, shoulder (clavicle fixed)
        std::array<size_t, JointsMaxCount>    jointsMaxAngles{};  // maxWristAngle, maxElbowAngle, maxShldrAngle, maxClvclShift

        std::array<frames_t, JointsMaxCount>  maxMoveFrames{}; // число кадров от полного раскрытия сустава до полного закрытия
        std::array<frames_t, JointsMaxCount>  minStopFrames{}; // число кадров для остановки сустава при полном ускорении

        //---велична изменения угла в каждый кадр---
        std::array<std::vector<double>, JointsMaxCount> framesMove{}; // при движении
        std::array<std::vector<double>, JointsMaxCount> framesStop{}; // при остановке

        Physics(IN const Point &baseClavicle, IN const std::list<JointInput> &jointInputs);
    };

    //----------------------------------------------------
    Status status;
    const Params params;
    const Physics physics;
        
    //----------------------------------------------------
    bool muscleFrame (muscle_t);
    void muscleMove  (frames_t frame, muscle_t m, frames_t last);
    //----------------------------------------------------
    void  jointMove  (joint_t joint, double offset);
    //----------------------------------------------------
    frames_t  Hand::move(IN Muscle muscle, IN frames_t last);
    frames_t  Hand::move(IN Muscle muscle, IN frames_t last, OUT Trajectory &visited);
        
public:
    //----------------------------------------------------
    void      getWorkSpace(OUT Trajectory&);

    void      draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const;
    void      step(IN frames_t frame, IN muscle_t muscle = Robo::MInvalid, IN frames_t last = 0);
    frames_t  move(IN const Control &control, OUT Trajectory &visited);
    
    //----------------------------------------------------
    /*  jointsOpenPercent={ j={ Clvcl, Shldr, Elbow, Wrist }, val <= 100.0% } */
    void  setJoints(IN const JointsOpenPercent&);
    void  resetJoint(IN joint_t);
    void  reset();
    
    //----------------------------------------------------    
    bool          moveEnd() const { return status.moveEnd; }
    const Point& position() const { return status.curPos[(params.drawPalm ? Joint::Wrist : Joint::Elbow)]; }
    Point jointPos(IN joint_t joint) const
    {
        if (joint >= JCount)
            throw std::exception("Inorrect joint");
        return status.curPos[J(joint)];
    }
    //----------------------------------------------------   
    unsigned getVisitedRarity() const { return status.visitedRarity; }
    void     setVisitedRarity(unsigned rarity) { status.visitedRarity = rarity; }

    unsigned getWindy() const { return status.windy; }
    void     setWindy(bool windy) { status.windy = windy; }
    //----------------------------------------------------
    tstring name() const { return _T("Hand-v3"); }
    //----------------------------------------------------
    friend class Robo::EnvEdgesHand;
};

}
}
