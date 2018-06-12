#pragma once

#include "Robo.h"
#include "RoboEdges.h"


namespace Robo {
class EnvEdges;
class EnvEdgesTank;
namespace Mobile {

#define TANK_VER 1
#define TANK_DEBUG

//------------------------------------------------------------------------------
class Tank : public RoboI
{
public:
    //----------------------------------------------------
    enum Muscle : uint8_t
    { /* Frw - forward,
       * Bck - backward
       */
        LTrackFrw = 0,
        LTrackBck = 1,
        RTrackFrw = 2,
        RTrackBck = 3,
        MCount = 4,
        MInvalid = 5
    };

    //----------------------------------------------------
    enum Joint : uint8_t
    { LTrack = 0, RTrack = 1, JCount = 2, JInvalid = 3 };

    static const size_t MusclesMaxCount = (size_t)(Tank::Muscle::MCount);
    static const size_t JointsMaxCount = (size_t)(Tank::Joint::JCount);
    //----------------------------------------------------
    frames_t muscleMaxLast(muscle_t muscle) const;
    frames_t muscleMaxLast(const Control &control) const;
    //----------------------------------------------------
    static Muscle MofJ(Joint joint, bool open) { return Muscle(size_t(joint) * 2 + !open); }
    static Joint  JofM(Muscle muscle) { return Joint(size_t(muscle) / 2); }
    //----------------------------------------------------
    Muscle M(muscle_t muscle) const
    {
        if (muscle >= musclesCount())
            return Muscle::MInvalid;
        return params.musclesUsed[muscle];
    }
    Joint  J(joint_t joint) const
    {
        if (joint >= jointsCount())
            return Joint::JInvalid;
        return params.jointsUsed[joint];
    }
    //----------------------------------------------------
    muscle_t musclesCount() const { return MusclesMaxCount; }
    joint_t  jointsCount() const { return JointsMaxCount; }
    //----------------------------------------------------
    struct JointInput : public Robo::JointInput
    {
        Joint  type{ Joint::JInvalid };
        Point  base{};
        size_t nMoveFrames{};
        double maxMoveFrame{};
        // [ LTrack, Center, RTrack ]
        JointInput() : Robo::JointInput() {}
        JointInput(Joint type, const Point &base, size_t nMoveFrames, double maxMoveFrame,
                   const MotionLaws::JointMotionLaw &frames, bool show) :
            type(type), base(base), nMoveFrames(nMoveFrames), maxMoveFrame(maxMoveFrame),
            Robo::JointInput(frames, show) {}
    };
    //----------------------------------------------------
    Tank(IN const Point &baseCenter, IN const std::list<JointInput> &joints);
    Tank(IN const Point &baseCenter, IN const std::list<std::shared_ptr<Robo::JointInput>> &joints);

protected:
    //----------------------------------------------------
    bool    muscleFrame (Muscle muscle, bool atStop);
    void    muscleMove  (frames_t frame, Muscle muscle, frames_t last);

    void    realMove();

    frames_t  Tank::move(muscle_t muscle, IN frames_t last, OUT Trajectory &visited);
    frames_t  Tank::move(muscle_t muscle, IN frames_t last);
    //----------------------------------------------------
    struct Params
    {
        Muscle musclesUsed[MusclesMaxCount];
        Joint   jointsUsed[JointsMaxCount];

        uint8_t musclesUsedCount;
        uint8_t jointsUsedCount;

        bool dynamics;
        bool oppHandle;

        //--- draw constants ---
        double   trackWidth;
        double  trackHeight;
        double   bodyHeight;
        double centerRadius;

        Params(IN const std::list<Tank::JointInput> &jointInputs);
    };

    //----------------------------------------------------
    struct Status
    {
        //---current position---
        std::array<Point, JointsMaxCount+1> curPos{}; /* текущая позиция сочленения */
        std::array<double, MusclesMaxCount> shifts{}; /* смещения каждого сочления, чтобы двигать ими вместе */

        bool moveEnd = false; /* флаг окончания движения - полной остановки */

        //---parameters of hydraulic force---
        std::array<frames_t, MusclesMaxCount> lastsMove{}; /* индекс в массиве движения */
        std::array<frames_t, MusclesMaxCount> lastsStop{}; /* индекс в массиве остановки */
        std::array<frames_t, MusclesMaxCount> lasts{};     /* заданная длительность работы мускула */

        std::array<frames_t, MusclesMaxCount> musclesMove{}; /* задействованные мускулы (хотя для принудительного торможения) */
        std::array<double, MusclesMaxCount>     prevFrame{}; /* перемещение в предыдущий фрэйм */
        //std::array<double, JointsMaxCount>     velosity{}; 
        //std::array<double, JointsMaxCount> acceleration{};

        unsigned visitedRarity{};        /* число записей в траекторию */
        bool windy{};                    /* случайное перемещение в первый такт движения мускулом */
        std::shared_ptr<EnvEdges> edges; /* удары и биение на границах */

        Status(IN const std::list<Tank::JointInput> &jointInputs);
    };

#ifdef TANK_DEBUG
    Point center_;
    double r1_, r2_;
#endif
    //----------------------------------------------------
    struct Physics
    {
        //---internal phisical parameters---
        std::array<Point, JointsMaxCount+1> jointsBases{};
        double maxMoveFrame{};

        //---велична изменения угла в каждый кадр
        std::array<std::vector<double>, JointsMaxCount> framesMove{}; // при движении
        std::array<std::vector<double>, JointsMaxCount> framesStop{}; // при остановке

        Physics(IN const Point &baseCenter, IN const std::list<Tank::JointInput> &jointInputs);
    };

    Status status;
    const Params params;
    const Physics physics;
    
public:

    void      getWorkSpace(OUT Trajectory &workSpace);

    void      draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const;

    void      step(IN frames_t frames, IN muscle_t muscle = Robo::MInvalid, IN frames_t last = 0);
    frames_t  move(IN const Control &control, OUT Trajectory &visited);

    void  reset();
    void  resetJoint(IN joint_t);
    void  setJoints(IN const JointsOpenPercent&);
    
    bool           moveEnd() const { return status.moveEnd; }
    const Point&  position() const
    { 
        //Point center = (status.curPos[Joint::LTrack] + status.curPos[Joint::RTrack]) / 2.;
        //return center;
        return status.curPos[Joint::JCount];
    }
    Point jointPos(joint_t joint) const { return status.curPos[J(joint)]; }

    unsigned getVisitedRarity() const { return status.visitedRarity; }
    void     setVisitedRarity(unsigned rarity) { status.visitedRarity = rarity; }

    unsigned getWindy() const { return status.windy; }
    void     setWindy(bool windy) { status.windy = windy; }
    //----------------------------------------------------
    tstring name() const { return _T("Tank-v1"); }
    //----------------------------------------------------
    friend class Robo::EnvEdgesTank;
};

}
}
//------------------------------------------------------------------------------

