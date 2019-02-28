#pragma once
#include "Robo.h"

namespace Robo
{
class EnvEdges;
/*  jointsOpenPercent={ joint, 0.% <= value <= 100.% } */
using JointsOpenPercent = std::initializer_list<std::pair<joint_t, double>>;

class RoboPhysics : public RoboI
{
protected:
    struct Physics
    {
        // --- internal phisical parameters ---
        bool dynamics{ true };
        bool oppHandle{ true };

        std::array<Point, RoboI::jointsMaxCount + 1> jointsBases{};
        // --- велична перемещений в каждый кадр ---
        std::array<std::vector<distance_t>, RoboI::jointsMaxCount> framesMove{}; // при движении
        std::array<std::vector<distance_t>, RoboI::jointsMaxCount> framesStop{}; // при остановке

        Physics(const Point &base, const JointsInputsPtrs &joint_inputs);
    };
    const Physics physics;

    struct FeedBack
    {
        unsigned visitedRarity{ 10 };  // число записей в траекторию

        std::array<Point, RoboI::jointsMaxCount>      prevPos{}; // позиция сочленений в предыдущий такт
        std::array<Point, RoboI::jointsMaxCount>     velosity{}; // скорость сочленения в данный такт
        std::array<Point, RoboI::jointsMaxCount> acceleration{}; // ускорение сочленения в данный такт

        FeedBack(const std::array<Point,RoboI::jointsMaxCount+1>& basePos)
        {
            int i = 0;
            std::generate(prevPos.begin(), prevPos.end(), [&basePos, &i]() { return basePos[i++]; });
        }
        void currVelAcc(const joint_t n_joints, const std::array<Point,RoboI::jointsMaxCount+1>& curPos)
        {
            for (joint_t joint = 0; joint < n_joints; ++joint)
            {
                auto vel = curPos[joint] - prevPos[joint]; // momentum velosity
                acceleration[joint] = vel - velosity[joint]; // momentum acceleration
                velosity[joint] = vel;

                prevPos[joint] = curPos[joint];
            }
        }
        Learn::State calc() const { return Learn::State{ Point{} }; }
    };
    FeedBack feedback;

    struct Enviroment
    {
        bool windy{ false }; ///< случайное перемещение в первый такт движения мускулом
        double weather{ 1. };
        std::shared_ptr<EnvEdges> edges;  ///< удары и биение на границах
        Enviroment(std::shared_ptr<EnvEdges> edges) : edges(edges) {}
    };
    Enviroment env;

    struct Status
    {
        unsigned musclesCount{ 0 };
        unsigned jointsCount{ 0 };

        // ---current position---
        std::array<Point, RoboI::jointsMaxCount + 1>   curPos{}; /* текущая позиция сочленения */ // curPos_Palm, curPos_Hand, curPos_Arm, curPos_Shldr (curPosClvcl fixed)
        std::array<distance_t, RoboI::musclesMaxCount> shifts{}; /* смещения каждого сочления */  // angle_Wrist, angle_Elbow, angle_Shldr, shift_Clvcl

        bool moveEnd = false; // флаг окончания движения - полной остановки

        // ---parameters of hydraulic force---
        std::array<frames_t, RoboI::musclesMaxCount> lastsMove{}; // текущая продолжительность основного движения данного мускула (индекс кадра в массиве moveFrames)
        std::array<frames_t, RoboI::musclesMaxCount> lastsStop{}; // текущая продолжительность торможения данного мускула (индекс кадра в массиве stopFrames)
        std::array<frames_t, RoboI::musclesMaxCount> lasts{};     // заданная длительность работы мускула (индекс кадра в массиве moveFrames)

        // ---frames---
        std::array<frames_t, RoboI::musclesMaxCount> musclesMove{}; // задействованные в движениии двители
        std::array<distance_t, RoboI::musclesMaxCount> prevFrame{}; // величина смещения сочленения в предыдущий такт для данного мускула

        Status(const Point &base, const JointsInputsPtrs &joint_inputs);
    };
    Status status;

    Point _base() const { return physics.jointsBases[jointsCount()/*base_center*/]; }

    virtual bool somethingMoving() const;
    virtual void step(IN muscle_t muscle /*= Robo::MInvalid*/, IN frames_t lasts /*= 0*/);
    virtual void realMove() = 0;

    virtual void muscleDriveStop(muscle_t);
    virtual bool muscleDriveFrame(muscle_t);
    virtual void muscleDriveMove(frames_t frame, muscle_t muscle, frames_t last);

public:
    RoboPhysics(const Point &base /*Clavicle|Center*/,
                const JointsInputsPtrs &joint_inputs,
                const std::shared_ptr<EnvEdges> &eiges);

    muscle_t musclesCount() const { return status.musclesCount; }
    joint_t  jointsCount() const { return status.jointsCount; }

    //frames_t muscleMaxLasts(muscle_t muscle) const;
    //frames_t muscleMaxLasts(const Control &control) const;

    frames_t move(IN const Control &controls);
    frames_t move(IN const Control &controls, IN frames_t max_frames);
    frames_t move(IN const bitset_t &muscles, IN frames_t lasts);
    frames_t move(IN const bitset_t &muscles, IN frames_t lasts, IN frames_t max_frames);

    frames_t move(IN frames_t max_frames = LastsInfinity);
    frames_t move(IN const std::vector<muscle_t> &ctrls);

    void step();
    void step(IN const Control &control);
    void step(IN const Control &control, OUT size_t &control_curr);
    void step(IN const bitset_t &muscles, IN frames_t lasts);
    void step(const Robo::RoboI::bitwise &muscles);
    void reset();

    // DEBUG -- RM !!!
    virtual frames_t muscleStatus(muscle_t m) const
    { return status.musclesMove[m]; }
    virtual frames_t lastsStatus(muscle_t m) const
    { return std::max(status.lastsMove[m], status.lastsStop[m]); }
    virtual TCHAR lastsStatusT(muscle_t m) const
    {
        return (status.lastsMove[m] > status.lastsStop[m]) ? (
            (status.lastsMove[m] > 0) ? 'm' : '0') : 's';
    }

    Trajectory& traj() { return _trajectory; };
    virtual void resetJoint(joint_t) = 0;
    // -----------------------------

    const Point& jointPos(IN joint_t joint) const
    {
        if (joint >= jointsCount())
            throw std::logic_error("jointPos: Inorrect joint");
        return status.curPos[joint];
    }

    unsigned getVisitedRarity() const { return feedback.visitedRarity; }
    void     setVisitedRarity(unsigned rarity) { feedback.visitedRarity = rarity; }

    unsigned getWindy() const { return env.windy; }
    void     setWindy(bool windy) { env.windy = windy; }

    bool     moveEnd() const { return status.moveEnd; }
    const Point& position() const { return status.curPos[0]; }

    tstring getName() const { return RoboPhysics::name(); };
    static tstring name() { return _T("RoboPhysics"); };

private:
    static std::shared_ptr<RoboI> make(const tstring &type, tptree &node)
    { throw std::logic_error("Depricated"); };
    friend class EnvEdges;
};
}

