#pragma once
#include "Robo.h"


namespace Robo
{
using JointsOpenPercent = std::initializer_list<std::pair<joint_t, double>>;
using JointsInputs = std::list<Robo::JointInput>;
using JointsPInputs = std::list<std::shared_ptr<Robo::JointInput>>;
using JInSpecFunc = std::function<void(const JointInput&)>;

struct RoboPhysics : public RoboI
{
protected:
    struct Physics
    {
        // --- internal phisical parameters ---
        bool dynamics{ true };
        bool oppHandle{ true };

        std::array<Point, RoboI::jointsMaxCount + 1> jointsBases{};
        // --- велична перемещений в каждый кадр ---
        std::array<std::vector<double>, RoboI::jointsMaxCount> framesMove{}; // при движении
        std::array<std::vector<double>, RoboI::jointsMaxCount> framesStop{}; // при остановке

        Physics(const Point &base, const JointsPInputs &joint_inputs);
    };
    const Physics physics;

    struct FeedBack
    {
        unsigned visitedRarity{ 1 };  // число записей в траекторию

        std::array<Point, RoboI::jointsMaxCount>    direction{}; // направление перемещения
        std::array<Point, RoboI::jointsMaxCount>     velosity{}; // скорость сочленения в данный такт
        std::array<Point, RoboI::jointsMaxCount> acceleration{}; // ускорение сочленения в данный такт

        void calcVelosity()
        {
            /// TODO:
        }
        void calcAccel()
        {
            /// TODO:
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
        std::array<Point, RoboI::jointsMaxCount + 1>  curPos{}; /* текущая позиция сочленения */ // curPos_Palm, curPos_Hand, curPos_Arm, curPos_Shldr (curPosClvcl fixed)
        std::array<double, RoboI::jointsMaxCount> shifts{}; /* смещения каждого сочления */  // angle_Wrist, angle_Elbow, angle_Shldr, shift_Clvcl

        bool moveEnd = false; // флаг окончания движения - полной остановки

        // ---parameters of hydraulic force---
        std::array<frames_t, RoboI::musclesMaxCount> lastsMove{}; // текущая продолжительность основного движения данного мускула (индекс кадра в массиве moveFrames)
        std::array<frames_t, RoboI::musclesMaxCount> lastsStop{}; // текущая продолжительность торможения данного мускула (индекс кадра в массиве stopFrames)
        std::array<frames_t, RoboI::musclesMaxCount> lasts{};     // заданная длительность работы мускула (индекс кадра в массиве moveFrames)

        // ---frames---
        std::array<frames_t, RoboI::musclesMaxCount> musclesMove{}; // задействованные в движениии двители
        std::array<double, RoboI::musclesMaxCount>   prevFrame{}; // величина смещения сочленения в предыдущий такт для данного мускула

        Status(const JointsPInputs &joint_inputs)
        {
            for (auto &j_in : joint_inputs)
                if (j_in->show)
                {
                    ++jointsCount;
                    curPos[j_in->type] = j_in->base;
                }
            musclesCount = RoboI::musclesPerJoint * jointsCount;
        }
    };
    Status status;
    
    bool muscleFrame(muscle_t);
    void muscleMove(frames_t frame, muscle_t m, frames_t last);

public:
    RoboPhysics(const Point &base /*Clavicle|Center*/,
                const JointsPInputs &joint_inputs,
                const std::shared_ptr<EnvEdges> &eiges);

    muscle_t musclesCount() const { return status.musclesCount; }
    joint_t  jointsCount() const { return status.jointsCount; }

    //frames_t muscleMaxLast(muscle_t muscle) const;
    //frames_t muscleMaxLast(const Control &control) const;

    void step(frames_t frame, muscle_t muscle = Robo::MInvalid, frames_t last = 0);
    void step(frames_t frame, const Learn::Act &act);

    frames_t move(const Control &control, Trajectory &visited);
    frames_t move(muscle_t muscle, frames_t last, Trajectory &visited);
    frames_t move(muscle_t muscle, frames_t last);
    
    Point jointPos(IN joint_t joint) const
    {
        if (joint >= jointsCount())
            throw std::logic_error("Inorrect joint");
        return status.curPos[joint];
    }

    unsigned getVisitedRarity() const { return feedback.visitedRarity; }
    void     setVisitedRarity(unsigned rarity) { feedback.visitedRarity = rarity; }

    unsigned getWindy() const { return env.windy; }
    void     setWindy(bool windy) { env.windy = windy; }

    bool     moveEnd() const { return status.moveEnd; }

    const Point& position() const { return status.curPos[0]; }

    friend class EnvEdges;
};
}
