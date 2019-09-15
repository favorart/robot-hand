#pragma once

#include "RoboPhysics.h"

namespace Robo
{

struct RoboPhysics::Status
{
    using JointPrevs = std::array<Point, jointsMaxCount>;
    using JointPoses = std::array<Point, jointsMaxCount + 1>;
    using MuscleShifts = std::array<distance_t, RoboI::musclesMaxCount>;
    using MuscleFrames = std::array<frames_t, RoboI::musclesMaxCount>;

    /* const */ muscle_t musclesCount{ 0 };
    /* const */ joint_t jointsCount{ 0 };
    /* const */ JointPoses basePos{};

    // ---current position---
    JointPoses currPos{}; ///< текущая позиция сочленения
    MuscleShifts shifts{}; ///< смещения каждого сочления

    bool moveEnd = false; ///< флаг окончания движения - полной остановки

    // ---parameters of hydraulic force---
    MuscleFrames lastsMove{}; ///< текущая продолжительность основного движения данного мускула (индекс кадра в массиве moveFrames)
    MuscleFrames lastsStop{}; ///< текущая продолжительность торможения данного мускула (индекс кадра в массиве stopFrames)
    MuscleFrames lasts{};     ///< заданная длительность работы мускула (индекс кадра в массиве moveFrames)

    // ---frames---
    MuscleFrames musclesMove{}; ///< задействованные в движениии двители
    MuscleShifts prevFrame{}; ///< величина смещения сочленения в предыдущий такт для данного мускула

    // ---previous position---
    JointPrevs prevPos{}; ///< позиция сочленений в предыдущий такт
    JointPrevs prevVel{}; ///< мгновенная скорость сочленений в предыдущий такт

    Status(const Point &base, const JointsInputsPtrs &joint_inputs);
    void reset();
    void calcCurState(State &state, int spec);
    void getCurState(State &state, int spec) const;
};

//--------------------------------------------------------------------------------
struct RoboPhysics::EnvPhyState
{
    //frames_t visitedRarity{ 10 }; ///< писать в траекторию 1 раз в данное число тактов
    Enviroment conditions{ NOTHING };
    pEnvEdges edges; ///< удары и биение на самопересечениях и границах рабочей области

    bool momentum_happened{ false };
    frames_t momentum_n_start{ LastsInfinity };
    frames_t momentum_n_frames{ 10 };

    frames_t st_friction_n_frames{ 10 };
    distance_t st_friction_big_frame{ RoboI::minFrameMove };

    // systematic_change = []() { изменить несколько рандомных кадров framesMove или framesStop };

    // --- велична перемещений в каждый кадр ---
    using JointFrames = std::array<std::vector<distance_t>, RoboI::jointsMaxCount>;
    JointFrames framesMove{}; // при движении
    JointFrames framesStop{}; // при остановке

    EnvPhyState(const Point &base, const JointsInputsPtrs&, pEnvEdges);
    void reset();
};

}

