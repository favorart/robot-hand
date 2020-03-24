#pragma once

#include "RoboPhysics.h"

namespace Robo {
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
    using MLaws = std::array<MotionLaws::JointMotionLaw, RoboPhysics::jointsMaxCount>;
    //frames_t visitedRarity{ 10 }; ///< писать в траекторию 1 раз в данное число тактов
    Robo::Enviroment conditions{ Robo::Enviroment::NOTHING }; ///< уловия внешней среды
    pEnvEdges edges; ///< удары и биение на самопересечениях и границах рабочей области
    MLaws laws; ///< закон движения каждого сочленения

    bool momentum_happened{ false }; ///< ???
    frames_t momentum_n_start{ LastsInfinity }; ///< ???
    frames_t momentum_n_frames{ 10 }; ///< ???

    frames_t st_friction_n_frames{ 10 }; ///< количество кадров задержки начала движения из-за трения в сочленениях
    distance_t st_friction_big_frame{ RoboI::minFrameMove }; ///< ???

    // --- велична перемещений в каждый кадр ---
    using JointFrames = std::array<std::vector<distance_t>, RoboI::jointsMaxCount>;
    JointFrames framesMove{}; ///< кадры при движении (дельта прироста)
    JointFrames framesStop{}; ///< кадры при остановке (дельта прироста)

    EnvPhyState(const Point &base, const JointsInputsPtrs&, pEnvEdges);
    void reset();
    frames_t nFramesAll(joint_t j) const { return (framesMove[j].size() + framesStop[j].size()); }
    frames_t nFramesMove(joint_t j) const { return framesMove[j].size(); }
    frames_t nFramesStop(joint_t j) const { return framesStop[j].size(); }

    // systematic_change = []() { изменить несколько рандомных кадров framesMove или framesStop };
    template <typename RandGen  = std::mt19937, 
              typename T        = double,
              typename Distr    = std::uniform_real_distribution<T>>
    class SystematicChanges
    {
        RandGen gen;
        Distr dis;
        /**@  Загрузить большое (относительно) количество шума
         *    Под шумом понимается некоторое количество равномерно распределённых случайных чисел из интервала.
         *    Класс random_device с помощью внешнего источника генерирует начальные данные для последующего их использования в генераторе
         *    Генератор псевдо-случайных чисел (Вихрь Мерсенна) принимает random_device (во всей программе делается 1 раз)
         *    Интервал, в котором набираются случайные числа: от 0.0 до 1.0 — для шумов более, чем достаточно.
         */
        SystematicChanges(T left = 0.0f, T right = 1.0f) :
            gen(RandGen{ std::random_device{} }),
            dis(Distr{ left, right })
        {}
        T get() const { return dis(gen); }
    };
};
} // end Robo

