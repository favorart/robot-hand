#pragma once

#include "RoboPhysics.h"

namespace Robo {
class RoboPhysics;
using JointFrames = std::vector<distance_t>;
//--------------------------------------------------------------------------------
struct RoboPhysics::Status
{
    using JointPrevs = std::array<Point, jointsMaxCount>;
    //using JointPoses = std::array<Point, jointsMaxCount + 1>;
    using JointPoses = std::vector<Point>;
    using MuscleShifts = std::array<distance_t, RoboI::musclesMaxCount>;
    using MuscleFrames = std::array<frames_t, RoboI::musclesMaxCount>;

    const joint_t jointsCount{ 0 };
    const muscle_t musclesCount{ 0 };
    const JointPoses basePos{};

    // ---current position---
    JointPoses currPos{}; ///< текущая позиция сочленения
    MuscleShifts shifts{}; ///< смещения каждого сочления

private:
    bool _moveEnd = false; ///< флаг окончания движения - полной остановки

    // ---parameters of hydraulic force---
    MuscleFrames lastsMove{}; ///< текущая продолжительность основного движения данного мускула (индекс кадра в массиве moveFrames)
    MuscleFrames lastsStop{}; ///< текущая продолжительность торможения данного мускула (индекс кадра в массиве stopFrames)
    MuscleFrames lasts{};     ///< заданная длительность работы мускула (индекс кадра в массиве moveFrames)

    // ---frames---
    MuscleFrames musclesMove{}; ///< задействованные в движениии двители (номер посчитанного для мускула такта == _frame или _frame+1)
    MuscleShifts prevFrame{}; ///< величина смещения сочленения в предыдущий такт для данного мускула

    // ---previous position---
    JointPrevs prevPos{}; ///< позиция сочленений в предыдущий такт
    JointPrevs prevVel{}; ///< мгновенная скорость сочленений в предыдущий такт

public:
    using PreparedBasePoints = std::vector<Point>;
    static PreparedBasePoints prepareBasePoses(const Point &base, const JointsInputsPtrs&);

    Status(PreparedBasePoints bases);
    void reset();
    void updateState();
    void getCurState(State &state, int spec) const;
    const Point& curPos(joint_t joint) const { return currPos[joint]; }

    //using Cmp = std::function<bool(distance_t, distance_t)>;
    //distance_t muscleDriveFrame(muscle_t, const JointFrames &frames, Cmp cmp);

    void musclesAllDrivesStop();
    void muscleDriveStop(muscle_t);
    void muscleDriveMove(frames_t frame, muscle_t muscle, frames_t last, RoboPhysics &self);
    bool muscleDriveFrame(muscle_t, RoboPhysics&);

    frames_t movingOnFrame(muscle_t m) const { return lastsMove[m]; }
    bool movingOn(muscle_t m) const { return (lastsMove[m] > 0); }
    bool movingOff(muscle_t m) const { return (lastsStop[m] > 0); }
    bool somethingMoving() const { return !(ba::all_of_equal(musclesMove, 0)); }
    bool muscleMoveIsFrame(muscle_t m, frames_t _frame) const;

    //bool changes(muscle_t, joint_t, distance_t &Frame, ENV env);
    //void step(muscle_t m, IN frames_t lasts, frames_t _frame, RoboPhysics &self);
    void step(const bitwise &muscles, RoboPhysics &self);

    // edges
    void damping(muscle_t, distance_t);
    bool jointMovingOn(muscle_t mo, muscle_t mc) const { return (musclesMove[mo] > 0 || musclesMove[mc] > 0); }

    bool moveEnd() const { return _moveEnd; }

#ifdef DEBUG_RM
    frames_t muscleStatus(muscle_t m) const;
    frames_t lastsStatus(muscle_t m) const;
    TCHAR lastsStatusT(muscle_t m) const;
#endif // DEBUG_RM
};
//--------------------------------------------------------------------------------
struct RoboPhysics::EnvPhyState
{
    using MLaws = std::array<MotionLaws::JointMotionLaw, RoboPhysics::jointsMaxCount>;
    using JMaxFrames = std::array<distance_t, RoboPhysics::jointsMaxCount>;
    using JMass = std::array<double, RoboPhysics::jointsMaxCount + 1>;

    JMass mass{}; // m_j0, m_j1, m_j2, m_j3, m_j4

    // --- 0) Общие ограничения
    distance_t c1 = 0., c2 = 0;      // accelerate_limit: c1 * t + c2
    distance_t velosity_limit = 0.;  // velosity_limit:   c3

    //frames_t visitedRarity{ 10 }; ///< писать в траекторию 1 раз в данное число тактов

    MLaws laws{};                                 ///< "закон движения" каждого сочленения
    JMaxFrames max_frames{};                      ///< максимальная величина смещения в "законе движения" для сочленения

    // --- велична перемещений в каждый кадр ---
    using AllJointsFrames = std::array<JointFrames, RoboI::jointsMaxCount>;
    AllJointsFrames framesMove{}; ///< кадры при движении (дельта прироста)
    AllJointsFrames framesStop{}; ///< кадры при остановке (дельта прироста)

    Robo::Enviroment conditions{ Robo::Enviroment::NOTHING }; ///< включений моделирований условий внутренней и внешней среды

    // --- виды "граничных условий"
    pEnvEdges   edges{};                          ///< удары и биение при самопересечениях и на границах рабочей области
    frames_t   st_friction_n_frames{ 11 };        ///< количество кадров задержки начала движения из-за трения в сочленениях
    JMaxFrames st_friction_big_frame{ /*???*/ };  ///< взрывное ускорение после сильной задержки трением внутри пневматики

    // --- 4) Выбросы 
    frames_t momentum_expect_happens{ 1 /*%*/ };  ///< вероятность появление мгновенного изменения в работе мускула
    unsigned momentum_happened{ 0 };              ///< число появлений мгновенного изменения за одно движение модели
    unsigned momentum_max_happens{ 2 };           ///< максимальное число возможных появлений мгновенных изменений модели
    
    template <typename RandGen  = std::mt19937, //std::default_random_engine, //
              typename T        = distance_t,
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
    public:
        SystematicChanges(unsigned seed = std::random_device{}() /* true random value */, T left = 0., T right = 1.)
            : gen(seed), dis(left, right) {}
        T get() { return dis(gen); }
    };
    std::shared_ptr<SystematicChanges<>> systematic_changes; ///< изменения в пределах 1/2 и 2 раза, масштабирующий коэффициент.
    distance_t systematic_factor{ 1. }; ///< полученный случайным образом масштабирующий множитель кадров framesMove или framesStop

    class FrequencyComponent
    {
        distance_t c5(frames_t t) {} // передаточный коэффициент
        distance_t c6(frames_t t) {} // коэффициент затухания
        distance_t c7(frames_t t) {} // коэффициент масштаба времени

        distance_t f(frames_t t)
        {
            auto t_scale = c7(t);
            auto damping = c6(t);
            auto gear = c5(t);

            auto w = sqrt(1. - damping * damping);
            auto A = 1. / w;
            auto a = damping / t_scale;
            auto phy = atan(w / damping);

            w /= t_scale;

            return gear * (1. - A * exp(a * t) * sin(w * t + phy));
        }
    };

    EnvPhyState(/*const Point &base,*/ const JointsInputsPtrs&, pEnvEdges);
    void printFrames(joint_t) const;
    void reset();

    frames_t nFramesAll(joint_t j) const { return (framesMove[j].size() + framesStop[j].size()); }
    frames_t nFramesMove(joint_t j) const { return framesMove[j].size(); }
    frames_t nFramesStop(joint_t j) const { return framesStop[j].size(); }
};
} // end namespace Robo

