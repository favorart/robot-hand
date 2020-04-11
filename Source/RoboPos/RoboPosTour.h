#pragma once

#include "Robo.h"
#include "RoboMovesStore.h"
#include "RoboMovesTarget.h"

namespace RoboPos {
struct Counters;
//------------------------------------------------------------------------------
enum LastsIncrement
{
    LI_STEP_INIT,  ///< начальное значение длительности работы мускула (основного)
    LI_STEP,       ///< шаг изменения длительности работы одного мускула в течении одной непрерывной операции, согласно step_distance
    LI_BREAK_INIT, ///< начальное значение длительности работы мускула торможения
    LI_BREAK,      ///< шаг изменения длительности работы мускула торможения, согласно step_distance
    LI_ONTARGET,   ///< шаг изменения длительности работы, согласно step_distance на мишени
    _LI_LAST_,
    _LI_N_
};
using LastsIncrs = std::array<Robo::frames_t, _LI_LAST_>;
//------------------------------------------------------------------------------
class TourI
{
public:
    using JointsNumerator = std::function<Robo::joint_t(Robo::joint_t n_joints,
                                                        Robo::joint_t cur_joint,
                                                        bool first)>;
    static const JointsNumerator forward, reverse;
    static const Robo::distance_t divToMiliMeters;
    static const double divToMinutes;

    TourI(RoboMoves::Store *store, Robo::RoboI *robo, const tptree *config, const TourI::JointsNumerator next_joint = TourI::reverse);
    size_t complexity() const { return _complexity; }
    void run();
    void setPrecision(Robo::distance_t step_distance, Robo::frames_t lasts_step_increment)
    {
        _step_distance = step_distance;
        _lasts_step_increment = lasts_step_increment;
    }
    void setBrakings(bool brake) { _b_braking = brake; }
    void setSimulMoves(bool simul) { _b_simul = simul; }

protected:
    const tptree _config;
    const static Robo::frames_t too_long = 10000;
    const JointsNumerator _next_joint; ///< порядок использования сочленений

    const std::shared_ptr<RoboMoves::Store> _store;   ///< БД движений
    const std::shared_ptr<Robo::RoboI> _robo;         ///< Модель робота
    const std::shared_ptr<Counters> _counters;        ///< счётчики попаданий в цель

    bool _b_braking{ true };            ///< использование торможения мускулом
    bool _b_simul{ true };              ///< использование мускулов одновременно
    bool _b_starts{ true };             ///< варьировать начальные моменты
    
    unsigned       _lasts_step_n = 5;                   ///< число диапазонов, по которым будем делать усреднение lasts_step_increment-ов
    Robo::frames_t _lasts_step_increment = 0;           ///< шаг изменения длительности работы одного мускула в течении одной непрерывной операции
    Robo::frames_t _lasts_step_increment_init = 9;      ///< начальное значение длительности работы мускула (основного)
    Robo::frames_t _lasts_step_braking_init = 5;        ///< начальное значение длительности работы мускула торможения
    Robo::frames_t _lasts_step_braking_incr = 2;        ///< шаг изменения длительности работы мускула торможения
    Robo::frames_t _lasts_step_on_target = 7;           ///< шаг изменения длительности работы мускула торможения

    Robo::distance_t _step_distance = 0.;               ///< желаемая дистанция между попаданиями робота в всех направлениях    
    size_t _complexity = 0;                             ///< сложность, выраженная в числе движений руки
    size_t _max_nested = 0;                             ///< число сочленений робота
    size_t _breakings_controls_actives = 0;             ///< число инициализированных торможений
    std::vector<Robo::Actuator> _breakings_controls{};  ///< управления торможений противоположным мускулом

    LastsIncrs _lasts_step_increments = { 9, 50, 2, 5, 0 };
    Robo::frames_t lstep(LastsIncrement li) const { return _lasts_step_increments[li]; }

    class AvgLastsIncrement;
    std::shared_ptr<TourI::AvgLastsIncrement> _p_avg_lasts;

    void  exactBreakings(IN Robo::joint_t joint, IN const Robo::Control &controls);
    void appendBreakings(IN Robo::muscle_t m);
    bool removeBreakings(IN Robo::muscle_t m);
    void  cleanBreakings(IN Robo::joint_t joint);
    /// Адаптивное (в отношении к желамой величине шага попаданий)
    /// изменение длительности работы мускула и задание торможений
    /// противоположным мускулом сочленения
    void adaptiveLasts(IN const Point &prev_pos, IN const Point &curr_pos,
                       IN const Robo::Actuator &control_i, IN OUT Robo::frames_t &lasts_step,
                       IN bool target_contain = true, IN bool was_on_target = true);

    void adaptiveAvgLasts(IN const Point &prev_pos, IN const Point &curr_pos,
                          IN const Robo::Actuator &control_i, IN OUT Robo::frames_t &lasts_step,
                          IN bool target_contain, IN bool was_on_target, IN bool init);
        
    /// Descrete tour around all over the workspace
    /// \param[in]   joint          focus of moving on the next joint
    /// \param[in]   controls       result control to robo-move
    /// \param[out]  robo_pos_high  best _hit_ using the next joint
    virtual bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &avg_pos) = 0;
    /// Straight move by robot with chosen muscles
    /// \param[in]   controls       result control to robo-move
    /// \param[out]  robo_pos       best _hit_ using the next joint
    virtual bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_hit);
    virtual void printParameters() const = 0;
};

//------------------------------------------------------------------------------
/// Первоначальный широкий обход всей доступной области
class TourWorkSpace : public TourI
{
public:
    TourWorkSpace(RoboMoves::Store *store, Robo::RoboI *robo, const tptree *config, const TourI::JointsNumerator next_joint = TourI::reverse);
    bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &avg_pos);
protected:
    //std::vector<Robo::frames_t> avg_speed_on_start_for_muscle;
    //std::vector<Robo::frames_t> avg_speed_changes_for_muscle;
    virtual void printParameters() const;
};

//------------------------------------------------------------------------------
/// Обход всей мишени целиком
class TourTarget : public TourI
{
public:
    using  TargetContain = std::function<bool(const Point&)>;
    struct TargetBorderLasts { Robo::frames_t min_lasts, max_lasts; };
    using  TargetBorders = std::vector<TargetBorderLasts>;

    TourTarget(IN RoboMoves::Store *store,
               IN Robo::RoboI *robo,
               IN const tptree *config,
               IN const TargetI &target,
               IN const TargetContain &target_contain,
               IN const TourI::JointsNumerator next_joint = TourI::reverse);

    bool runNestedForMuscle(IN Robo::joint_t, IN Robo::Control&, OUT Point &robo_pos_high);
    bool runNestedMove(IN const Robo::Control&, OUT Point &robo_pos);

    void setPredict(bool pred) { _b_predict = pred; }
    void setChecking(bool check) { _b_checking = check; }

protected:
    bool _b_predict{};                    ///< использование интерполяции для предсказания места остановки
    bool _b_checking{};                   ///< проверка предсказаний основных направлений
    TargetBorders _target_borders{};      ///< границы длительности мускуов, в которые помещается мишень
    const TargetI &_target;               ///< форма мишени
    const TargetContain &_target_contain; ///< настраиваемая функция проверки принадлежности координат мишени

    void specifyBordersByRecord(const RoboMoves::Record &rec);
    void defineTargetBorders(Robo::distance_t side);
    virtual void printParameters() const;
};
} // end RoboPos

