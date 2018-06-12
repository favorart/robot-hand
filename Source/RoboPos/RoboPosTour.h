#pragma once

#include "Robo.h"
#include "RoboPos.h"

#include "RoboMovesTarget.h"

namespace RoboPos {
//------------------------------------------------------------------------------
class Approx;

class TourI
{
public:
    using  JointsNumerator = std::function<Robo::joint_t(const Robo::joint_t njoints, Robo::joint_t joint, bool first)>;
    static JointsNumerator forward, reverse;
    static double divToMeters, divToMinutes;

protected:
    JointsNumerator  &_next_joint; ///< порядок использования сочленений
    RoboMoves::Store &_store;
    Robo::RoboI      &_robo;
    Point             _base_pos{};
    Counters          _counters{};
    size_t            _complexity = 0;

    bool _b_braking{ false };  ///< использование торможения мускулом
    bool _b_checking{ false }; ///< подсчитывать число попаданий траекторий в мишень

    double _step_distance{0.};
    Robo::frames_t _lasts_step_increment{0};

    Robo::frames_t _lasts_step_increment_thick = 2;
    Robo::frames_t _lasts_step_increment_init = 6;

    Robo::frames_t _lasts_step_braking_init = 3; // 30
    Robo::frames_t _lasts_step_braking_incr = 2;
    
    size_t _max_nested = 0;
    size_t _breakings_controls_actives = 0;
    std::vector<Robo::Actuator> _breakings_controls{};

    void  exactBreakings(IN Robo::joint_t joint, IN const Robo::Control &controls);
    void appendBreakings(IN Robo::joint_t joint, IN const Robo::Actuator &a);
    void removeBreakings(IN Robo::joint_t joint);
    void  cleanBreakings(IN Robo::joint_t joint);
        
    /// Descrete tour around all over the workspace
    /// \param[in]   joint          focus of moving on the next joint
    /// \param[in]   controls       result control to robo-move
    /// \param[out]  robo_pos_high  best _hit_ using the next joint
    virtual bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high) = 0;
    /// Straight move by robot with chosen muscles
    /// \param[in]   controls       result control to robo-move
    /// \param[out]  robo_pos       best _hit_ using the next joint
    virtual bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos);

public:
    //TourI(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN const tstring &config) :
    //    _store(store), _robo(robo), _borders(borders_t{}) {}    
    TourI(RoboMoves::Store &store, Robo::RoboI &robo, TourI::JointsNumerator &next_joint=TourI::reverse) :
        _store(store), _robo(robo), _next_joint(next_joint),
        _max_nested(_robo.jointsCount()),
        _breakings_controls(_max_nested),
        _breakings_controls_actives(0)
    {}

    size_t complexity() const { return _complexity; }

    void setIncrement(double step_distance, Robo::frames_t lasts_step_increment)
    {
        _step_distance = step_distance;
        _lasts_step_increment = lasts_step_increment;
    }

    void run();
};

//------------------------------------------------------------------------------
/// Первоначальный широкий обход всей доступной области
class TourWorkSpace : public TourI
{
public:
    TourWorkSpace(RoboMoves::Store &store, Robo::RoboI &robo, TourI::JointsNumerator &next_joint=TourI::reverse) :
        TourI(store, robo, next_joint)
    {}
    bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high);
};

//------------------------------------------------------------------------------
/// Обход всей мишени целиком
class TourTarget : public TourI
{
public:
    using TargetContain = std::function<bool(const Point&)>;
    struct BorderLasts { Robo::frames_t min_lasts, max_lasts; };
    using Borders = std::vector<BorderLasts>;

    TourTarget(IN RoboMoves::Store &store,
               IN Robo::RoboI &robo,
               IN Approx &approx,
               IN TargetI &target, // !! RM
               IN TargetContain &target_contain,
               IN TourI::JointsNumerator &next_joint = TourI::reverse);

    bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high);
    bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos);

    void setPredict(bool pred) { _b_predict = pred; }
    void setChecking(bool check) { _b_checking = check; }

private:
    TargetContain &_target_contain; ///< функция проверки принадлежности координат мишени
    Approx &_approx;                ///< интерполяция функции (x,y)остановки по управлениям мускулов
    bool _b_predict;                ///< использование интерполяции для предсказания места остановки
    bool _b_checking;               ///< проверка предсказаний основных направлений
    Borders _borders;
    TargetI &_target;

    void specifyBordersByRecord(const RoboMoves::Record &rec);
    void defineTargetBorders(distance_t side);
};

}
//------------------------------------------------------------------------------

