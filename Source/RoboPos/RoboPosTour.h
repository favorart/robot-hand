#pragma once

#include "Robo.h"
#include "RoboPos.h"


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
    borders_t        &_borders;
    RoboMoves::Store &_store;
    Robo::RoboI      &_robo;
    Point             _base_pos{};
    Counters          _counters{};
    size_t            _complexity = 0;

    bool _b_braking{ false };  ///< использование торможения мускулом
    bool _b_checking{ false }; ///< подсчитывать число попаданий траекторий в мишень

    double _step_distance{0.};
    Robo::frames_t _lasts_step_increment{0};

    Robo::frames_t _lasts_step_increment_thick = 20;
    Robo::frames_t _lasts_step_increment_init = 6;

    Robo::frames_t _lasts_step_braking_init = 10; // 30
    Robo::frames_t _lasts_step_braking_incr = 2;
    
    size_t _max_nested = 0;
    size_t _breakings_controls_actives = 0;
    std::vector<Robo::Actuator> _breakings_controls{};

    void exactsBreakings(IN Robo::joint_t joint, IN const Robo::Control &controls);
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
    virtual bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos) = 0;

public:
    //TourI(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN const tstring &config) :
    //    _store(store), _robo(robo), _borders(borders_t{})
    //{}    
    TourI(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN borders_t &borders,
          IN TourI::JointsNumerator &next_joint = TourI::reverse) :
        _store(store), _robo(robo), _borders(borders), _next_joint(next_joint),
        _max_nested(_robo.jointsCount()),
        _breakings_controls(_max_nested),
        _breakings_controls_actives(0)
    {}

    size_t complexity() const { return _complexity; }

    void setBorders(borders_t &borders)
    { _borders = borders; }
    void setIncrement(double step_distance, Robo::frames_t lasts_step_increment)
    {
        _step_distance = step_distance;
        _lasts_step_increment = lasts_step_increment;
    }

    void run()
    {
        _complexity = 0;
        _counters.clear();
        _max_nested = _robo.jointsCount();
        // ----------------------------------------------------
        _robo.reset();
        _base_pos = _robo.position();
        // ----------------------------------------------------
        try
        {
            if (_step_distance < DBL_EPSILON || _lasts_step_increment == 0)
                throw std::runtime_error{"Increment Params aren't set"};

            Point useless;
            runNestedForMuscle(_next_joint(_robo.jointsCount(), 0, true), Robo::Control{}, useless);
        }
        catch (boost::thread_interrupted&)
        {
            CINFO("WorkingThread interrupted");
            //return;
        }
        catch (const std::exception &e)
        {
            CERROR(e.what());
            //return;
        }
        // ----------------------------------------------------
        if (_b_checking) { _counters.print(); }
        tcout << _T("\nStep: ") << (_step_distance / TourI::divToMeters) << _T("mm.");
        tcout << _T("\nComplexity: ") << complexity();
        tcout << _T("  minutes:") << double(complexity()) / TourI::divToMinutes << std::endl;
    }
};

//------------------------------------------------------------------------------
/// Первоначальный широкий обход всей доступной области
class TourWorkSpace : public TourI
{
public:
    TourWorkSpace(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN borders_t &borders,
                  IN TourI::JointsNumerator &next_joint = TourI::reverse) :
        TourI(store, robo, borders, next_joint)
    {}
    bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos);
    bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high);
};

//------------------------------------------------------------------------------
/// Обход всей мишени целиком
class TourTarget : public TourI
{
public:
    using TargetContain = std::function<bool(const Point&)>;

    TourTarget(IN RoboMoves::Store &store,
               IN Robo::RoboI &robo,
               IN borders_t &borders,
               IN Approx &approx,
               IN TargetContain &target_contain,
               IN TourI::JointsNumerator &next_joint = TourI::reverse) :
        TourI(store, robo, borders, next_joint),
        _target_contain(target_contain),
        _approx(approx)
    {}
    bool runNestedMove(IN const Robo::Control &controls, OUT Point &robo_pos);
    bool runNestedForMuscle(IN Robo::joint_t joint, IN Robo::Control &controls, OUT Point &robo_pos_high);
    
private:
    TargetContain & _target_contain; ///< функция проверки принадлежности координат мишени
    Approx        &_approx; ///< интерполяция функции (x,y)остановки по управлениям мускулов

    bool _b_predict;  ///< использование интерполяции для предсказания места остановки
    bool _b_checking; ///< проверка предсказаний основных направлений
};

}
//------------------------------------------------------------------------------

