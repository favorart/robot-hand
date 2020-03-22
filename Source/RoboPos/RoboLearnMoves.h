#pragma once

#include "StdAfx.h"

#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"

//#define USE_MID_STAT
//#define USE_REACH_STAT


namespace Robo {
class RoboPhysics;
}

namespace RoboPos {
class TourI;
//------------------------------------------------------------------------------
/* тестовые движения рукой */
void  testApprox (RoboMoves::Store&, Robo::RoboI&);
void  testRandom (RoboMoves::Store&, Robo::RoboI&, size_t tries);
void  testCover  (RoboMoves::Store&, Robo::RoboPhysics&);
//------------------------------------------------------------------------------
enum class Admix { GradWMeans, WeightMean, GradPoints, AllRundown, DirRundown, _AllAdmixes_ };

//------------------------------------------------------------------------------


/*!  Количество точек, в окресности искомой точки.
*    Что я могу варьировать?
*   
*    v  1. Длительность работы каждого мускула
*    v  2. Время старта каждого мускула
*    v  3. Тормозить мускулом? (прерывать '1' входа на которкое время)
*   
*    ?  4. Может быть !1 перелом! траектории (полная остановка)
*    Он может нам понадобиться в зависимости от законов движения
*    разных мускулов, если один до второго не успевает... (?позже включить??)
*/
class LearnMoves
{
    static const Robo::frames_t lasts_min = 1;
    //static const Robo::frames_t start_next = 1;

    const TargetI &_target;
    RoboMoves::Store &_store;
    Robo::RoboI &_robo;
    Point _base_pos{};
    /// Подсчёт сложности
    size_t _complexity = 0;
    //size_t _gradient_points_complexity = 0;
    //size_t _gradient_wmeans_complexity = 0;
    //size_t _rundown_alldirs_complexity = 0;
    //size_t _rundown_maindir_complexity = 0;
    //size_t _wmean_complexity = 0;
    using ComplexCounters = std::array<size_t, int(Admix::_AllAdmixes_)>;
    ComplexCounters _complex{};

#ifdef USE_REACH_STAT
    ComplexCounters _random_by_admix{};
    std::vector<std::pair<ComplexCounters, bool>> _reached_by_admix{};
    int _reach_current = 0;
#endif
    void updateReachedStat(Admix admix);
    void printReachedStat();

    tptree _config{};

    bool use_weighted_mean{};
    Robo::distance_t annealing{};
    Robo::distance_t side3{};
    Robo::distance_t side_decrease_step{};
    size_t _tries = 0;
    size_t _random_try = 0;

    std::shared_ptr<TourI> makeTour(int stage);
    //=============================================
    Robo::distance_t actionRobo(IN const Point &aim, IN const Robo::Control &controls);
    //---------------------------------------------
    /*inline*/ bool less(Robo::distance_t distance, Robo::distance_t new_distance);
    /*inline*/ bool check_precision(Robo::distance_t distance);
    /*inline*/ bool check_precision(Robo::distance_t distance, const Point &aim);
    //---------------------------------------------
    Point predict(const Robo::Control &controls);
    //---------------------------------------------
    bool defineDependenceOfMuscles() {}
    bool defineStaticalPeriodsinMuscles() {}
    //---------------------------------------------
#ifdef USE_MID_STAT
    struct MidHitStat;
    MidHitStat *mid_hit_stat{}, *mid_hit_stat1{};
    void append(MidHitStat *mhs, Robo::distance_t d);
#endif
    //---------------------------------------------
    void weightedMeanControls(IN  const Point &aim,
                              IN  const RoboMoves::adjacency_ptrs_t &range, // диапазон управлений с конечными точками вблизи цели
                              OUT Robo::Control &controls, // возвращает массив размера <= musclesCount, уберает все повторения мускулов -- неприменим для танка.
                              OUT Point &mid_hit);         // среднее расстояние диапазона до цели
    void weightedMeanControlsOrdered(IN const Point &aim, IN const RoboMoves::adjacency_ptrs_t &range,
                                     OUT std::vector<Robo::Actuator> &controls, OUT Point &mid_hit);

    bool  weightedMeanULAdjs(IN  const Point   &aim,
                             OUT RoboMoves::Record *pRec,
                             OUT Robo::Control &lower_controls,
                             OUT Robo::Control &upper_controls,
                             OUT Robo::distance_t &delta);
    //---------------------------------------------
    void  gradientControls(IN  const Point &aim, IN  double  d_d,
                           IN  const Robo::Control &inits_controls,
                           IN  const Robo::Control &lower_controls,
                           IN  const Robo::Control &upper_controls,
                           OUT       Robo::Control &controls);
    void gradientControlsNew(IN const Point   &aim, IN  double d_d,
                             IN const Robo::Control &inits_controls,
                             IN const Robo::Control &lower_controls,
                             IN const Robo::Control &upper_controls,
                             OUT      Robo::Control &controls);
    //---------------------------------------------
    class RundownAllDirsIncrementor;
    using change_t = int8_t; // 1,0,-1
    using changes_t = std::vector<change_t>;
    //---------------------------------------------
    Robo::frames_t rundownVelosity(Robo::distance_t);
    //---------------------------------------------
    void  rundownControls(IN OUT Robo::Control &controls);
    //---------------------------------------------
    bool  rundownNextControl(IN OUT Robo::Control  &controls,
                             IN OUT size_t         &controls_curr,
                             IN OUT Robo::frames_t &velosity,
                             IN OUT Robo::frames_t &velosity_prev);
    //---------------------------------------------
    bool  rundownNextControl(IN OUT Robo::Control    &controls,
                             IN OUT changes_t        &lasts_changes,
                             IN OUT Robo::frames_t    velosity,
                             std::vector<int> &muscles_repeats);
    //---------------------------------------------
    using HitPosRelToAim = std::function<bool(const RoboMoves::Record&)>;
    using visited_t = std::set<std::size_t>;
    //---------------------------------------------
    const RoboMoves::Record*
    gradientClothestRecord(IN const RoboMoves::adjacency_ptrs_t &range,
                           IN const Point           &aim,
                           IN const HitPosRelToAim  *pHitPosPred = NULL,
                           IN OUT   visited_t       *pVisited = NULL);
    //---------------------------------------------
    bool
    gradientSomeClothestRecords(IN  const      Point  &aim,
                                OUT RoboMoves::Record *pRecClose,
                                OUT RoboMoves::Record *pRecLower,
                                OUT RoboMoves::Record *pRecUpper,
                                IN OUT visited_t      *pVisited = NULL);
    //=============================================
    /* Mixtures */
    Robo::distance_t weightedMean   (IN const Point &aim);
    Robo::distance_t rundownAllDirs (IN const Point &aim);
    Robo::distance_t rundownMainDir (IN const Point &aim);
    Robo::distance_t gradientMethod (IN const Point &aim);

    using pAdmixMethod = Robo::distance_t(LearnMoves::*)(const Point&);
    const pAdmixMethod admixes[3] = {
        &LearnMoves::weightedMean,
        &LearnMoves::gradientMethod,
        //&LearnMoves::rundownMainDir,
        &LearnMoves::rundownAllDirs
    };

    tstring _fn_config;
    void read_config();

public:
    LearnMoves(IN RoboMoves::Store &store, IN Robo::RoboI &robo, 
               IN const TargetI &target, IN const tstring &fn_config);
    ~LearnMoves();
    //---------------------------------------------
    size_t complexity() const { return _complexity; };
    //---------------------------------------------
    RoboMoves::ApproxFilter getApproxRangeFilter(Robo::distance_t side=0, size_t pick_points=3) const;
    //---------------------------------------------
    /// грубое покрытие всего рабочего пространства
    void  STAGE_1();
    /// Покрытие всей мишени не слишком плотно
    void  STAGE_2();
    /// Попадание в оставшиеся непокрытыми точки мишени
    void  STAGE_3(OUT Robo::Trajectory &uncovered);
    //---------------------------------------------
    void  uncover(OUT Robo::Trajectory &uncovered);
    //---------------------------------------------
    Robo::distance_t testStage3(IN const Point &aim);
    //---------------------------------------------
    void save(tptree &node) const;
    void load(tptree &node);
}; // end LearnMovements

}