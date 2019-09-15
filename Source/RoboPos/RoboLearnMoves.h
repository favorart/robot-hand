﻿#pragma once

#include "StdAfx.h"

#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"

namespace Robo {
class RoboPhysics;
}

namespace RoboPos
{
class TourI;
/* тестовые движения рукой */
//------------------------------------------------------------------------------
void  testRandom (RoboMoves::Store &store, Robo::RoboI &robo, size_t tries);
void  testCover  (RoboMoves::Store &store, Robo::RoboPhysics &robo);
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

    const TargetI &_target;
    RoboMoves::Store &_store;
    Robo::RoboI &_robo;
    Point _base_pos{};

    size_t _complexity = 0;  ///< Подсчёт сложности
    tptree _config{};

    bool use_weighted_mean{};
    Robo::distance_t side3{};
    Robo::distance_t side_decrease_step{};
    size_t _tries = 0;
    size_t _random_try = 0;

    std::shared_ptr<TourI> makeTour(int stage);
    //=============================================
    bool actionRobo(IN const Point &aim, IN const Robo::Control &controls, OUT Point &hit);
    //---------------------------------------------
    bool defineDependenceOfMuscles() {}
    bool defineStaticalPeriodsinMuscles() {}

    void  weightedMeanControls(IN  const Point &aim,
                               IN  const RoboMoves::adjacency_ptrs_t &range, // диапазон управлений с конечными точками вблизи цели
                               OUT Robo::Control &controls, // возвращает массив размера <= musclesCount, уберает все повторения мускулов -- неприменим для танка.
                               OUT Point &mid_hit); // среднее расстояние диапазона до цели 

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
    void  rundownControls(IN OUT Robo::Control &controls);
    //---------------------------------------------
    bool  rundownNextControl(IN OUT Robo::Control  &controls,
                             IN OUT size_t         &controls_curr,
                             IN OUT Robo::frames_t &velosity,
                             IN OUT Robo::frames_t &velosity_prev);
    //---------------------------------------------
    bool  rundownNextControl(IN OUT Robo::Control    &controls,
                             IN OUT std::vector<int> &lasts_changes,
                             IN OUT Robo::frames_t   &velosity);
    //---------------------------------------------
    typedef std::function<bool(const RoboMoves::Record &rec, const Point &aim)> HitPosRelToAim;
    typedef std::set<std::size_t> visited_t;
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
    size_t weightedMean(IN const Point &aim, OUT Point &hit);
    size_t rundownMDir(IN const Point &aim, OUT Point &hit);
    size_t rundownFull(IN const Point &aim, OUT Point &hit);
    size_t gradientMethod(IN const Point &aim, OUT Point &pos);

    using pAdmixMethod = size_t(LearnMoves::*)(const Point&, Point&);
    const pAdmixMethod admixes[2] = {
        &LearnMoves::weightedMean,
        &LearnMoves::gradientMethod//,
        //&LearnMoves::rundownMDir,
        //&LearnMoves::rundownFull
    };

    tstring _fn_config;
    void read_config();

public:
    LearnMoves(IN RoboMoves::Store &store, IN Robo::RoboI &robo, 
               IN const TargetI &target, IN const tstring &fn_config);
    //---------------------------------------------
    size_t complexity() const { return _complexity; };
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
    size_t testStage3(IN const Point &aim);
    //---------------------------------------------
    void save(tptree &node) const;
    void load(tptree &node);
}; // end LearnMovements

}