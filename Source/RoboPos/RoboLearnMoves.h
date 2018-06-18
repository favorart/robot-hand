#pragma once

#include "StdAfx.h"

#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "ConfigJSON.h"

//#include "RoboPosApprox.h"

namespace RoboPos
{
/* тестовые движения рукой */
//------------------------------------------------------------------------------
void  testRandom (RoboMoves::Store &store, Robo::RoboI &robo, size_t tries);
void  testCover  (RoboMoves::Store &store, Robo::RoboI &robo);
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
    static int tries;

    RoboMoves::Store &_store;
    Robo::RoboI &_robo;
    std::shared_ptr<Robo::RoboI> _p_robo_copy;
    Point _base_pos;

    RecTarget &_target;
    const double _precision; ///< Точность = 1.5 мм  
    size_t _complexity = 0;  ///< Подсчёт сложности

    ConfigJSON::StageInput1 _stage1_params;
    ConfigJSON::StageInput2 _stage2_params;
    ConfigJSON::StageInput3 _stage3_params;

    //=============================================
    bool  actionRobo(IN const Point &aim, IN const Robo::Control &controls, OUT Point &hit);
    //---------------------------------------------
    void  weightedMeanControls(IN  const Point &aim,
                               IN  const RoboMoves::adjacency_ptrs_t &range,
                               OUT Robo::Control &controls,
                               OUT double *weight = NULL);

    bool  weightedMeanULAdjs(IN  const Point   &aim, OUT RoboMoves::Record *pRec,
                             OUT Robo::Control &lower_controls,
                             OUT Robo::Control &upper_controls,
                             OUT double        &lower_distance,
                             OUT double        &upper_distance);
    //---------------------------------------------
    void  gradientControls(IN  const Point &aim, IN  double  d_d,
                           IN  const Robo::Control &inits_controls,
                           IN  const Robo::Control &lower_controls,
                           IN  const Robo::Control &upper_controls,
                           OUT       Robo::Control &controls);
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
    //---------------------------------------------
    tstring Tour1_type{};
    tstring Tour2_type{};

public:
    LearnMoves(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target);
    LearnMoves(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target,
               IN double precision,
               IN const ConfigJSON::StageInput1 &stage1,
               IN const ConfigJSON::StageInput2 &stage2,
               IN const ConfigJSON::StageInput3 &stage3);
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
    size_t  gradientMethod(IN const Point &aim);
    size_t  gradientMethod_admixture(IN const Point &aim);
    //---------------------------------------------
    void  testStage1();
    void  testStage2();
    void  testStage3(OUT Robo::Trajectory &uncovered);
    //---------------------------------------------
    void  testCoverTarget(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target);
}; // end LearnMovements

}