#pragma once

#include "StdAfx.h"

#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "ConfigJSON.h"


namespace RoboPos
{
/*  Количество точек, в окресности искомой точки.
*  Что я могу варьировать?
*
*  v  1. Длительность работы каждого мускула
*  v  2. Время старта каждого мускула
*  v  3. Тормозить мускулом? (прерывать '1' входа на которкое время)
*
*  ?  4. Может быть !1 перелом! траектории (полная остановка)
*  Он может нам понадобиться в зависимости от законов движения
*  разных мускулов, если один до второго не успевает... (?позже включить??)
*/
class LearnMoves
{
    RoboMoves::Store &store;
    Robo::RoboI &robo;
    Point base_pos;

    RecTarget &target;
    const double precision; ///< Точность = 1.5 мм  
    size_t complexity = 0;  ///< Подсчёт сложности

    ConfigJSON::StageInput1 stage1;
    ConfigJSON::StageInput2 stage2;
    ConfigJSON::StageInput3 stage3;

    //==============================================
    bool  actionRobo(IN const Point &aim, IN const Robo::Control &controls, OUT Point &hit);
    //------------------------------------------------------------------------------
    void  weightedMeanControls(IN  const Point &aim,
                               IN  const RoboMoves::adjacency_ptrs_t &range,
                               OUT Robo::Control &controls,
                               OUT double *weight = NULL);

    bool  weightedMeanULAdjs(IN  const Point   &aim, OUT RoboMoves::Record *pRec,
                             OUT Robo::Control &lower_controls,
                             OUT Robo::Control &upper_controls,
                             OUT double        &lower_distance,
                             OUT double        &upper_distance);
    //------------------------------------------------------------------------------
    void  gradientControls(IN  const Point &aim, IN  double  d_d,
                           IN  const Robo::Control &inits_controls,
                           IN  const Robo::Control &lower_controls,
                           IN  const Robo::Control &upper_controls,
                           OUT       Robo::Control &controls);
    //------------------------------------------------------------------------------
    void  rundownControls(IN OUT Robo::Control &controls);
    //------------------------------------------------------------------------------
    bool  rundownNextControl(IN OUT Robo::Control  &controls,
                             IN OUT size_t         &controls_curr,
                             IN OUT Robo::frames_t &velosity,
                             IN OUT Robo::frames_t &velosity_prev);
    //------------------------------------------------------------------------------
    bool  rundownNextControl(IN OUT Robo::Control    &controls,
                             IN OUT std::vector<int> &lasts_changes,
                             IN OUT Robo::frames_t   &velosity);
    //------------------------------------------------------------------------------
    typedef std::function<bool(const RoboMoves::Record &, const Point &)> func_t;
    typedef std::set<std::size_t> visited_t;
    //------------------------------------------------------------------------------
    const RoboMoves::Record*
        gradientClothestRecord(IN const RoboMoves::adjacency_ptrs_t &range,
                               IN const Point   &aim,
                               IN const func_t  *pPred = NULL,
                               IN OUT visited_t *pVisited = NULL);
    //------------------------------------------------------------------------------
    bool
        gradientClothestRecords(IN  const      Point  &aim,
                                OUT RoboMoves::Record *pRecClose,
                                OUT RoboMoves::Record *pRecLower,
                                OUT RoboMoves::Record *pRecUpper,
                                IN OUT visited_t      *pVisited = NULL);
    //------------------------------------------------------------------------------


    //==============================================
    /* Mixtures */
    size_t  weightedMean(IN const Point &aim, OUT Point &hand_position, IN bool verbose = false);
    //------------------------------------------------------------------------------
    size_t  rundownMDir(IN const Point &aim, OUT Point &hand_position, IN bool verbose = false);
    size_t  rundownFull(IN const Point &aim, OUT Point &hand_position, IN bool verbose = false);
    //------------------------------------------------------------------------------

public:
    LearnMoves(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target) :
        store(store), robo(robo), target(target), precision(target.precision())
    {
        robo.reset();
        base_pos = robo.position();
    }
    LearnMoves(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target,
               IN double precision,
               IN const ConfigJSON::StageInput1 &stage1,
               IN const ConfigJSON::StageInput2 &stage2,
               IN const ConfigJSON::StageInput3 &stage3) :
        store(store), robo(robo), target(target),
        precision(precision),
        stage1(stage1), stage2(stage2), stage3(stage3)
    {
        robo.reset();
        base_pos = robo.position();
    }
    //------------------------------------------------------------------------------
    /* грубое покрытие всего рабочего пространства */
    void  STAGE_1(IN bool verbose = true);
    /* Покрытие всей мишени не слишком плотно */
    void  STAGE_2(IN bool verbose = true);
    /* Попадание в оставшиеся непокрытыми точки мишени */
    void  STAGE_3(OUT Robo::Trajectory &uncovered, OUT size_t &complexity, IN bool verbose = true);
    //------------------------------------------------------------------------------
    void  uncover(OUT Robo::Trajectory &uncovered);
    //------------------------------------------------------------------------------
    size_t  gradientMethod(IN const Point &aim, IN bool verbose = false);
    size_t  gradientMethod_admixture(IN const Point &aim, IN bool verbose = false);

    //------------------------------------------------------------------------------
    /* грубое покрытие всего рабочего пространства */
    void  testStage1();
    /* Покрытие всей мишени не слишком плотно */
    void  testStage2();
    /* Попадание в оставшиеся непокрытыми точки мишени */
    void  testStage3(OUT std::list<Point> &uncovered);
    //------------------------------------------------------------------------------
    void  Mean(IN const Point &aim,
               OUT Robo::Trajectory   *trajectory = NULL,
               OUT Robo::Trajectories *trajectories = NULL);

    void  Close(IN const Point &aim,
                OUT Robo::Trajectory   *trajectory = NULL,
                OUT Robo::Trajectories *trajectories = NULL);
    //------------------------------------------------------------------------------
    void  testCoverTarget(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target);
    //------------------------------------------------------------------------------
    void  getTargetCenter(IN RoboMoves::Store &store, IN Robo::RoboI &robo, OUT Point &center);
    //------------------------------------------------------------------------------
    bool  tryToHitTheAim(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN const Point &aim);
    //------------------------------------------------------------------------------
    void  rundownMethod_old(IN const Point &aim,
                            IN const Robo::Control &controls,
                            IN Point &hand_position);
    //------------------------------------------------------------------------------
}; // end LearnMovements

}