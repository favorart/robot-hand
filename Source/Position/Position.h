#include "StdAfx.h"

#ifndef  _POSITION_H_
#define  _POSITION_H_

#include "WindowHeader.h"
#include "target.h"
#include "Store.h"

#include "LinearOperator.h"
#include "DirectionPredictor.h"

namespace Positions
{
  // #define _HAND_TEST_CONSOLE_PRINTF
  // #define _DEBUG_PRINT
#define _DEBUG_PRINT_RES
  //------------------------------------------------------------------------------
  void  testCover (IN HandMoves::Store &store, IN Hand &hand,
                   IN HandMoves::trajectories_t &trajectories);
  //------------------------------------------------------------------------------
  typedef  std::map
  < Hand::MusclesEnum /* -------------muscle---*/,
    std::pair< Hand::frames_t /*---min lasts---*/,
               Hand::frames_t /*---max lasts---*/ >
  >  borders_t;
  //------------------------------------------------------------------------------
  void  insertRecordToBorders (borders_t &borders, const HandMoves::Record &rec);
  void  defineBorders (borders_t &borders, Hand &hand, Hand::frames_t lasts_init);
  void  defineBorders (borders_t &borders, RecTarget &target, HandMoves::Store &store, double distance);
  //------------------------------------------------------------------------------
  struct counts_t
  {
    int count = 0;
    int count_TP = 0;
    int count_FP = 0;
    int count_TN = 0;
    int count_FN = 0;

    void  incr (bool model, bool real)
    {
      ++count;
      if ( model & real )
        ++count_TP;
      else if ( !model & !real )
        ++count_TN;
      else if ( model & !real )
        ++count_FP;
      else if ( !model & real )
        ++count_FN;
    }
    void  fill (Hand &hand, RecTarget &target,
                HandMoves::controling_t &controls,
                const Point &end)
    {
      //-------------------------------------------------------------
      HandMoves::trajectory_t  trajectory;
      hand.SET_DEFAULT;
      hand.move (controls.begin (), controls.end (), &trajectory);
      //-------------------------------------------------------------
      bool model = target.contain (end);
      bool real = target.contain (hand.position);
      incr (model, real);
    }
    void  print ()
    {
      tcout << _T ("count = ") << count << std::endl;
      tcout << _T ("\t< T >\t < F >") << std::endl;
      tcout << _T ("< P >\t") << count_TP << _T ("\t") << count_FP << std::endl;
      tcout << _T ("< N >\t") << count_TN << _T ("\t") << count_FN << std::endl;
    }
    void  clear ()
    {
      count = 0;
      count_TP = 0;
      count_FP = 0;
      count_TN = 0;
      count_FN = 0;
    }
  };
  //------------------------------------------------------------------------------
  class LearnMovements
  {
    /*  Количество точек, в окресности искомой точки.
     *
     *  Что я могу варьировать?
     *
     *  v  1. Длительность работы каждого мускула
     *  v  2. Время старта каждого мускула
     *  v  3. Тормозить мускулом? (прерывать '1' входа на которкое время)
     *
     *  ?  4. Может быть !1 перелом! траектории (полная остановка)
     *  Он может нам понадобиться в зависимости от законов движения
     *  разных мускулов, если один до второго не успевает ...
     *  (?позже включить??)
     *
     */

     /* stage_1 */
     //==============================================
     /* Желаемое расстояние между конеными точками
      * чернового покрытия
      */
    double             draft_distance = 0.07;
    /* Время преодоления инерции */
    Hand::frames_t   lasts_init_value1 = 50U;
    /* Изменение шага времени */
    Hand::frames_t   lasts_incr_value1 = 10U;
     //==============================================

     /* stage_2 */
     //==============================================
    double            detail_distance  = 0.005;
    /* Время преодоления инерции */
    Hand::frames_t   lasts_init_value2 = 1U;
    /* Изменение шага времени */
    Hand::frames_t   lasts_incr_value2 = 3U;
     //==============================================

    /* stage_3 */
    //==============================================
    /* Точность = 1.5 мм */
    const double  precision = 0.0035;
    /* половина ребра квадрата, из которого берутся все точки */
    double side = 0.1;
    /* шаг уменьшения области поиска для взвешанной суммы */
    double side_decrease_step = 0.001;

    Hand::frames_t lasts_step = 1U;
    // -----------------------------------------------
    std::set<size_t>  visited;

    //==============================================

    /* hit the aim */
    //==============================================
    size_t hit_tries;
    double hit_precision;
    //==============================================

    //==============================================
    /* Mean */
    /* Взвешенная арифметическое среднее (иначе обычное) */
    bool weighted_mean = true;

    //==============================================
    HandMoves::Store &store;

    Hand &hand;
    Point hand_pos_base;

    RecTarget &target;

    size_t complexity = 0U;
    //==============================================
    bool  hand_act (IN  const Point &aim,
                    IN  const HandMoves::controling_t  &controls,
                    OUT Point &hand_position, IN bool copy=true);
    //------------------------------------------------------------------------------
    void  w_meansControls (IN  const Point &aim,
                           IN  const HandMoves::adjacency_refs_t &range,
                           OUT HandMoves::controling_t &controls,
                           OUT double *weight=NULL);

    bool  w_meansULAdjs (IN  const Point  &aim, OUT HandMoves::Record *pRec,
                         OUT HandMoves::controling_t &lower_controls,
                         OUT HandMoves::controling_t &upper_controls,
                         OUT double       &lower_distance,
                         OUT double       &upper_distance);
    //------------------------------------------------------------------------------
    void  gradientControls (IN  const Point &aim, IN  double  d_d,
                            IN  const HandMoves::controling_t &inits_controls,
                            IN  const HandMoves::controling_t &lower_controls,
                            IN  const HandMoves::controling_t &upper_controls,
                            OUT       HandMoves::controling_t &controls);
    //------------------------------------------------------------------------------
    void  rundownControls  (IN OUT HandMoves::controling_t &controls);
    //-----------------------------------------------------------------------------
    void  rundownControlsOrder (IN const HandMoves::controling_t &init_controls,
                                OUT      HandMoves::controling_t      &controls);
    //------------------------------------------------------------------------------
    bool  rundownNextControl (IN OUT HandMoves::controling_t &controls,
                              IN OUT                  size_t &controls_curr,
                              IN OUT          Hand::frames_t &velosity,
                              IN OUT          Hand::frames_t &velosity_prev);
    //------------------------------------------------------------------------------
    int deep = 0U;

    //==============================================
    /* Mixtures */
    size_t  w_means (IN const Point &aim, OUT Point &hand_position, IN bool verbose=false);
    size_t  rundown (IN const Point &aim, OUT Point &hand_position, IN bool verbose=false);

  public:
    // LearnMovements () : lasts_incr_value1 (10U), lasts_incr_value2 (3U) {}
    // LearnMovements (HandMoves::Store &store, Hand &hand, RecTarget &target)
    // {
    //   lasts_incr_value1 = 0U;
    //   for ( auto m : hand.muscles_ )
    //     lasts_incr_value1 += hand.maxMuscleLast (m);
    //   lasts_incr_value1 /= (Hand::frames_t) hand.muscles_.size ();
    //   lasts_incr_value1 /= count_points;
    // }

    LearnMovements (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target) :
      store (store), hand (hand), target (target)
    {
      hand.SET_DEFAULT;
      hand_pos_base = hand.position;
    }
    //------------------------------------------------------------------------------
    /* грубое покрытие всего рабочего пространства */
    void  STAGE_1 (IN  bool verbose=true);
    /* Покрытие всей мишени не слишком плотно */
    void  STAGE_2 (IN  bool verbose=true);
    /* Попадание в оставшиеся непокрытыми точки мишени */
    void  STAGE_3 (OUT HandMoves::trajectory_t &uncovered, OUT size_t &complexity,
                   IN  bool verbose=true);
    //------------------------------------------------------------------------------
    void  LearnMovements::uncover (OUT HandMoves::trajectory_t &uncovered);
    //------------------------------------------------------------------------------
    size_t   rundownMethod           (IN const Point &aim, IN bool verbose=false);
    size_t  gradientMethod           (IN const Point &aim, IN bool verbose=false);
    size_t  gradientMethod_admixture (IN const Point &aim, IN bool verbose=false);
    //------------------------------------------------------------------------------
    /* грубое покрытие всего рабочего пространства */
    void  testStage1 (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target);
    /* Покрытие всей мишени не слишком плотно */
    void  testStage2 (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target);
    /* Попадание в оставшиеся непокрытыми точки мишени */
    void  testStage3 (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target,
                      OUT std::list<Point> &uncovered);
    //------------------------------------------------------------------------------
    void  Mean  (IN const Point &aim,
                 OUT HandMoves::trajectory_t   *trajectory=NULL,
                 OUT HandMoves::trajectories_t *trajectories=NULL);

    void  Close (IN const Point &aim,
                 OUT HandMoves::trajectory_t   *trajectory=NULL,
                 OUT HandMoves::trajectories_t *trajectories=NULL);
    //------------------------------------------------------------------------------
    void  testCoverTarget (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target);
    //------------------------------------------------------------------------------
    void  getTargetCenter (IN HandMoves::Store &store, IN Hand &hand, OUT Point &center);
    //------------------------------------------------------------------------------
    bool  tryToHitTheAim  (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim);
    //------------------------------------------------------------------------------
    void  rundownMethod_old (IN const Point &aim,
                             IN const HandMoves::controling_t &controls,
                             IN Point &hand_position);
    //------------------------------------------------------------------------------
  }; // end LearnMovements
};
#endif // _POSITION_H_
