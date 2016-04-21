#include "StdAfx.h"

#ifndef  _POSITION_H_
#define  _POSITION_H_
//------------------------------------------------------------------------------
#define HAND_VER 2
#if   HAND_VER == 1
#include "Hand.h"
using namespace OldHand;
#elif HAND_VER == 2
#include "Hand.h"
using namespace NewHand;
#include "HandMuscles.h"
#endif

//------------------------------------------------------------------------------
#include "WindowHeader.h"
#include "target.h"

#include "Store.h"

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
  class LinearOperator
  {
    Point  min_, max_;
    std::vector<double>  xCoefs, yCoefs;

    static const int     n_muscles;
    static const double normilizer;

    const int  LSO = 0;
    const int  LSC = 1;
    const int  LEO = 2;
    const int  LEC = 3;

    void createJointControl (IN HandMoves::controling_t &controls,
                             IN int *solution, IN size_t opn, IN size_t cls,
                             IN Hand::MusclesEnum Opn, IN Hand::MusclesEnum Cls);

  public:
    LinearOperator () {}
    LinearOperator (IN  HandMoves::Store &store,
                    IN  const Point &aim,
                    IN  double radius,
                    OUT HandMoves::controling_t &controls,
                    IN  bool verbose=false) throw (...);

    void  predict (IN  const Point &aim,
                   OUT HandMoves::controling_t &controls,
                   IN  bool verbose=false);
  };
  //------------------------------------------------------------------------------

  // class PredictedDirection
  // {
  //   Hand::MusclesEnum  muscle;
  //   Hand::frames_t     last;
  //   // trajectory_t trajectory; ???
  //   // Point center;
  // 
  //   PredictedDirection ():
  //     muscle (Hand::EmptyMov), last (0U), end (0,0) {}
  //   PredictedDirection (Hand::MusclesEnum m, Hand::frames_t l, Point e) :
  //     muscle (m), last (l), end (e) {}
  // 
  // public:
  //   Point end;
  // };
  class  DirectionPredictor
  {
    //------------------------------------------
    struct  MainDirection
    {
      //------------------------------------------
      Hand::MusclesEnum   muscle;

      std::vector<Point>  shifts;
      Point               center;

      double            min_radius;
      double            max_radius;
      //------------------------------------------
      MainDirection () {}
      MainDirection (Hand::MusclesEnum m, Hand &hand) :
        muscle (m),
        center (hand.jointPosition (jointByMuscle (m)))
      {
        std::list<Point>  trajectory;
        //-----Must be !!! EACH !!!--------
        auto oldEach = hand.visitedSaveEach;
        hand.visitedSaveEach = 1U;
        hand.move (m, hand.maxMuscleLast (m), trajectory /*, !!! */);
        hand.visitedSaveEach = oldEach;
        //---------------------------------
        Point &front = trajectory.front ();
        max_radius = boost_distance (center, front);
        min_radius = boost_distance (center, front);
        //---------------------------------
        for ( auto pt : trajectory )
        {
          double radius = boost_distance (center, pt);
          //---------------------------------
          if ( radius > max_radius ) max_radius = radius;
          if ( radius < min_radius ) min_radius = radius;
          //---------------------------------
          shifts.push_back (Point (pt.x - front.x,
                                   pt.y - front.y));
        }
      }
      //------------------------------------------
      bool  operator<  (const MainDirection &md) const
      { return  (muscle  < md.muscle); }
      bool  operator== (const MainDirection &md) const
      { return  (muscle == md.muscle); }
      bool  operator!= (const MainDirection &md) const
      { return  (muscle != md.muscle); }

      bool  operator== (Hand::MusclesEnum m) const
      { return  (muscle == m); }
      bool  operator!= (Hand::MusclesEnum m) const
      { return  (muscle != m); }
    };
    //------------------------------------------
    std::map<Hand::MusclesEnum, MainDirection>  main_directions;
    Point hand_base;
    //------------------------------------------
  public:
    DirectionPredictor (IN Hand &hand);
    //------------------------------------------
    void  predict (IN Hand::Control control, OUT Point &end);
    void  predict (IN  Hand::MusclesEnum m,
                   IN  Hand::frames_t    l,
                   OUT Point &end)

    { predict (Hand::Control (m, 0U, l), end); }
    void  measure (IN Hand &hand, IN  const Point &aim,
                   OUT HandMoves::controling_t &controls);
    //------------------------------------------
  private:
    bool  shifting_gt (Hand::MusclesEnum m, unsigned int &inx, const Point &aim);
    bool  shifting_ls (Hand::MusclesEnum m, unsigned int &inx, const Point &aim);
  };
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
    /* Взвешенная арифметическое среднее (иначе обычное) */
    bool weighted_mean = true;
    /* половина ребра квадрата, из которого берутся все точки */
    double side = 0.005;
    /* шаг уменьшения области поиска для взвешанной суммы */
    double side_decrease_step = 0.001;

    Hand::frames_t lasts_step = 1U;

    size_t complexity = 0U;
    //==============================================

    /* hit the aim */
    //==============================================
    size_t hit_tries;
    double hit_precision;
    //==============================================

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

    //------------------------------------------------------------------------------
    /* грубое покрытие всего рабочего пространства */
    void  STAGE_1 (IN  HandMoves::Store &store, IN Hand &hand, IN RecTarget &target);
    /* Покрытие всей мишени не слишком плотно */
    void  STAGE_2 (IN  HandMoves::Store &store, IN Hand &hand, IN RecTarget &target);
    /* Попадание в оставшиеся непокрытыми точки мишени */
    void  STAGE_3 (IN  HandMoves::Store &store, IN Hand &hand, IN RecTarget &target);
    //------------------------------------------------------------------------------
    void  Mean  (IN  HandMoves::Store &store, IN Hand &hand, IN const Point &aim, IN double side,
                 OUT HandMoves::trajectory_t   *trajectory=NULL,
                 OUT HandMoves::trajectories_t *trajectories=NULL);

    void  Close (IN  HandMoves::Store &store, IN Hand &hand, IN const Point &aim, IN double side,
                 OUT HandMoves::trajectory_t   *trajectory=NULL,
                 OUT HandMoves::trajectories_t *trajectories=NULL);
    //------------------------------------------------------------------------------
    void  rundownMethod     (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim,
                             IN const HandMoves::controling_t &init_controls, IN Point &hand_position);
    void  rundownMethod_old (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim,
                             IN const HandMoves::controling_t &init_controls, IN Point &hand_position);
    void  gradientMethod    (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim); //,
                             // IN const HandMoves::controling_t &init_controls, IN Point &hand_position);
    //------------------------------------------------------------------------------
    /* грубое покрытие всего рабочего пространства */
    void  testStage1 (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target);
    /* Покрытие всей мишени не слишком плотно */
    void  testStage2 (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target);
    /* Попадание в оставшиеся непокрытыми точки мишени */
    void  testStage3 (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target,
                      OUT std::list<Point> &uncovered);
    //------------------------------------------------------------------------------
    void  testCoverTarget (IN HandMoves::Store &store, IN Hand &hand, IN RecTarget &target);
    //------------------------------------------------------------------------------
    void  getTargetCenter (IN HandMoves::Store &store, IN Hand &hand, OUT Point &center);
    //------------------------------------------------------------------------------
    bool  tryToHitTheAim  (IN HandMoves::Store &store, IN Hand &hand, IN const Point &aim);
    //------------------------------------------------------------------------------
  }; // end LearnMovements
};
#endif // _POSITION_H_
