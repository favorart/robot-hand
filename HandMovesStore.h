#include "StdAfx.h"

#ifndef  _HAND_MOVES_H_
#define  _HAND_MOVES_H_
//------------------------------------------------------------------------------
#define HAND_VER 2
#if   HAND_VER == 1
#include "Hand.h"
using namespace OldHand;
#elif HAND_VER == 2
#include "NewHand.h"
using namespace NewHand;
#include "HandMuscles.h"
#endif
//------------------------------------------------------------------------------
#include "MyWindow.h"

namespace HandMoves
{
  // !!! tree types of point !!!
  //-------------------------------
  // Point  aim_;
  // Point  hand_;
  // Point  close_network_;
  
  typedef std::list<Point>          trajectory_t;
  typedef std::list<trajectory_t>   trajectories_t;
  typedef std::list<Hand::Control>  controling_t;

  class Record
  {
  public:
    const static size_t  maxControlsCount = 2U;

    const static size_t  arrays_size = 4U;
    typedef std::array<Hand::frames_t, arrays_size> times_array;
    typedef std::array<Hand::MusclesEnum, arrays_size> muscles_array;

    controling_t  hand_controls_;
  private:
    // ----------------------------------------
    Point  aim_, hand_begin_, hand_final_;
    // ----------------------------------------
    Hand::MusclesEnum   muscles_;
    trajectory_t        visited_;

    /* ПЕРЕЛОМы !?!?! */

    // ----------------------------------------
    friend class boost::serialization::access;
    BOOST_SERIALIZATION_SPLIT_MEMBER ()

    template <class Archive>
    void  save (Archive & ar, const unsigned int version) const
    {
      ar << aim_ << hand_begin_ << hand_final_;
      ar << muscles_ << hand_controls_ << visited_;
    }
    template <class Archive>
    void  load (Archive & ar, const unsigned int version)
    {
      ar >> aim_ >> hand_begin_ >> hand_final_;
      ar >> muscles_ >> hand_controls_ >> visited_;
    }

  public:
    // ----------------------------------------
    struct ByP {};
    struct ByX {};
    struct ByY {};
    struct ByD {};
    struct ByC {};

    struct ChangePoint : public std::unary_function<Record, void>
    {
      ChangePoint (const Point &p) : p_ (p) {}
      void operator() (Record rec) { rec.aim_ = p_; }
    private:
      Point p_;
    };

    double hit_x () const { return hand_final_.x; }
    double hit_y () const { return hand_final_.y; }

    Hand::MusclesEnum  muscles () const
    { return muscles_; }
    // ----------------------------------------
    Record () {}

    Record (const Point         &aim,
            const Point         &hand_begin,
            const Point         &hand_final,
            const muscles_array &muscles,
            const times_array   &times,
            const times_array   &lasts,
            size_t               controls_count,
            const trajectory_t  &visited);

    Record (const Point         &aim,
            const Point         &hand_begin,
            const Point         &hand_final,
            const std::list<Hand::Control> controls,
            const trajectory_t  &visited);

    // ----------------------------------------
    operator tstring () const
    { return  str (boost::wformat (_T ("rec<x=%1%, y=%2%>")) % hit.x % hit.y); }

    // ----------------------------------------
    /* Microsoft specific: C++ properties */
    __declspec(property(get = get_aim)) const Point &aim;
    const Point&  get_aim () const { return aim_; }

    __declspec(property(get = get_final)) const Point &hit;
    const Point&  get_final () const { return hand_final_; }

    __declspec(property(get = get_trajectory)) const trajectory_t &trajectory;
    const trajectory_t  &get_trajectory () const { return visited_; }

    __declspec(property(get = get_controls_count)) size_t  controlsCount;
    size_t  get_controls_count () const { return hand_controls_.size (); }

    size_t   controls_array (OUT std::array<Hand::Control, Record::maxControlsCount> &controls) const
    {
      size_t i = 0U;
      for ( auto &hc : hand_controls_ )
      { controls[i++] = hc; }
      return controlsCount;
    }
    const controling_t&  controls () const { return hand_controls_; }
    // ----------------------------------------
    bool    validateMusclesTimes () const;
    void    repeatMove (Hand &hand) const
    {
      hand.SET_DEFAULT;
      trajectory_t visited;
      hand.move (hand_controls_.begin (), hand_controls_.end (), &visited);

      auto it1 = visited .begin ();
      auto it2 = visited_.begin ();
      auto index = 0U;
      for ( ; it1 != visited.end () && it2 != visited_.end (); ++it1, ++it2 )
      {
        if ( *it1 != *it2 )
        {
          auto  &j1 = *it1;
          auto  &j2 = *it2;
        }
        ++index;
      }
    }

    double  eleganceMove (/* const Point &aim */) const;
    double  distanceCovered () const
    { return boost_distance (hand_final_, hand_begin_); }
    // ----------------------------------------
    double  Record::ratioDistanceByTrajectory () const;
    double  Record::ratioTrajectoryDivirgence () const;

    double  Record::ratioUsedMusclesCount () const;
    double  Record::ratioTrajectoryBrakes () const;
    // ----------------------------------------
  };

  struct ControlingHasher : std::unary_function<const controling_t&, size_t>
  {
    std::size_t operator()(const controling_t &controls) const
    {
      std::size_t seed = 0U;
      for ( Hand::Control c : controls )
      {
        boost::hash_combine (seed, boost::hash_value (c.muscle));
        boost::hash_combine (seed, boost::hash_value (c.start));
        boost::hash_combine (seed, boost::hash_value (c.last));
      }
      return seed;
    }
  };


  using namespace boost::multi_index;
  //------------------------------------------------------------------------------
  typedef boost::multi_index_container
  < Record,
    indexed_by < hashed_unique      < tag<Record::ByC>,
                                      member<Record, controling_t, &Record::hand_controls_>,
                                      ControlingHasher
                                    >,
                 ordered_non_unique < tag<Record::ByP>,
                                      composite_key < Record,
                                                      const_mem_fun<Record, double, &Record::hit_x>,
                                                      const_mem_fun<Record, double, &Record::hit_y>
                                                    >
                                    >,
                 ordered_non_unique < tag<Record::ByX>, const_mem_fun<Record, double, &Record::hit_x> >,
                 ordered_non_unique < tag<Record::ByY>, const_mem_fun<Record, double, &Record::hit_y> >,
                 ordered_non_unique < tag<Record::ByD>, const_mem_fun<Record, double, &Record::distanceCovered> > // ,
              // random_access      < > // доступ, как у вектору
               >
  > Store;
  //------------------------------------------------------------------------------
  void  storeInsert (Store &store, const Record &rec);
  //------------------------------------------------------------------------------
  /* прямоугольная окрестность точки */
  size_t  adjacencyRectPoints (Store &store, std::list<Record> &range,
                               const Point &left_down, const Point &right_up);
  size_t  adjacencyRectPoints (Store &store, std::list<std::shared_ptr<Record>> &range,
                               const Point &left_down, const Point &right_up);

  /* круглая окрестность точки */
  size_t  adjacencyPoints (Store &store, std::list<Record> &range,
                           const Point &center, double radius);

  size_t  adjacencyPoints (Store &store, std::list<std::shared_ptr<Record>> &range,
                           const Point &center, double radius);

  size_t  similDistances (Store &store, std::list<std::shared_ptr<HandMoves::Record>> &range,
                          double distance, double over=EPS);

  size_t  similControls (Store &store, std::list<std::shared_ptr<HandMoves::Record>> &range,
                         Hand::MusclesEnum control, bool contains_not_exact=true);
  
  /* Все точки с данным x | y */
  void  adjacencyYsByXPoints (Store &store, std::list<Record> &range,
                              double x, double up = 0., double down = 0.);
  void  adjacencyXsByYPoints (Store &store, std::list<Record> &range,
                              double y, double left=0., double right=0.);
  //------------------------------------------------------------------------------
  class ClosestPredicate
  {
    boost_point2_t aim;
  public:
    ClosestPredicate (const Point &aim) : aim (aim) {}
    double  operator() (const std::shared_ptr<Record> &a,
                        const std::shared_ptr<Record> &b)
    { return this->operator() (boost_point2_t (a->hit),
                               boost_point2_t (b->hit));
    }
    double  operator() (const Record &a,
                        const Record &b)
    { return this->operator() (boost_point2_t (a.hit),
                               boost_point2_t (b.hit));
    }
    double  operator() (const boost_point2_t &a,
                        const boost_point2_t &b)
    {
      double  da = boost::geometry::distance (aim, a);
      double  db = boost::geometry::distance (aim, b);
      return (da < db);
    }
  };
  //------------------------------------------------------------------------------
  class ElegancePredicate
  {
  public:
    double  operator() (const Record &a,
                        const Record &b)
    {
       /*  Элегантность - это
        *  отношение дистанции к длине траектории
        *  и отсутствие переломов 
        */

        // (1. / controlsCount) * /* Количество движений != ПЕРЕЛОМ */

      /* Длина траектории по сравнениею с дистанцией */
      auto  ra = a.ratioDistanceByTrajectory ();
      auto  rb = b.ratioDistanceByTrajectory ();
      return (ra < rb);
    }
    double  operator() (const std::shared_ptr<Record> &a,
                        const std::shared_ptr<Record> &b)
    { return this->operator() (*a, *b); }
  };
  //------------------------------------------------------------------------------
  class RecordHasher
  {
    std::size_t operator()(const Record& rec) const
    {
      std::size_t  seed = 0U;
      // modify seed by xor and bit-shifting
      // of the key members
      boost::hash_combine (seed, boost::hash_value (rec.hit.x));
      boost::hash_combine (seed, boost::hash_value (rec.hit.y));
      // the result.
      return seed;
    }
  
  };
  //------------------------------------------------------------------------------
  /* сериализация */
  void  storeSave (const Store& store, tstring filename);
  void  storeLoad (      Store& store, tstring filename);
  //------------------------------------------------------------------------------
  /* тестовые движения рукой */
  void  test_random (Store &store, Hand &hand, size_t tries);
  void  test_cover  (Store &store, Hand &hand, size_t nesting /* = 1,2,3 */);
  //------------------------------------------------------------------------------
  // void  testCover (Store &store, Hand &hand, Hand::MusclesEnum muscles, \
                     std::list<std::shared_ptr<std::list<Point>>> &trajectories);

  // void  getTargetCenter (Hand &hand, Point &center);
  // void  testCoverTarget (Store &store, Hand &hand, RecTarget &target);
}
BOOST_CLASS_VERSION (HandMoves::Record, 1)

#include "Position.h"

#endif // _HAND_MOVES_H_
