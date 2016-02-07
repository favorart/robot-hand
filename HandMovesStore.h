#include "StdAfx.h"

#ifndef  _HAND_MOVES_H_
#define  _HAND_MOVES_H_
//------------------------------------------------------------------------------
#define HAND_VER 2
#if   HAND_VER == 1
#include "Hand.h"
using namespace OldHand;
#include "HandMuscles.h"
#elif HAND_VER == 2
#include "NewHand.h"
using namespace NewHand;
#include "HandMuscles.h"
#endif

//------------------------------------------------------------------------------
#include "MyWindow.h"

namespace HandMoves
{
  // !!! tree types of point !!! ????
  // Point  aim_;
  // Point  hand_;
  // Point  close_network_;
  
  // !!!!
  // compare (x1 < x2 & y1 < y2)
  
  typedef std::list<Point> trajectory_t;

  class Record
  {
    typedef uint32_t times_t;

    const static size_t  arrays_size = 4U;
  public:
    const static size_t  maxControlsCount = 2U;

    typedef std::array<Hand::time_t, arrays_size> times_array;
    typedef std::array<Hand::MusclesEnum, arrays_size> muscles_array;

    struct HandControl
    {
      Hand::MusclesEnum  muscle;

      times_t  time;
      times_t  last;

      HandControl () : muscle (Hand::EmptyMov), time (0), last (0) {}

      HandControl (Hand::MusclesEnum  muscle, times_t time, times_t last) :
        muscle (muscle), time (time), last (last) {}

      template<class Archive>
      void  serialize (Archive & ar, const unsigned int version)
      { ar & muscle & time & last; }
    };

  private:
    // typedef std::/*unordered_*/ map < Hand::MusclesEnum,
    //                              std::pair<time_t, time_t> 
    //                            > muscle_times_t;

    // ----------------------------------------
    size_t             controls_count_;

    Hand::MusclesEnum  muscles_;
    trajectory_t       visited_;

    // muscle_times_t     times_;
    std::list<HandControl>   hand_controls_;
    // times_array   times_start_;
    // times_array   times_stop_ ;

    Point     aim_;
    Point     hand_;
    
    // ----- calc -----------------------------
    double    distance_;
    double    elegance_;

    // ----------------------------------------
    friend class boost::serialization::access;
    BOOST_SERIALIZATION_SPLIT_MEMBER ()
    
    template <class Archive>
    void  save (Archive & ar, const unsigned int version) const
      { ar << aim_ << hand_ << muscles_ << controls_count_;
        // ar << times_start_  << times_stop_ << times_;
        ar << hand_controls_ << visited_ << distance_ << elegance_;
      }
    template <class Archive>
    void  load (Archive & ar, const unsigned int version)
      { ar >> aim_ >> hand_ >> muscles_ >> controls_count_;
        // ar >> times_start_ >> times_stop_ >> times_;
        ar >> hand_controls_ >> visited_ >> distance_ >> elegance_;
      }

    // template<class Archive>
    // void serialize (Archive & ar, const unsigned int version)
    // { ar & moves_count_ & /* controls_ & */ start_times_
    //      & distance_ & elegance_ & hand_ & aim_; }
    // ----------------------------------------

  public:
    // ----------------------------------------
    struct ByP {};
    struct ByX {};
    struct ByY {};

    struct ChangePoint : public std::unary_function<Record,void>
    {
      ChangePoint (const Point &p) : p_ (p) {}
      void operator() (Record &rec) { rec.aim_ = p_; }
    private:
      Point p_;
    };

    double aim_x () const { return aim_.x; }
    double aim_y () const { return aim_.y; }
    // ----------------------------------------
    Record () {}

    Record (const Point         &aim,
            const Point         &hand,
            const muscles_array &muscles,
            const times_array   &times,
            const times_array   &lasts,
            size_t               controls_count,
            const trajectory_t  &visited);

    Record (const Point         &aim,
            const Point         &hand,
            const std::initializer_list<HandControl> controls,
            const trajectory_t  &visited);
    
    // ----------------------------------------
    operator  tstring () const
    { return  str (boost::wformat (_T("rec<x=%1%, y=%2%>")) % aim.x % aim.y); }

    // ----------------------------------------
    /* Microsoft specific: C++ properties */
    __declspec(property(get = get_aim)) const Point &aim;
    const Point&  get_aim () const { return aim_; }
    __declspec(property(get = get_trajectory)) const trajectory_t &trajectory;
    const trajectory_t&  get_trajectory () const { return visited_; }
    __declspec(property(get = get_controls_count)) size_t  controlsCount;
    size_t  get_controls_count () const { return controls_count_; }

    size_t   controls (OUT std::array<HandControl, Record::maxControlsCount> &controls) const
    {
      size_t i = 0U;
      for ( auto hc : hand_controls_ )
      { controls[i++] = hc; }
      return controls_count_;
    }
    std::list<HandControl> const &  controls () const
    { return hand_controls_; }
    // ----------------------------------------
    bool    validateMusclesTimes () const;
    void    repeatMove   (Hand &hand) const;
    double  eleganceMove (const Point &aim) const;
    // ----------------------------------------
  };

  using namespace boost::multi_index;
  //------------------------------------------------------------------------------
  typedef boost::multi_index_container
  < Record,
    indexed_by <
                  ordered_unique    < tag<Record::ByP>,
                                      composite_key < Record,
                                                      const_mem_fun<Record, double, &Record::aim_x>,
                                                      const_mem_fun<Record, double, &Record::aim_y>
                                                    >
                                     >,
                  ordered_non_unique < tag<Record::ByX>, const_mem_fun<Record, double, &Record::aim_x> >,
                  ordered_non_unique < tag<Record::ByY>, const_mem_fun<Record, double, &Record::aim_y> > //,
                  // random_access      <> // доступ, как у вектору
                >
  > Store;
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
    { return this->operator() (boost_point2_t (a->aim),
                               boost_point2_t (b->aim));
    }
    double  operator() (const Record &a,
                        const Record &b)
    { return this->operator() (boost_point2_t (a.aim),
                               boost_point2_t (b.aim));
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
  /* сериализация */
  void  storeSave (const Store& store, tstring filename);
  void  storeLoad (      Store& store, tstring filename);
  //------------------------------------------------------------------------------
  /* тестовые движения рукой */
  void  test_random (Store &store, Hand &hand, size_t tries);
  void  test_cover  (Store &store, Hand &hand, size_t nesting /* = 1,2,3 */);
}
BOOST_CLASS_VERSION (HandMoves::Record, 1)

#endif // _HAND_MOVES_H_
