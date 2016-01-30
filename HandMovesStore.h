#include "StdAfx.h"

#ifndef  _HAND_MOVES_H_
#define  _HAND_MOVES_H_

#pragma once

//#include "Hand.h"
//using namespace OldHand;
#include "NewHand.h"
using namespace NewHand;

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
    const static size_t  maxMovesCount = 2U;

    typedef std::array<Hand::time_t, arrays_size> times_array;
    typedef std::array<Hand::MusclesEnum, arrays_size> muscles_array;

    struct MovePart
    {
      Hand::MusclesEnum  muscle;

      times_t  time;
      times_t  last;

      MovePart () : muscle (Hand::EmptyMov), time (0), last (0) {}

      MovePart (Hand::MusclesEnum  muscle, times_t time, times_t last) :
        muscle (muscle), time (time), last (last) {}

      template<class Archive>
      void serialize (Archive & ar, const unsigned int version)
      { ar & muscle & time & last; }
    };

  private:
    // typedef std::/*unordered_*/ map < Hand::MusclesEnum,
    //                              std::pair<time_t, time_t> 
    //                            > muscle_times_t;

    // ----------------------------------------
    size_t             moves_count_;

    Hand::MusclesEnum  muscles_;
    trajectory_t       visited_;

    // muscle_times_t     times_;
    std::list<MovePart>   moves_;
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
      { ar << aim_ << hand_ << muscles_ << moves_count_;
        // ar << times_start_  << times_stop_; <<
        // ar << times_ << visited_;
        ar << moves_ << visited_;
        ar << distance_ << elegance_;
      }
    
    template <class Archive>
    void  load (Archive & ar, const unsigned int version)
      { ar >> aim_;
        ar >> hand_;
        ar >> muscles_;
        ar >> moves_count_;
        // ar >> times_;
        ar >> moves_;
        ar >> visited_;
        //ar >> times_start_;
        //ar >> times_stop_;
        ar >> distance_;
        ar >> elegance_;
      }

    // template<class Archive>
    // void serialize (Archive & ar, const unsigned int version)
    // { ar & moves_count_ & /* controls_ & */ start_times_ & distance_ & elegance_ & hand_ & aim_; }
    // ----------------------------------------

    double  Elegance ();

  public:
    // ----------------------------------------
    struct ByP {};
    struct ByX {};
    struct ByY {};

    struct ChangePoint : public std::unary_function<Record, void>
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
            size_t               moves_count,
            const trajectory_t  &visited);
    
    // ----------------------------------------
    operator  tstring () const
    { return  str (boost::wformat (_T("rec<x=%1%, y=%2%>")) % aim.x % aim.y); }

    // ----------------------------------------
    /* Microsoft specific: C++ properties */
    __declspec(property(get = get_aim)) const Point &aim;
    const Point&  get_aim () const { return aim_; }

    __declspec(property(get = get_traj)) const trajectory_t &trajectory;
    const trajectory_t&  get_traj () const { return visited_; }
    // ----------------------------------------
    bool  validateMusclesTimes ();

    void makeHandMove (MyWindowData &wd);
  };

  using namespace boost::multi_index;
  //------------------------------------------------------------------
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
  /* сериализация */
  void  storeSave (const Store& store, const TCHAR *filename=_T("moves.bin"));
  void  storeLoad (      Store& store, const TCHAR *filename=_T("moves.bin"));
  //------------------------------------------------------------------------------
  /* тестовые движения рукой */
  void  test_random (Store &store, Hand &hand, size_t tries);
  void  test_cover  (Store &store, Hand &hand,
                     // std::list< std::list<Point> > &trajectories,
                     size_t nesting /* 1, 2, 3 */);
}
BOOST_CLASS_VERSION (HandMoves::Record, 1)

#endif // _HAND_MOVES_H_
