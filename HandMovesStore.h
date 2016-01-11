#include "StdAfx.h"

#ifndef  _HAND_MOVES_H_
#define  _HAND_MOVES_H_

#pragma once

#include "Hand.h"
#include "MyWindow.h"

namespace HandMoves
{
  // !!! tree types of point !!! ????
  // Point  aim_;
  // Point  hand_;
  // Point  close_network_;
  
  // !!!!
  // compare (x1 < x2 & y1 < y2)
  
  class Record
  {
    public:
      const static uint_t  maxMovesCount = 4U;
      // typedef std::bitset< maxMovesCount * Hand::musclesCount >  bitset_t;

    private:
      // ----------------------------------------
      uint_t    moves_count_;
      Hand::MusclesEnum  muscles_;
      // bitset_t  controls_;
      
      std::map<Hand::MusclesEnum, std::pair<uint_t, uint_t>> times_;

      // uint_t   times_start_[maxMovesCount];
      // uint_t   times_stop_ [maxMovesCount];
      // struct move_t
      // {
      //   Hand::MusclesEnum  muscle_;
      //   uint_t start_;
      //   uint_t stop_;
      // 
      //   for (auto m : Hand::muscles)
      //     if ( muscle & m )
      //       index
      // };
      // move_t times_;

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
        ar << times_ << distance_ << elegance_;
      }
      
      template <class Archive>
      void  load (Archive & ar, const unsigned int version)
      { ar >> aim_;
        ar >> hand_;
        ar >> muscles_;
        ar >> moves_count_;
        ar >> times_;
        //ar >> times_start_;
        //ar >> times_stop_;
        ar >> distance_;
        ar >> elegance_;
      }

      // template<class Archive>
      // void serialize (Archive & ar, const unsigned int version)
      // { ar & moves_count_ & /* controls_ & */ start_times_ & distance_ & elegance_ & hand_ & aim_; }
      // ----------------------------------------

      double  Elegance ()
      { return 0.; }

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

    Record (const Point        &aim,
            const Point        &hand,
            Hand::MusclesEnum  *muscles,
            const uint_t       *times_start,
            const uint_t       *times_stop,
            uint_t              moves_count);

    // Record (const Point        &aim,
    //         const Point        &hand,
    //         Hand::MusclesEnum   muscles,
    //         const uint_t       *times_start,
    //         const uint_t       *times_stop);

    //Record (const Record &rec):
    //  Record (rec.aim_, rec.hand_, rec.muscles_, 
    //          //rec.times_start_, rec.times_stop_)
    //{}

    const Record& operator=(const Record& rec)
    {
      if ( &rec != this )
      {
        this->~Record ();
        new (this) Record (rec);
      }
      return *this;
    }

    
    operator  std::string () const
    { return  str (boost::format ("rec<x=%1%, y=%2%>") % aim_.x % aim_.y); }
  
    /* Microsoft specific: C++ properties */
    __declspec(property(get = get_aim)) Point aim;
    const Point& get_aim () const { return aim_; }

    bool  validateMusclesTimes ();

    void  makeMove (Hand &hand, const Point &aim)
    {
      for ( size_t i = 0; i < moves_count_; ++i )
      {
      
      }
    }

  };

using namespace boost::multi_index;
//------------------------------------------------------------------
typedef boost::multi_index_container
< Record,
  indexed_by <
                ordered_unique    < tag<Record::ByP>,
                                    composite_key < Record,
                                                    const_mem_fun<Record, double, &Record::aim_x>, //.get_x
                                                    const_mem_fun<Record, double, &Record::aim_y>  //.get_y
                                                  >
                                   >,
                ordered_non_unique < tag<Record::ByX>, const_mem_fun<Record, double, &Record::aim_x> >,
                ordered_non_unique < tag<Record::ByY>, const_mem_fun<Record, double, &Record::aim_y> > //,
                // random_access      <> // доступ, как у вектору
              >
> Store;
  //------------------------------------------------------------------------------
  // прямоугольная окрестность точки
  template<typename T>
  uint_t  adjacencyRectPoints (Store &store,
                               std::back_insert_iterator<T> &range_it,
                               const boost::tuple<double, double> &left_down,
                               const boost::tuple<double, double> &right_up,
                               bool  pointer_type = false);

  // круглая окрестность точки
  template<typename T>
  uint_t  adjacencyPoints (Store &store,
                           std::back_insert_iterator<T> &range_it,
                           const boost_point2_t &center,
                           double radius,
                           bool   pointer_type = false);

  void  adjacencyYsByXPoints (Store &store, std::list<Record> &range,
                              double x, double up=0., double down=0.);

  void  adjacencyXsByYPoints (Store &store, std::list<Record> &range,
                              double y, double left=0., double right=0.);
  //------------------------------------------------------------------------------
  void  storeSave (const Store& store, const TCHAR *filename=_T("moves.bin"));
  void  storeLoad (      Store& store, const TCHAR *filename=_T("moves.bin"));
  //------------------------------------------------------------------------------
  void  test_random (Store &store, Hand &hand, uint_t  tries=1000U);
  void  test_cover  (Store &store, Hand &hand, 
                     std::list< std::list<Point> > &trajectories,
                     int nesting=2);
}
BOOST_CLASS_VERSION (HandMoves::Record, 1)

#endif // _HAND_MOVES_H_
