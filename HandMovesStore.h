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
      
      uint_t   times_start_[maxMovesCount];
      uint_t   times_stop_ [maxMovesCount];
      // bitset_t  controls_;

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
        ar << times_start_  << times_stop_ << distance_ << elegance_;
      }
      
      template <class Archive>
      void  load (Archive & ar, const unsigned int version)
      { ar >> aim_;
        ar >> hand_;
        ar >> muscles_;
        ar >> moves_count_;
        ar >> times_start_;
        ar >> times_stop_;
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
            Hand::MusclesEnum   muscles,
            const uint_t       *times_start,
            const uint_t       *times_stop);

    Record (const Record &rec):
      Record (rec.aim_, rec.hand_, rec.muscles_, 
              rec.times_start_, rec.times_stop_)
    {}

    const Record& operator=(const Record& rec)
    {
      if ( &rec != this )
      {
        // this->~Record ();
        // this = new ();
      }
      return *this;
    }

    
    operator  std::string () const
    { return  str (boost::format ("rec<x=%1%, y=%2%>") % aim_.x % aim_.y); }
  
    /* Microsoft specific: C++ properties */
    __declspec(property(get = get_aim)) Point aim;
    const Point& get_aim () const { return aim_; }

    void  make_move (Hand &hand)
    {
      for ( size_t i = 0; i < moves_count_; ++i )
      {
      
      }
    }

    

  };
  //BOOST_CLASS_VERSION (HandMoveRecord, 1)
  
  
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
                ordered_non_unique < tag<Record::ByY>, const_mem_fun<Record, double, &Record::aim_y> >,
                random_access      <> // доступ, как у вектору
              >
> Store;
  
  //------------------------------------------------------------------------------
  // bool hit (const Point* ptg, const Point* p) const
  // { return (!ptg->hit (*p, 1e-3) && *ptg < *p); }
  

  // void  store_init (const Store &store, std::list<Point> target);

  // const Point&  store_search (const Store &store, const Point  &aim);
  
  // template <Archive>
  // void  store_save (Archive & ar, const Store &store);
  // template <Archive>
  // void  store_load (Archive & ar, Store       &store);
  //------------------------------------------------------------------------------

  Hand::MusclesEnum  switch_move (uint_t choose);
  
  void  test_random (Store &store, Hand &hand, uint_t  tries=1000U);
  void  test_cover  (Store &store, Hand &hand, double radius, 
                     std::list< std::list<Point> > &trajectories,
                     int number);
  
  
  // 
  // void createRectTarget ()
  // {
  // 
  // 
  // }
}
#endif // _HAND_MOVES_H_
