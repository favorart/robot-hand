#include "StdAfx.h"

#ifndef  _POINT_H_
#define  _POINT_H_
//--------------------------------------------------------------------------------
class Point
{
  double x_, y_;
  //-----------------------------------------
public:
  Point () : x_ (0.), y_ (0.) {}
  Point (double x, double y) : x_ (x), y_ (y) {}
  Point (const Point &p) : x_ (p.x), y_ (p.y) {}
  //-----------------------------------------
  // compare operators
  bool  operator<  (const Point &p) const
  { return (x_ == p.x_) ? (y_ < p.y_) : (x_ < p.x_); }
  bool  operator!= (const Point &p) const { return !hit (p); }
  bool  operator== (const Point &p) const { return  hit (p); }

  //-----------------------------------------
  const Point&  operator= (const Point &p)
  { if ( this != &p )
    { x = p.x; y = p.y; }
    return *this;
  }

  operator std::wstring () const
  { return  boost::str (boost::wformat (_T("pt<x=%1%, y=%2%>")) % x % y); }
  //-----------------------------------------
  /* internal functions */
  void  rotate (const Point &center, double angle);
  bool  hit    (const Point &p, double eps=EPS) const
  { return ((abs (x_ - p.x_) < eps)
         && (abs (y_ - p.y_) < eps));
  }
  //-----------------------------------------
  /* Microsoft specific: C++ properties */
  __declspec(property(get = get_x, put = put_x)) double x;
  double get_x () const { return x_; }
  void   put_x (double x) { x_ = x; }

  __declspec(property(get = get_y, put = put_y)) double y;
  double get_y () const { return y_; }
  void   put_y (double y) { y_ = y; }
  //-----------------------------------------
  // type casts to boost
  operator boost::tuple<double, double> () const
  { return boost::make_tuple (x, y); }
  
  operator boost::geometry::model::d2::point_xy<double> () const
  { return boost::geometry::model::d2::point_xy<double> (x, y); }
  //-----------------------------------------
private:

  friend class boost::serialization::access;
  BOOST_SERIALIZATION_SPLIT_MEMBER ()
  
  // serialization
  template <class Archive>
  void  save (Archive & ar, const unsigned int version) const { ar & x_ & y_; }
  
  template <class Archive>
  void  load (Archive & ar, const unsigned int version) { ar & x_ & y_; }
  //-----------------------------------------
  // printing
  friend std::istream&  operator>> (std::istream &in, Point &p);
  friend std::ostream&  operator<< (std::ostream &out, const Point &p);
};
//-------------------------------------------------------------------------------
//BOOST_CLASS_VERSION (Point, 1)

#endif // _POINT_H_
