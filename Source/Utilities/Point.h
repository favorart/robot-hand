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
  bool  operator<  (const Point &p) const;
  bool  operator>  (const Point &p) const;
  bool  operator<= (const Point &p) const;
  bool  operator>= (const Point &p) const;

  bool  operator!= (const Point &p) const; 
  bool  operator== (const Point &p) const;

  Point operator- () const
  { return Point{ -x, -y }; }
  Point operator+ () const
  { return Point{ x, y }; }
  //-----------------------------------------
  const Point&  operator= (const Point &p);
  //-----------------------------------------
  void  rotate_degrees(const Point &center, double angle)
  { rotate_radians (center, angle * M_PI / 180.); }
  void  rotate_radians(const Point &center, double angle);

  void  rotate (const Point &center, double angle)
  { rotate_radians (center, angle * M_PI / 180.);}
  bool     hit (const Point &p, double eps = EPS) const;
  double angle (const Point &p) const;
  double norm2 () const { return sqrt(x * x + y * y); }
  //-----------------------------------------
  /* Microsoft specific: C++ properties */
  __declspec(property(get = get_x, put = put_x)) double x;
  double get_x () const { return x_; }
  void   put_x (double x) { x_ = x; }

  __declspec(property(get = get_y, put = put_y)) double y;
  double get_y () const { return y_; }
  void   put_y (double y) { y_ = y; }
  //-----------------------------------------
  /* type-casts */
  operator std::pair<double, double> () const
  { return std::make_pair(x, y); }
  operator boost::tuple<double, double> () const
  { return boost::make_tuple (x, y); }
  operator boost::geometry::model::d2::point_xy<double> () const
  { return boost::geometry::model::d2::point_xy<double> (x, y); }

  operator tstring () const
  {
    tstringstream ss;
    ss << _T ("pt<x=") << x << _T (", y=") << y << _T (">");
    return ss.str ();
  }
  //-----------------------------------------
  friend tistream&  operator>> (tistream &in, Point &p);
  friend tostream&  operator<< (tostream &out, const Point &p);
  //-----------------------------------------
  Point operator- (const Point &p) const { return Point{ x - p.x, y - p.y }; }
  Point operator+ (const Point &p) const { return Point{ x + p.x, y + p.y }; }
  Point operator* (const Point &p) const { return Point{ x * p.x, y * p.y }; }
  Point operator/ (const Point &p) const { return Point{ x / p.x, y / p.y }; }

  Point& operator+= (const Point &p) { x += p.x; y += p.y; return *this; }
  Point& operator-= (const Point &p) { x -= p.x; y -= p.y; return *this; }
  Point& operator*= (const Point &p) { x *= p.x; y *= p.y; return *this; }
  Point& operator/= (const Point &p) { x /= p.x; y /= p.y; return *this; }

  Point& operator+= (double d) { x += d; y += d; return *this; }
  Point& operator-= (double d) { x -= d; y -= d; return *this; }
  Point& operator*= (double d) { x *= d; y *= d; return *this; }
  Point& operator/= (double d) { x /= d; y /= d; return *this; }
  //-----------------------------------------
private:
  /* serialization */
  friend class boost::serialization::access;
  BOOST_SERIALIZATION_SPLIT_MEMBER ()
  
  template <class Archive>
  void  save (Archive &ar, unsigned version) const { ar & x_ & y_; }  
  template <class Archive>
  void  load (Archive &ar, unsigned version) { ar & x_ & y_; }
};
//-------------------------------------------------------------------------------
inline Point rotate (const Point &p, const Point &center, double angle)
{
    Point pt{ p };
    pt.rotate(center, angle);
    return pt;
}

// Angle = L ABC
inline double angle_radians (const Point &A, const Point &B, const Point &C)
{
    return atan2(boost_distance(A, C), boost_distance(B, C));
}
// Angle = L ABC
inline double angle_degrees (const Point &A, const Point &B, const Point &C)
{
    return atan2(boost_distance(A, C), boost_distance(B, C)) * 180. / M_PI;
}

inline double norm2 (const Point &p)
{ return p.norm2(); }
//-------------------------------------------------------------------------------
// The intersection point of two lines
Point intersectionLines   (const Point &L1_s, const Point &L1_f,
                           const Point &L2_s, const Point &L2_f);
Point alongLineAtDistance (const Point &from, const Point &to, double distance);
//-------------------------------------------------------------------------------
inline Point operator+ (const Point &p, double d) { return Point{ p.x + d, p.y + d }; }
inline Point operator- (const Point &p, double d) { return Point{ p.x - d, p.y - d }; }
inline Point operator* (const Point &p, double d) { return Point{ p.x * d, p.y * d }; }
inline Point operator/ (const Point &p, double d) { return Point{ p.x / d, p.y / d }; }
//-------------------------------------------------------------------------------
struct PointHasher
{
  std::size_t operator()(const Point& k) const
  {
    std::size_t seed = 0;
    // modify seed by xor and bit-shifting of the key members
    boost::hash_combine (seed, boost::hash_value (k.x));
    boost::hash_combine (seed, boost::hash_value (k.y));
    return seed;
  }
};
//-------------------------------------------------------------------------------
BOOST_CLASS_VERSION (Point, 1)
#endif // _POINT_H_
