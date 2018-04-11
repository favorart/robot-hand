#include "StdAfx.h"

#ifndef  _TARGET_H_
#define  _TARGET_H_

#include "WindowHeader.h"
//------------------------------------------------------------------------------
class RecTarget
{
public:
  typedef std::vector<Point> vec_t;

private:
  // --- main content --------------------
  vec_t  coords_;

  // --- rectangle range -----------------
  size_t  c_rows, c_cols; // число строк и столбцов (по числу мишений)
  // -------------------------------------
  Point  center_, min_, max_;
  Point thickness_; // (ширина строки, ширина столбца)
  // -------------------------------------  
  void  generate ();
  
public:
  // -------------------------------------
  RecTarget (size_t r, size_t c,
             const Point &min,
             const Point &max):
    c_rows (r), c_cols (c), 
    center_ ( (min.x + max.x) / 2., (min.y + max.y) / 2. ), 
    min_ (min), max_ (max), coords_ (c_rows * c_cols)
  { generate (); }

  RecTarget (size_t r, size_t c,
             double lft, double rgh,
             double top, double btm):
    c_rows (r), c_cols (c),
    center_ ((lft + rgh) / 2., (btm + top) / 2. ), 
    min_ (lft, btm), max_ (rgh, top),
    coords_ (r * c)
  { generate (); }
  // -------------------------------------
  size_t  coordsCount () const { return coords_.size (); }
  const vec_t& coords () const { return coords_; }

  void  draw (HDC hdc, HPEN hPen,
              bool internalLines,
              bool internalPoints,
              bool ellipseInsteadPixel) const;

  bool  contain (const Point &p) const
  { return (p.x >= min_.x && p.x <= max_.x && p.y >= min_.y && p.y <= max_.y); }

  const Point&   RecTarget::min    () const { return     min_; }
  const Point&   RecTarget::max    () const { return     max_; }
  const Point&   RecTarget::center () const { return  center_; }

  double     precision () const
  { return /* 0.004 */ (std::max (max_.x - min_.x, max_.y - min_.y) / 200.); }
  double     thickness () const
  { return   std::max (thickness_.x, thickness_.y); }
  // ---------------------------------
};
//------------------------------------------------------------------------------
#endif // _TARGET_H_
