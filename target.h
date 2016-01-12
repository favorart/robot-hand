#include "StdAfx.h"

#ifndef  _TARGET_H_
#define  _TARGET_H_

#include "MyWindow.h"
//------------------------------------------------------------------------------
class RecTarget
{
public:
  // ---------------------------------
  typedef std::set<Point> set_t;

private:
  // ---------------------------------
  set_t  coords_; /* main content */

  // --- rectangle range -------------
  double  lft, rgh, top, btm;
  size_t  c_rows, c_cols;
  // ---------------------------------

public:
  // ---------------------------------
  RecTarget () : 
    c_rows (0), c_cols (0), lft (0), rgh (0), top (0), btm (0) {}
  RecTarget (size_t r, size_t c, const Point &min, const Point &max):
    c_rows (r), c_cols (c), lft(min.x), rgh (max.x), top (max.y), btm(min.y) {}
  RecTarget (size_t r, size_t c, double lft, double rgh, double top, double btm):
    c_rows (r), c_cols (c), lft (lft), rgh (rgh), top (top), btm (btm)
  {
  }
  // ---------------------------------
  uint_t  coordsCount () const { return coords_.size (); }
  void  draw (HDC hdc, HPEN hPen) const;
  // ---------------------------------
};
//------------------------------------------------------------------------------
#endif // _TARGET_H_
