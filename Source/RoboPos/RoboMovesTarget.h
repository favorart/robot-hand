#include "StdAfx.h"
#include <vector>
#include "WindowHeader.h"

#ifndef  _TARGET_H_
#define  _TARGET_H_
//------------------------------------------------------------------------------
class TargetI
{
public:
    typedef std::vector<Point> vec_t; // TODO : Any Container !

    virtual std::pair<vec_t::const_iterator, vec_t::const_iterator> it_coords() const
    { throw std::logic_error{ "Not implemented" }; };
    virtual const vec_t &coords() const
    { throw std::logic_error{ "Not implemented" }; };

    virtual size_t n_coords() const = 0;
    virtual bool contain(const Point &p) const = 0;
    virtual const Point& min() const = 0;
    virtual const Point& max() const = 0;
    virtual const Point& center() const = 0;
    virtual double precision() const = 0;
    virtual double thickness() const = 0;
    virtual void draw(HDC hdc, HPEN hPen,
                      bool internalLines,
                      bool internalPoints,
                      double internalPointsRadius) const = 0;
};
//------------------------------------------------------------------------------
class RecTarget : public TargetI
{
protected:
  // --- main content --------------------
  vec_t  coords_;

  // --- rectangle range -----------------
  size_t  c_rows, c_cols; ///< число строк и столбцов (по числу мишений)
  // -------------------------------------
  Point center_, min_, max_;
  Point thickness_; ///< max(ширина строки, ширина столбца)
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
  size_t  n_coords() const { return coords_.size (); }
  const vec_t &coords() const { return coords_; }
  std::pair<vec_t::const_iterator, vec_t::const_iterator> it_coords() const
  { return std::make_pair(coords_.begin(), coords_.end()); }

  void  draw (HDC hdc, HPEN hPen,
              bool internalLines,
              bool internalPoints,
              double internalPointsRadius = 0.007) const;

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
