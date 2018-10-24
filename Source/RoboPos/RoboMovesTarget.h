#include "StdAfx.h"
#include <vector>
#include "WindowHeader.h"

#ifndef  _TARGET_H_
#define  _TARGET_H_
//------------------------------------------------------------------------------
class TargetI
{
public:
    static double draw_radius;
    using vec_t = std::vector<Point>;

    size_t n_coords() const { return _coords.size(); }
    const vec_t &coords() const { return _coords; }
    std::pair<vec_t::const_iterator, vec_t::const_iterator> it_coords() const
    { return std::make_pair(_coords.begin(), _coords.end()); }

    virtual const Point& min() const { return _min; }
    virtual const Point& max() const { return _max; }
    virtual const Point& center() const { return _center; }
    virtual double thickness() const { return _thickness; }
    virtual double precision() const { return _precision; }
    //{ return /* 0.004 */ (std::max(max().x - min().x, max().y - min().y) / n_coords()/*200.*/); }
    void set_precision(double d) { _precision = d; }

    virtual bool contain(const Point &p) const
    { return (p.x >= min().x && p.x <= max().x && p.y >= min().y && p.y <= max().y); }
    virtual void draw(HDC hdc, HPEN hPen,
                      bool internalLines,
                      bool internalPoints,
                      double internalPointsRadius) const = 0;

    virtual void save(tptree&) const;
    virtual void load(tptree&);
    static tstring name() { return _T("target"); }

protected:
    TargetI(tptree&) {}
    TargetI(const Point &min, const Point &max) :
        _center((min.x + max.x) / 2., (min.y + max.y) / 2.), _min(min), _max(max)
    {}
    
    vec_t _coords{}; ///< main content
    Point _center{}, _min{}, _max{};
    double _thickness{}; ///< max(ширина строки, ширина столбца)
    double _precision{ 0.004 };

    virtual void generate() = 0;
    void set_thickness(double d) { _thickness = d; }
};
//------------------------------------------------------------------------------
class RecTarget : public TargetI
{
protected:
  unsigned _n_rows, _n_cols; ///< rectangle range: число строк и столбцов (по числу мишений)
  void generate();
  
public:
  static std::shared_ptr<TargetI> make(const tstring &type, tptree &root)
  { return (type == RecTarget::name()) ? std::make_shared<RecTarget>(root) : std::shared_ptr<TargetI>(nullptr); }
  static tstring name() { return _T("RecT"); }
  RecTarget(tptree &root) :
      TargetI(root)
  {
      load(root);
      generate();
  }

  RecTarget (size_t n_rows, size_t n_cols, const Point &min, const Point &max):
    TargetI(min, max), _n_rows(n_rows), _n_cols(n_cols)
  { 
      generate();
  }
  RecTarget (size_t n_rows, size_t n_cols, double lft, double rgh, double top, double btm):
      RecTarget(n_rows, n_cols, { lft, btm }, { rgh, top })
  {}
  void  draw (HDC hdc, HPEN hPen,
              bool internalLines,
              bool internalPoints,
              double internalPointsRadius = TargetI::draw_radius) const;

  void save(tptree&) const;
  void load(tptree&);
};
//------------------------------------------------------------------------------
class PolyTarget : public TargetI
{
protected:
    vec_t _polygon;           ///< outer polygon vertices
    unsigned _n_rows, _n_cols;  ///< rectangle range: число строк и столбцов (по числу мишений)
    void generate();

public:
    static std::shared_ptr<TargetI> make(const tstring &type, tptree &root)
    { return (type == PolyTarget::name()) ? std::make_shared<PolyTarget>(root) : std::shared_ptr<TargetI>(nullptr); }
    static tstring name() { return _T("PolyT"); }
    PolyTarget(tptree &root) :
        TargetI(root)
    {
        load(root);
        generate();
    }

    PolyTarget(const vec_t &polygon, size_t n_rows, size_t n_cols, const Point &min, const Point &max) :
        TargetI(min, max), _n_rows(n_rows), _n_cols(n_cols), _polygon(polygon)
    {
        generate();
    }
    static void calc_min_max(const vec_t &polygon, Point &min, Point &max)
    {
        min = { 1.,1. };
        max = { -1., -1. };
        br::for_each(polygon, [&min, &max](const Point &p) {
            if (min.x > p.x) min.x = p.x;
            if (min.y > p.y) min.y = p.y;
            if (max.x < p.x) max.x = p.x;
            if (max.y < p.y) max.y = p.y;
        });
    }
    bool contain(const Point &p) const;
    void  draw(HDC hdc, HPEN hPen,
               bool internalLines,
               bool internalPoints,
               double internalPointsRadius = TargetI::draw_radius) const;

    void save(tptree&) const;
    void load(tptree&);
};
//------------------------------------------------------------------------------
#endif // _TARGET_H_
