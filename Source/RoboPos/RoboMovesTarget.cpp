#include "StdAfx.h"
#include "WindowDraw.h"
#include "RoboMovesTarget.h"


double TargetI::draw_radius = 0.009;

void TargetI::save(tptree &root) const
{
    tptree node;
    root.add_child(_T("target"), node);
    node.put(_T("center"), _center);
    node.put(_T("min"), _min);
    node.put(_T("max"), _max);
    node.put(_T("_thickness"), _thickness);
    node.put(_T("_precision"), _precision);
}
void TargetI::load(tptree &root)
{
    auto &node = root.get_child_optional(_T("target")).get_value_or(root);
    _center.load(node.get_child(_T("center")));
    _min.load(node.get_child(_T("min")));
    _max.load(node.get_child(_T("max")));
    _thickness = node.get<double>(_T("thickness"));
    _precision = node.get<double>(_T("precision"));
}
//------------------------------------------------------------------------------
void  RecTarget::generate ()
{
    _coords.resize(_n_rows * _n_cols);
  double r_step = (_max.y - _min.y) / (_n_rows - 1U);
  double c_step = (_max.x - _min.x) / (_n_cols - 1U);

  int k = 0;
  for (auto i = 0u; i < _n_rows; ++i)
    for (auto j = 0u; j < _n_cols; ++j)
      _coords[k++] = (Point (/* left   */ _min.x + j * c_step,
                             /* bottom */ _min.y + i * r_step));

  _min.x -= Utils::EPSILONT; _min.y -= Utils::EPSILONT;
  _max.x += Utils::EPSILONT; _max.y += Utils::EPSILONT;
  set_thickness(std::max(r_step, c_step));
}

void  RecTarget::draw(HDC hdc, HPEN hPen,
                      bool internalLines,
                      bool internalPoints,
                      double internalPointsRadius) const
{
    double r = (max().y - min().y) / (_n_rows - 1U);
    double c = (max().x - min().x) / (_n_cols - 1U);

    drawMyFigure(hdc,
                 { (max().x + min().x) / 2, (max().y + min().y) / 2 },
                   (max().x - min().x) + c, (max().y - min().y) + r, 0.,
                 MyFigure::Rectangle, hPen);
    //-----------------------------------------------------------------
    if (internalLines)
    {
        for (auto i = 1U; i < _n_cols; ++i)
            drawLine(hdc, { /* lft */ min().x + i * c - c / 2., /* top */ min().y - r / 2. },
                          { /* lft */ min().x + i * c - c / 2., /* btm */ max().y + r / 2. }, hPen);

        for (auto i = 1U; i < _n_rows; ++i)
            drawLine(hdc, { /* lft */ min().x - c / 2., /* btm */ min().y + i * r - r / 2. },
                          { /* rgh */ max().x + c / 2., /* btm */ min().y + i * r - r / 2. }, hPen);
    }
    //-----------------------------------------------------------------
    if (internalPoints)
    {
        for (auto p : _coords)
            drawCircle(hdc, p, internalPointsRadius, hPen);
    }
}

void RecTarget::save(tptree &root) const
{
    TargetI::save(root);
    auto &node = root.get_child(_T("target"));
    node.put(_T("type"), RecTarget::name());
    node.put(_T("hn_aims"), _n_cols);
    node.put(_T("vn_aims"), _n_rows);
    //node.put(_T("left"), target.left);
    //node.put(_T("right"), target.right);
    //node.put(_T("top"), target.top);
    //node.put(_T("bottom"), target.bottom);
}
void RecTarget::load(tptree &root)
{
    TargetI::load(root);
    auto &node = root.get_child_optional(_T("target")).get_value_or(root);
    //auto type = node.get<tstring>(_T("type"));
    //if (type != RecTarget::name())
    //    CERROR("RecTarget: Invalid load");
    _n_cols = node.get<unsigned>(_T("hn_aims"));
    _n_rows = node.get<unsigned>(_T("vn_aims"));
    //target.left = node.get<double>(_T("left"));
    //target.right = node.get<double>(_T("right"));
    //target.top = node.get<double>(_T("top"));
    //target.bottom = node.get<double>(_T("bottom"));
}
//------------------------------------------------------------------------------
// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(Point p, Point q, Point r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
        return true;
    return false;
}

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(const Point &p, const Point &q, const Point &r)
{
    auto val = (q.y - p.y) * (r.x - q.x) -
               (q.x - p.x) * (r.y - q.y); // ??? norm2
    if (val == 0) return 0;  // colinear 
    return (val > 0) ? 1 : 2; // clock or counterclock wise 
}

// The function that returns true if line segment 'p1q1' and 'p2q2' intersect. 
bool doIntersect(const Point &p1, const Point &q1, const Point &p2, const Point &q2)
{
    // Find the four orientations needed for general and special cases 
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    // General case 
    if (o1 != o2 && o3 != o4)
        return true;
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;
    // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;
    return false; // Doesn't fall in any of the above cases 
}

// Returns true if the point p lies inside the polygon[] with n vertices
#include <memory> // for std::allocator
#include <limits> // std::numeric_limits<T>
template <template <typename, typename> class Container,
    typename Value = Point,
    typename Allocator = std::allocator<Value> >
bool isInsidePolygon(const Container<Value, Allocator> &polygon, const Point &p)
{
    // There must be at least 3 vertices in polygon[] 
    if (polygon.size() < 3)
        return false;

    // Create a point for line segment from p to infinite 
    Point extreme = { std::numeric_limits<double>::infinity(), p.y };

    // Count intersections of the above line with sides of polygon 
    int count = 0;
    for (auto it = polygon.begin(); it != polygon.end(); ++it)
    {
        auto next = std::next(it);
        if (next == polygon.end())
            next = polygon.begin();

        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(*it, *next, p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(*it, p, *next) == 0)
                return onSegment(*it, p, *next);
            count++;
        }
    }
    // Return true if count is odd, false otherwise 
    return (count & 1);  // Same as (count%2 == 1) 
}
//------------------------------------------------------------------------------

void PolyTarget::generate()
{
    _coords.reserve(_n_rows * _n_cols);
    double r_step = (_max.y - _min.y) / (_n_rows - 1U);
    double c_step = (_max.x - _min.x) / (_n_cols - 1U);

    for (auto i = 0U; i < _n_rows; ++i)
        for (auto j = 0U; j < _n_cols; ++j)
        {
            Point p{ _min.x + j * c_step,
                     _min.y + i * r_step };
            if (isInsidePolygon(_polygon, p))
            {
                CDEBUG(p);
                _coords.push_back(p);
            }
        }

    _min.x -= Utils::EPSILONT; _min.y -= Utils::EPSILONT;
    _max.x += Utils::EPSILONT; _max.y += Utils::EPSILONT;
    set_thickness(std::max(r_step, c_step));
}

bool PolyTarget::contain(const Point &p) const
{ return isInsidePolygon(_polygon, p); }

void PolyTarget::draw(HDC hdc, HPEN hPen,
                      bool internalLines,
                      bool internalPoints,
                      double internalPointsRadius) const
{
    double r = (max().y - min().y) / (_n_rows - 1U);
    double c = (max().x - min().x) / (_n_cols - 1U);

    drawMyFigure(hdc, _polygon, MyFigure::Polygon, hPen);
    //-----------------------------------------------------------------
    if (internalLines)
    {
        for (auto i = 1U; i < _n_cols; ++i)
            drawLine(hdc, { /* lft */ min().x + i * c - c / 2., /* top */ min().y - r / 2. },
                          { /* lft */ min().x + i * c - c / 2., /* btm */ max().y + r / 2. }, hPen);

        for (auto i = 1U; i < _n_rows; ++i)
            drawLine(hdc, { /* lft */ min().x - c / 2., /* btm */ min().y + i * r - r / 2. },
                          { /* rgh */ max().x + c / 2., /* btm */ min().y + i * r - r / 2. }, hPen);
    }
    //-----------------------------------------------------------------
    if (internalPoints)
    {
        for (auto p : _coords)
            drawCircle(hdc, p, internalPointsRadius, hPen);
    }
}

void PolyTarget::save(tptree &root) const
{
    TargetI::save(root);
    auto &node = root.get_child(_T("target"));
    node.put(_T("type"), RecTarget::name());
    node.put(_T("hn_aims"), _n_cols);
    node.put(_T("vn_aims"), _n_rows);
    tptree polygon;
    node.add_child(_T("polygon"), polygon);
    for (auto &p : _polygon)
    {
        tptree pt;
        p.save(pt);
        //tptree pp;
        //pp.put_value(pt);
        polygon.push_back(std::make_pair(_T(""), pt));
    }
}
void PolyTarget::load(tptree &root)
{
    TargetI::load(root);
    auto &node = root.get_child_optional(_T("target")).get_value_or(root);
    //auto type = node.get<tstring>(_T("type"));
    //if (type != RecTarget::name())
    //    CERROR("RecTarget: Invalid load");
    _n_cols = node.get<unsigned>(_T("hn_aims"));
    _n_rows = node.get<unsigned>(_T("vn_aims"));
    auto &polygon = node.get_child(_T("polygon"));
    for (tptree::value_type &v : node.get_child(_T("polygon")))
    {
        Point p;
        p.load(v.second);
        _polygon.push_back(p);
    }
}
//------------------------------------------------------------------------------
