#include "Point.h"

//--------------------------------------------------------------------------------
const Point&  Point::operator= (const Point &p)
{ if ( this != &p )
  { x = p.x; y = p.y; }
  return *this;
}
//--------------------------------------------------------------------------------
/* compare operators */
bool  Point::operator<  (const Point &p) const
{ return (x_ < p.x_) && (y_ < p.y_); }
bool  Point::operator>  (const Point &p) const
{ return !(*this < p); }
bool  Point::operator<= (const Point &p) const
{ return (*this < p) || hit(p); }
bool  Point::operator>= (const Point &p) const
{ return !(*this < p) || hit(p); }

bool  Point::operator== (const Point &p) const
{ return  hit (p); }
bool  Point::operator!= (const Point &p) const
{ return !hit (p); }
//--------------------------------------------------------------------------------
bool  Point::hit(const Point &p, Point::value_type eps) const
{ return ((abs (x_ - p.x_) < eps) && (abs (y_ - p.y_) < eps)); }
bool  Point::hit(const Point &p) const
{ return hit(p, Utils::EPSILONT); }

//--------------------------------------------------------------------------------
void Point::rotate_radians(const Point &center, Point::value_type angle)
{
    Point::value_type tmp;
    x_ -= center.x_;
    y_ -= center.y_;

    tmp = (x_ * cos(angle)) - (y_ * sin(angle));
    y_  = (x_ * sin(angle)) + (y_ * cos(angle));
    x_  = tmp;

    x_ += center.x_;
    y_ += center.y_;
}

Point::value_type Point::angle(const Point &p) const
{ return atan2(p.y - y, p.x - x) * 180. / M_PI; }
//--------------------------------------------------------------------------------
/// To determine which side of the line from-to a point pt falls on
LineSide onLineSide(const Point &from, const Point &to, const Point &pt)
{
    Point::value_type d = (pt.x - from.x)*(to.y - from.y) - (pt.y - from.y)*(to.x - from.x);
    return (d == 0) ? LineSide::OnLine : ((d < 0) ? LineSide::Left : LineSide::Right);
    // To see whether points on the left side of the line are those with positive or negative values 
    // compute the value for d for a point you know is to the left of the line, such as(x1−1, y1)
    // and then compare the sign with the point you are interested in.
}

/// Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr' 
bool onSegment(const Point &p, const Point &q, const Point &r)
{
    return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
}

/// To find orientation of ordered triplet (p, q, r)
Orientation orientation(const Point &p, const Point &q, const Point &r)
{
    auto val = (q.y - p.y) * (r.x - q.x) -
               (q.x - p.x) * (r.y - q.y); // ??? norm2
    if (val == 0) return Orientation::Colinear;
    return (val > 0) ? Orientation::ClockWise : Orientation::CounterClockWise;
}

/// Returns true if the line segments 'p1q1' and 'p2q2' intersect
bool isIntersectLines(const Point &p1, const Point &q1, const Point &p2, const Point &q2)
{
    // Find the four orientations needed for general and special cases 
    auto o1 = orientation(p1, q1, p2);
    auto o2 = orientation(p1, q1, q2);
    auto o3 = orientation(p2, q2, p1);
    auto o4 = orientation(p2, q2, q1);
    // General case 
    if (o1 != o2 && o3 != o4)
        return true;
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == Orientation::Colinear && onSegment(p1, p2, q1))
        return true;
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == Orientation::Colinear && onSegment(p1, q2, q1))
        return true;
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == Orientation::Colinear && onSegment(p2, p1, q2))
        return true;
    // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == Orientation::Colinear && onSegment(p2, q1, q2))
        return true;
    return false; // Doesn't fall in any of the above cases 
}

//--------------------------------------------------------------------------------
/// Get intersection point of the two lines if it exists
Point intersectionPoint(const Point &L1_s, const Point &L1_f,
                        const Point &L2_s, const Point &L2_f)
{
    // Line_1 represented as a1*x + b1*y = c1
    Point::value_type a1 = L1_f.y - L1_s.y;
    Point::value_type b1 = L1_s.x - L1_f.x;
    Point::value_type c1 = a1 * (L1_s.x) + b1 * (L1_s.y);

    // Line_2 represented as a2*x + b2*y = c2
    Point::value_type a2 = L2_f.y - L2_s.y;
    Point::value_type b2 = L2_s.x - L2_f.x;
    Point::value_type c2 = a2 * (L2_s.x) + b2 * (L2_s.y);

    Point::value_type determinant = a1 * b2 - a2 * b1;

    if (determinant == 0)
    {
        // The lines are parallel.
        // This is simplified by returning a pair of FLT_MAX
        return Point(FLT_MAX, FLT_MAX);
    }
    return Point{ (b2 * c1 - b1 * c2) / determinant,
                  (a1 * c2 - a2 * c1) / determinant };
}
//--------------------------------------------------------------------------------
Point alongLineAtDistance(const Point &from, const Point &to, Point::value_type distance)
{
    Point dirLn{ to.x - from.x, to.y - from.y };
    dirLn /= dirLn.norm2();
    return Point{ from + dirLn * distance };
}
//--------------------------------------------------------------------------------
tostream& operator<<(tostream &s, const Point &p)
{ return s << _T('(') << p.x_ << _T(", ") << p.y_ << _T(')'); }
tistream& operator>>(tistream &s, Point &p)
{ return s >> ConfirmInput(_T("(")) >> p.x_ >> ConfirmInput(_T(", ")) >> p.y_ >> ConfirmInput(_T(")")); }
//--------------------------------------------------------------------------------
void Point::save(tptree &node) const
{
    tptree xelem, yelem;
    xelem.put_value(x); node.push_back(std::make_pair(_T(""), xelem));
    yelem.put_value(y); node.push_back(std::make_pair(_T(""), yelem));
}
void Point::load(tptree &node)
{
    assert(node.size() == 2);
    x = node.front().second.get_value<Point::value_type>();
    y = node.back().second.get_value<Point::value_type>();
}
