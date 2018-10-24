#include "Point.h"

//--------------------------------------------------------------------------------
const Point&  Point::operator= (const Point &p)
{ if ( this != &p )
  { x = p.x; y = p.y; }
  return *this;
}
//--------------------------------------------------------------------------------
/* compare operator */
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
bool  Point::hit    (const Point &p, double eps) const
{ return ((abs (x_ - p.x_) < eps) && (abs (y_ - p.y_) < eps)); }
bool  Point::hit(const Point &p) const
{ return hit(p, Utils::EPSILONT); }


void  Point::rotate_radians (const Point &center, double angle)
{
    double  tmp;

    x_ -= center.x_;
    y_ -= center.y_;

    tmp = (x_ * cos(angle)) - (y_ * sin(angle));
    y_  = (x_ * sin(angle)) + (y_ * cos(angle));
    x_  = tmp;

    x_ += center.x_;
    y_ += center.y_;
}

double Point::angle (const Point &p) const
{ return atan2(p.y - y, p.x - x) * 180. / M_PI; }
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
Point intersectionLines(const Point &L1_s, const Point &L1_f,
                        const Point &L2_s, const Point &L2_f)
{
    // Line_1 represented as a1*x + b1*y = c1
    double a1 = L1_f.y - L1_s.y;
    double b1 = L1_s.x - L1_f.x;
    double c1 = a1 * (L1_s.x) + b1 * (L1_s.y);

    // Line_2 represented as a2*x + b2*y = c2
    double a2 = L2_f.y - L2_s.y;
    double b2 = L2_s.x - L2_f.x;
    double c2 = a2 * (L2_s.x) + b2 * (L2_s.y);

    double determinant = a1 * b2 - a2 * b1;

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
Point alongLineAtDistance(const Point &from, const Point &to, double distance)
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
    x = node.front().second.get_value<double>();
    y = node.back().second.get_value<double>();
}
