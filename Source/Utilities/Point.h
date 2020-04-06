#pragma once

#ifndef  _POINT_H_
#define  _POINT_H_
//--------------------------------------------------------------------------------
class Point
{
public:
    static unsigned w;
    static unsigned prec;
    static const unsigned ndimensions = 2;
    using value_type = double;

    Point() : x_(0.), y_(0.) {}
    Point(Point::value_type x, Point::value_type y) : x_(x), y_(y) {}
    Point(const Point &p) : x_(p.x), y_(p.y) {}
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
    void  rotate(const Point &center, Point::value_type angle)
    { rotate_radians(center, angle * M_PI / 180.); }
    void  rotate_degrees(const Point &center, Point::value_type angle)
    { rotate_radians(center, angle * M_PI / 180.); }
    void  rotate_radians(const Point &center, Point::value_type angle);

    bool hit(const Point &p, Point::value_type eps) const;
    bool hit(const Point &p) const;
    Point::value_type angle(const Point &p) const;
    Point::value_type norm2() const { return sqrt(x * x + y * y); }
    Point orto() const { return Point{ -y,x }; }
    //-----------------------------------------
    /* Microsoft specific: C++ properties */
    __declspec(property(get = get_x, put = put_x)) Point::value_type x;
    Point::value_type get_x() const { return x_; }
    void put_x(Point::value_type x) { x_ = x; }

    __declspec(property(get = get_y, put = put_y)) Point::value_type y;
    Point::value_type get_y() const { return y_; }
    void put_y(Point::value_type y) { y_ = y; }
    //-----------------------------------------
    operator std::pair<Point::value_type, Point::value_type>() const
    { return std::make_pair(x, y); }
    operator boost::tuple<Point::value_type, Point::value_type>() const
    { return boost::make_tuple(x, y); }

    operator tstring() const
    {
        tstringstream ss;
        ss << std::setprecision(3) << _T("pt<x=") << x << _T(", y=") << y << _T(">");
        return ss.str();
    }
    void Point::save(tptree &node) const;
    void Point::load(tptree &node);
    //-------------------------------------------------------------------------------
    Point operator- (const Point &p) const { return Point{ x - p.x, y - p.y }; }
    Point operator+ (const Point &p) const { return Point{ x + p.x, y + p.y }; }
    Point operator* (const Point &p) const { return Point{ x * p.x, y * p.y }; }
    Point operator/ (const Point &p) const { return Point{ x / p.x, y / p.y }; }

    Point& operator+= (const Point &p) { x += p.x; y += p.y; return *this; }
    Point& operator-= (const Point &p) { x -= p.x; y -= p.y; return *this; }
    Point& operator*= (const Point &p) { x *= p.x; y *= p.y; return *this; }
    Point& operator/= (const Point &p) { x /= p.x; y /= p.y; return *this; }

    Point& operator+= (Point::value_type d) { x += d; y += d; return *this; }
    Point& operator-= (Point::value_type d) { x -= d; y -= d; return *this; }
    Point& operator*= (Point::value_type d) { x *= d; y *= d; return *this; }
    Point& operator/= (Point::value_type d) { x /= d; y /= d; return *this; }
    //-------------------------------------------------------------------------------
    friend tostream& operator<<(tostream &s, const Point &p);
    friend tistream& operator>>(tistream &s, Point &p);
    friend std::ostream& operator<<(std::ostream &s, const Point &p);
    friend std::istream& operator>>(std::istream &s, Point &p);

    //-------------------------------------------------------------------------------
    Point::value_type& operator[](size_t i);
    const Point::value_type& operator[](size_t i) const;

private:
    Point::value_type x_, y_;
    //-----------------------------------------
    /* serialization */
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned /*version*/) { ar & x_ & y_; }
}; // end class Point
//-------------------------------------------------------------------------------
inline Point rotate (const Point &p, const Point &center, Point::value_type angle)
{
    Point pt{ p };
    pt.rotate(center, angle);
    return pt;
}
inline Point rotate_radians(const Point &p, const Point &center, Point::value_type angle)
{
    Point pt{ p };
    pt.rotate_radians(center, angle);
    return pt;
}

/// Angle = L ABC
inline Point::value_type angle_radians (const Point &A, const Point &B, const Point &C)
{
    Point BA = (A - B), BC = (C - B);
    return atan2(BA.x * BC.y - BA.y * BC.x, BA.x * BC.x + BA.y * BC.y);
}
/// Angle = L ABC
inline Point::value_type angle_degrees (const Point &A, const Point &B, const Point &C)
{
    return angle_radians(A, B, C) * 180. / M_PI;
}

inline Point::value_type norm2 (const Point &p)
{ return p.norm2(); }
//--------------------------------------------------------------------------------
/// Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr' 
bool onSegment(const Point &p, const Point &q, const Point &r);

enum class LineSide { OnLine, Left, Right };
/// To determine which side of the line from-to a point pt falls on
LineSide onLineSide(const Point &from, const Point &to, const Point &pt);

enum class Orientation { Colinear, ClockWise, CounterClockWise };
/// To find orientation of ordered triplet (p, q, r)
Orientation orientation(const Point &p, const Point &q, const Point &r);

/// Returns true if the line segments 'p1q1' and 'p2q2' intersect
bool isIntersectLines(const Point &p1, const Point &q1, const Point &p2, const Point &q2);

/// Returns true if the point 'pt' lies inside the 'polygon' with n vertices
template <template <typename, typename> class Container,
    typename Value = Point,
    typename Allocator = std::allocator<Value> >
bool insidePolygon(const Container<Value, Allocator> &polygon, const Point &pt)
{
    // There must be at least 3 vertices in 'polygon'
    if (polygon.size() < 3)
        return false;

    // Create a point for line segment from 'pt' to infinite 
    Point extreme = { std::numeric_limits<Point::value_type>::max(), pt.y };
    // Count intersections of the above line with sides of 'polygon'
    int count = 0;
    for (auto it = polygon.begin(); it != polygon.end(); ++it)
    {
        auto next = std::next(it);
        if (next == polygon.end())
        {
            if (*it == polygon.front())
                break;
            next = polygon.begin();
        }
        // Check if the line segment from 'pt' to 'extreme'
        // intersects with the with line segment 'it-next' 
        if (isIntersectLines(*it, *next, pt, extreme))
        {
            // If the point 'pt' is colinear with line segment 'it-next', 
            // then it must lie on that segment to be inside.
            if (orientation(*it, pt, *next) == Orientation::Colinear)
                return onSegment(*it, pt, *next);
            count++;
        }
    }
    // Return true if count is odd, false otherwise 
    return (count & 1); // Same as (count%2 == 1) 
}
//--------------------------------------------------------------------------------
/// Get intersection point of the two lines if it exists
Point intersectionPoint(const Point &L1_s, const Point &L1_f,
                        const Point &L2_s, const Point &L2_f);
Point alongLineAtDistance(const Point &from, const Point &to, Point::value_type distance);

//-------------------------------------------------------------------------------
inline Point operator+ (const Point &p, Point::value_type d) { return Point{ p.x + d, p.y + d }; }
inline Point operator- (const Point &p, Point::value_type d) { return Point{ p.x - d, p.y - d }; }
inline Point operator* (const Point &p, Point::value_type d) { return Point{ p.x * d, p.y * d }; }
inline Point operator/ (const Point &p, Point::value_type d) { return Point{ p.x / d, p.y / d }; }
//-------------------------------------------------------------------------------
struct PointHasher
{
    std::size_t operator()(const Point& k) const
    {
        std::size_t seed = 0;
        // modify seed by xor and bit-shifting of the key members
        boost::hash_combine(seed, boost::hash_value(k.x));
        boost::hash_combine(seed, boost::hash_value(k.y));
        return seed;
    }
};
//-------------------------------------------------------------------------------
BOOST_CLASS_VERSION(Point, 2);
//------------------------------------------------------------------------------
#endif // _POINT_H_
