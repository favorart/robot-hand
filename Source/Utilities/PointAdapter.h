#include "StdAfx.h"
#include "Point.h"

#pragma once

namespace boost {
namespace geometry {
namespace traits {

template<> struct tag<Point>
{ typedef point_tag type; };

template<> struct coordinate_type<Point>
{ typedef Point::value_type type; };

template<> struct coordinate_system<Point>
{ typedef cs::cartesian type; };

template<> struct dimension<Point> : boost::mpl::int_<2> {};

template<>
struct access<Point, 0>
{
    static Point::value_type get(const Point &p)
    { return p.get_x(); }
    static void set(Point& p, const Point::value_type& value)
    { p.put_x(value); }
};

template<>
struct access<Point, 1>
{
    static Point::value_type get(Point const& p)
    { return p.get_y(); }
    static void set(Point& p, Point::value_type const& value)
    { p.put_y(value); }
};
}
}
} // namespace boost::geometry::traits

