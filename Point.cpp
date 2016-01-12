#include "StdAfx.h"
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
// { return (x_ == p.x_) ? (y_ < p.y_) : (x_ < p.x_); }

bool  Point::operator== (const Point &p) const
{ return  hit (p); }
bool  Point::operator!= (const Point &p) const
{ return !hit (p); }
//--------------------------------------------------------------------------------
bool  Point::hit (const Point &p, double eps) const
{ return ((abs (x_ - p.x_) < eps)
       && (abs (y_ - p.y_) < eps));
}
void  Point::rotate (const Point &center, double angle)
{
  double  tmp, Angle = angle * M_PI / 180.;

  x_ -= center.x_;
	y_ -= center.y_;

	tmp = (x_ * cos (Angle)) - (y_ * sin (Angle));
	y_  = (x_ * sin (Angle)) + (y_ * cos (Angle));
	x_  =  tmp;		

	x_ += center.x_;
	y_ += center.y_;
}
//--------------------------------------------------------------------------------
std::istream&  operator>> (std::istream &in,        Point &p)
{ return in >> p.x_ >> p.y_; }
std::ostream&  operator<< (std::ostream &out, const Point &p)
{ return out << _T ('(') << p.x_ << _T (", ") << p.y_ << _T (')'); }
//--------------------------------------------------------------------------------
