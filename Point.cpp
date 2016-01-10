#include "StdAfx.h"
#include "Point.h"

//--------------------------------------------------------------------------------
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

std::istream&  operator>> (std::istream &in,        Point &p)
{ return in >> p.x_ >> p.y_; }
std::ostream&  operator<< (std::ostream &out, const Point &p)
{ return out << _T ('(') << p.x_ << _T (", ") << p.y_ << _T (')'); }
//--------------------------------------------------------------------------------
