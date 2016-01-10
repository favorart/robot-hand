#include "StdAfx.h"
#include "target.h"

//------------------------------------------------------------------------------
// Point*   Target::HitH (const Point &h) const
// { set_t::iterator it = coords_.find (h);
// 	return ( it == coords_.end () ) ? NULL : (Point*)&(*it); // !!!
// }

// flann::Matrix<double>  RecTarget::Generate ()
// { double r_step = (top - btm) / (c_rows - 1U);
//   double c_step = (rgh - lft) / (c_cols - 1U);
// 
// 		for(double i=lft; i<=rgh; i+=c_step)
// 			for(double j=btm; j<=top; j+=r_step)
// 			{ Point p (i,j);
// 		   coords_.insert (p);
// 			}
// 
//   average_distance_ = MIN (c_step, r_step);
//   
//   flann::Matrix<double> m;
//   return m;
// }

void  RecTarget::draw (const HDC &hdc, const HPEN &hPen) const // matrix 80*80
{ const double  REllipse = 0.007;
 
  HPEN hPen_old = (HPEN) SelectObject (hdc, hPen);
	//---target--------------------------------------------------------
	double r = (top - btm) / (c_rows-1U);
	double c = (rgh - lft) / (c_cols-1U);

	Rectangle (hdc, Tx (lft - c/2.0), Ty (top + r/2.0),
                  Tx (rgh + c/2.0), Ty (btm - r/2.0));
	
	for(uint_t i=1U; i<c_cols; ++i)
	{ MoveToEx (hdc, Tx (lft + i*c - c/2.0), Ty (btm - r/2.0), NULL);
			LineTo (hdc, Tx (lft + i*c - c/2.0), Ty (top + r/2.0) );
	}
	for(uint_t i=1U; i<c_rows; ++i)
	{ MoveToEx (hdc, Tx (lft - c/2.0), Ty (btm + i*r - r/2.0), NULL);
			LineTo (hdc, Tx (rgh + c/2.0), Ty (btm + i*r - r/2.0)  );
	}

	for( set_t::const_iterator it  = coords_.cbegin ();
                             it != coords_.cend   (); ++it)
	 Ellipse (hdc, Tx (it->x - REllipse), Ty (it->y + REllipse),
                 Tx (it->x + REllipse), Ty (it->y - REllipse));
	//-----------------------------------------------------------------
	// отменяем ручку
	SelectObject (hdc, hPen_old);
}
//------------------------------------------------------------------------------


