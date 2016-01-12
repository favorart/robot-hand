#include "StdAfx.h"
#include "target.h"

//------------------------------------------------------------------------------
void  RecTarget::draw (HDC hdc, HPEN hPen) const // matrix 80 * 80
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
//void insertTargetToHandMovesStore (Target &T, HandMoves::Store &store)
//{ for ( auto it = T.coords_.begin (); it != T.coords_.end (); ++it )
//  { store.insert ( (*it), (*it), /* ... list */ ); }
//}
//------------------------------------------------------------------------------


