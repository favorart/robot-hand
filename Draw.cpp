#include "StdAfx.h"
#include "MyWindow.h"
#include "Draw.h"
#include "Hand.h"
#include "HandMovesStore.h"


//template <class integer>
//integer   random_input (integer max, integer min)
//{ return (max) ? ((min + (integer)rand ()) % max) : (srand ( (integer)clock () ), (integer) 0); }

ulong_t   random_input (ulong_t max, ulong_t min)
{ return (max) ? ((min + (ulong_t) rand ()) % max) : (srand ((ulong_t) clock ()), (ulong_t) 0); }
//------------------------------------------------------------------------------
void  target_draw (const HDC &hdc, const HPEN &hPen)
{ const double  lft_btm = -0.4;
  const double  rgt_top =  0.0;
	const double    r_ell =  0.011;
  const uint_t        m = 3U;
	const uint_t        n = (uint_t) (10U * (abs (rgt_top) + abs (lft_btm)) + 1U);

	HPEN hPen_old = (HPEN) SelectObject (hdc, hPen);
	//---target--------------------------------------------------------
	// Arc (hdc, Tx (-1.00), Ty (-0.50), Tx (-0.50), Ty (-1.00), 0, 0, 0, 0 );

	Rectangle (hdc, Tx (lft_btm), Ty (rgt_top), Tx (rgt_top), Ty (lft_btm));
	for(uint_t i=0U; i<m; i++)
		for(uint_t j=0U; j<m; j++)
		 Ellipse (hdc, Tx (lft_btm + (0.1*(double)i) - r_ell), Ty (rgt_top - (0.1*(double)j) + r_ell),
		               Tx (lft_btm + (0.1*(double)i) + r_ell), Ty (rgt_top - (0.1*(double)j) - r_ell) );
  //-----------------------------------------------------------------
  // отменяем ручку
	SelectObject (hdc, hPen_old); 			
}
//------------------------------------------------------------------------------
#define NSHOTS   35U
#define RELLPS   0.03

void  draw_decards_coordinates (HDC &hdc)
{ uint_t i = 0U;
	 //---Ox---
	 MoveToEx (hdc, Tx (-1.000), Ty ( 0.000), NULL);
	 LineTo   (hdc, Tx ( 1.000), Ty ( 0.000) );
	 LineTo   (hdc, Tx ( 0.970), Ty ( 0.010) );
	 LineTo   (hdc, Tx ( 0.990), Ty ( 0.000) );
	 LineTo   (hdc, Tx ( 0.970), Ty (-0.010) );
	 LineTo   (hdc, Tx ( 1.000), Ty ( 0.000) );
	 //---Oy---													 
	 MoveToEx (hdc, Tx ( 0.000), Ty(-1.000), NULL);
	 LineTo   (hdc, Tx ( 0.000), Ty( 1.000));
	 LineTo   (hdc, Tx (-0.005), Ty( 0.960));
	 LineTo   (hdc, Tx ( 0.000), Ty( 0.990));
	 LineTo   (hdc, Tx ( 0.008), Ty( 0.960));
	 LineTo   (hdc, Tx ( 0.000), Ty( 1.000));
	 //--------
	for(i=0; i<19; i++)
	{
		 MoveToEx (hdc, Tx (-0.90 + 0.1*i), Ty (-0.01        ), NULL);
		 LineTo   (hdc, Tx (-0.90 + 0.1*i), Ty ( 0.01        ) );
		 MoveToEx (hdc, Tx (-0.01        ), Ty (-0.90 + 0.1*i), NULL);
		 LineTo   (hdc, Tx ( 0.01        ), Ty (-0.90 + 0.1*i) );
	}
}
void  draw_simple_hand_moving  (HDC &hdc   /* контекст, куда отрисовывать */, 
	                               ulong_t time /* номер кадра */ )
{ Point hand (-0.3, 0.8), arm (0.3, 0.5), sholder (0.8, 0.0);

	int i = time % (2 * NSHOTS);
	//-----------------------------------------------------------------
	 arm.rotate (sholder, 2.2 * ((i > NSHOTS) ? 2 * NSHOTS - i : i));
	hand.rotate (sholder, 2.2 * ((i > NSHOTS) ? 2 * NSHOTS - i : i));
	hand.rotate (arm    , 3.8 * ((i > NSHOTS) ? 2 * NSHOTS - i : i));

	//-----------------------------------------------------------------
	MoveToEx (hdc, Tx (      1.0), Ty (sholder.y), NULL);
	LineTo   (hdc, Tx (sholder.x), Ty (sholder.y) );
	LineTo   (hdc, Tx (    arm.x), Ty (    arm.y) ); 
 	LineTo   (hdc, Tx (   hand.x), Ty (   hand.y) );
																														 
	//---sholder--------------------------------------------------------
	Ellipse (hdc, Tx (-RELLPS + sholder.x), Ty ( RELLPS + sholder.y),
		            Tx ( RELLPS + sholder.x), Ty (-RELLPS + sholder.y) );

	//---arm------------------------------------------------------------
	Ellipse (hdc, Tx (-RELLPS + arm.x), Ty ( RELLPS + arm.y),
		            Tx ( RELLPS + arm.x), Ty (-RELLPS + arm.y) );

	//---hand_-----------------------------------------------------------
	Ellipse (hdc, Tx (-RELLPS + hand.x), Ty ( RELLPS + hand.y),
		            Tx ( RELLPS + hand.x), Ty (-RELLPS + hand.y) );
	//------------------------------------------------------------------
}

#undef NSHOTS
#undef RELLPS
//------------------------------------------------------------------------------

//#include <gdiplus.h>
/* Drawing the trajectory, where we got ... */
void  draw_trajectory (HDC &hdc, std::list<Point> &walk_through, const HPEN hPen)
{
  if ( !walk_through.empty () )
  {
    HPEN hPen_old = (HPEN) SelectObject (hdc, hPen);
    //------------------------------------------------------------------
    Point &p = walk_through.front ();
    MoveToEx (hdc, Tx (p.x), Ty (p.y), NULL);

    for ( auto p : walk_through )
    { LineTo (hdc, Tx (p.x), Ty (p.y)); }
    //------------------------------------------------------------------
    // отменяем ручку
    SelectObject (hdc, hPen_old);
  }
}
//------------------------------------------------------------------------------
void  draw_learnt_points(HDC hdc, std::list<Point> &points, HandMoves::Store &store)
{

}
//------------------------------------------------------------------------------

