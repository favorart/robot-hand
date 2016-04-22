#include "StdAfx.h"

#include "Draw.h"
//------------------------------------------------------------------------------
void  DrawDecardsCoordinates (HDC hdc)
{ uint_t i = 90U;
 //---Ox---
  MoveToEx (hdc, Tx (-1.000), Ty ( 0.000), NULL);
  LineTo   (hdc, Tx ( 1.000), Ty ( 0.000));
  LineTo   (hdc, Tx ( 0.970), Ty ( 0.010));
  LineTo   (hdc, Tx ( 0.990), Ty ( 0.000));
  LineTo   (hdc, Tx ( 0.970), Ty (-0.010));
  LineTo   (hdc, Tx ( 1.000), Ty ( 0.000));
  //---Oy---              
  MoveToEx (hdc, Tx ( 0.000), Ty (-1.000), NULL);
  LineTo   (hdc, Tx ( 0.000), Ty ( 1.000));
  LineTo   (hdc, Tx (-0.005), Ty ( 0.960));
  LineTo   (hdc, Tx ( 0.000), Ty ( 0.990));
  LineTo   (hdc, Tx ( 0.008), Ty ( 0.960));
  LineTo   (hdc, Tx ( 0.000), Ty ( 1.000));
 //--------
 for(i=0; i<19; i++)
 {
   MoveToEx (hdc, Tx (-0.90 + 0.1*i), Ty (-0.01        ), NULL);
   LineTo   (hdc, Tx (-0.90 + 0.1*i), Ty ( 0.01        ) );
   MoveToEx (hdc, Tx (-0.01        ), Ty (-0.90 + 0.1*i), NULL);
   LineTo   (hdc, Tx ( 0.01        ), Ty (-0.90 + 0.1*i) );
 }
}
//------------------------------------------------------------------------------
void  DrawTrajectory (HDC hdc, const HandMoves::trajectory_refs_t &trajectory, HPEN hPen)
{
  if ( !trajectory.empty () )
  {
    HPEN hPen_old = (HPEN) SelectObject (hdc, hPen);
    //------------------------------------------------------------------
    const auto &p = trajectory.front ();
    MoveToEx (hdc, Tx (p->x), Ty (p->y), NULL);
    for ( const auto &p : trajectory )
    { LineTo (hdc, Tx (p->x), Ty (p->y)); }
    //------------------------------------------------------------------
    // отменяем ручку
    SelectObject (hdc, hPen_old);
  }
}
void  DrawTrajectory (HDC hdc, const HandMoves::trajectory_t      &trajectory, HPEN hPen)
{
  if ( !trajectory.empty () )
  {
    HPEN hPen_old = (HPEN) SelectObject (hdc, hPen);
    //------------------------------------------------------------------
    const Point &p = trajectory.front ();
    MoveToEx (hdc, Tx (p.x), Ty (p.y), NULL);

    for ( const Point &p : trajectory )
    { LineTo (hdc, Tx (p.x), Ty (p.y)); }
    //------------------------------------------------------------------
    // отменяем ручку
    SelectObject (hdc, hPen_old);
  }
}

void  DrawAdjacency (HDC hdc, const Point &center, double radius,
                     figure_t figure, HPEN hPen)
{
  HPEN hPen_old = (HPEN) SelectObject (hdc, hPen);
  switch ( figure )
  {
    case ellipse:     Ellipse (hdc, Tx (center.x - radius), Ty (center.y + radius),
                                    Tx (center.x + radius), Ty (center.y - radius)); break;
    case rectangle: Rectangle (hdc, Tx (center.x - radius), Ty (center.y + radius),
                                    Tx (center.x + radius), Ty (center.y - radius)); break;
  }
  SelectObject (hdc, hPen_old);
}
//------------------------------------------------------------------------------

