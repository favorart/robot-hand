#include "StdAfx.h"
#include "target.h"


//------------------------------------------------------------------------------
void  RecTarget::generate ()
{
  double r_step = (top - btm) / (c_rows - 1U);
  double c_step = (rgh - lft) / (c_cols - 1U);

  for ( uint_t i = 0U; i < c_rows; ++i )
  { for ( uint_t j = 0U; j < c_cols; ++j )
    { coords_.push_back (Point (lft + j * c_step,
                                btm + i * r_step));
    }
  }
  x_distance = r_step;
  y_distance = c_step;
}
//------------------------------------------------------------------------------
void  RecTarget::draw (HDC hdc, HPEN hPen,
                       bool internalLines,
                       bool internalPoints,
                       bool ellipseOrPixel) const
{
  HPEN hPen_old = (HPEN) SelectObject (hdc, hPen);
  //---target--------------------------------------------------------
  double r = (top - btm) / (c_rows - 1U);
  double c = (rgh - lft) / (c_cols - 1U);

  Rectangle (hdc, Tx (lft - c / 2.0), Ty (top + r / 2.0),
                  Tx (rgh + c / 2.0), Ty (btm - r / 2.0));
  //-----------------------------------------------------------------
  if ( internalLines )
  {
    for ( uint_t i = 1U; i < c_cols; ++i )
    {
      MoveToEx (hdc, Tx (lft + i*c - c / 2.0), Ty (btm - r / 2.0), NULL);
      LineTo   (hdc, Tx (lft + i*c - c / 2.0), Ty (top + r / 2.0));
    }
    for ( uint_t i = 1U; i < c_rows; ++i )
    {
      MoveToEx (hdc, Tx (lft - c / 2.0), Ty (btm + i*r - r / 2.0), NULL);
      LineTo   (hdc, Tx (rgh + c / 2.0), Ty (btm + i*r - r / 2.0));
    } // end for
  } // end if
  //-----------------------------------------------------------------
  if ( internalPoints )
  {
    COLORREF color = 0U;
    if ( !ellipseOrPixel )
    {
      LOGPEN logpen = {};
      GetObject (hPen, sizeof (LOGPEN), &logpen);
      color = logpen.lopnColor;
    }

    for( auto p : coords_ )
    { 
      if ( ellipseOrPixel )
      {
        const double  REllipse = 0.007;
        Ellipse (hdc, Tx (p.x - REllipse), Ty (p.y + REllipse),
                      Tx (p.x + REllipse), Ty (p.y - REllipse));
      }
      else
      { SetPixel (hdc, Tx (p.x), Ty (p.y), color); } // end else
    } // end for
  } // end if
  //-----------------------------------------------------------------
  SelectObject (hdc, hPen_old);
}
//------------------------------------------------------------------------------
