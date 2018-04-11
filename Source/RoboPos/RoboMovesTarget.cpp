#include "StdAfx.h"
#include "RoboMovesTarget.h"


//------------------------------------------------------------------------------
void  RecTarget::generate ()
{
  double r_step = (max_.y - min_.y) / (c_rows - 1U);
  double c_step = (max_.x - min_.x) / (c_cols - 1U);

  thickness_ = Point(r_step, c_step);

  for ( uint_t i = 0U; i < c_rows; ++i )
    for ( uint_t j = 0U; j < c_cols; ++j )
      coords_.push_back (Point (/* left   */ min_.x + j * c_step,
                                /* bottom */ min_.y + i * r_step));

  min_.x -= EPS; min_.y -= EPS;
  max_.x += EPS; max_.y += EPS;
}
//------------------------------------------------------------------------------
void  RecTarget::draw (HDC hdc, HPEN hPen,
                       bool internalLines,
                       bool internalPoints,
                       bool ellipseInsteadPixel) const
{
  HPEN hPen_old = (HPEN) SelectObject (hdc, hPen);
  //---target--------------------------------------------------------
  double r = (max_.y - min_.y) / (c_rows - 1U);
  double c = (max_.x - min_.x) / (c_cols - 1U);

  Rectangle (hdc, Tx (/* lft */ min_.x - c / 2.), Ty (/* top */ min_.y - r / 2.),
                  Tx (/* rgh */ max_.x + c / 2.), Ty (/* btm */ max_.y + r / 2.));
  //-----------------------------------------------------------------
  if ( internalLines )
  {
    for ( uint_t i = 1U; i < c_cols; ++i )
    {
      MoveToEx (hdc, Tx (/* lft */ min_.x + i*c - c / 2.), Ty (/* top */ min_.y - r / 2.), NULL);
      LineTo   (hdc, Tx (/* lft */ min_.x + i*c - c / 2.), Ty (/* btm */ max_.y + r / 2.));
    }
    for ( uint_t i = 1U; i < c_rows; ++i )
    {
      MoveToEx (hdc, Tx (/* lft */ min_.x - c / 2.), Ty (/* btm */ min_.y + i*r - r / 2.), NULL);
      LineTo   (hdc, Tx (/* rgh */ max_.x + c / 2.), Ty (/* btm */ min_.y + i*r - r / 2.));
    } // end for
  } // end if
  //-----------------------------------------------------------------
  if ( internalPoints )
  {
    COLORREF color = 0U;
    if ( !ellipseInsteadPixel)
    {
      LOGPEN logpen = {};
      GetObject (hPen, sizeof (LOGPEN), &logpen);
      color = logpen.lopnColor;
    }

    for( auto p : coords_ )
    { 
      if (ellipseInsteadPixel)
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
