#include "StdAfx.h"
#include "WindowDraw.h"
#include "RoboMovesTarget.h"


//------------------------------------------------------------------------------
void  RecTarget::generate ()
{
  double r_step = (max_.y - min_.y) / (c_rows - 1U);
  double c_step = (max_.x - min_.x) / (c_cols - 1U);

  thickness_ = Point(r_step, c_step);

  for (auto i = 0U; i < c_rows; ++i)
    for (auto j = 0U; j < c_cols; ++j)
      coords_.push_back (Point (/* left   */ min_.x + j * c_step,
                                /* bottom */ min_.y + i * r_step));

  min_.x -= Utils::EPSILONT; min_.y -= Utils::EPSILONT;
  max_.x += Utils::EPSILONT; max_.y += Utils::EPSILONT;
}
//------------------------------------------------------------------------------
void  RecTarget::draw (HDC hdc, HPEN hPen,
                       bool internalLines,
                       bool internalPoints,
                       double internalPointsRadius) const
{
  double r = (max_.y - min_.y) / (c_rows - 1U);
  double c = (max_.x - min_.x) / (c_cols - 1U);

  drawMyFigure(hdc,
               { (max_.x + min_.x) / 2, (max_.y + min_.y) / 2 },
               (max_.x - min_.x) + c, (max_.y - min_.y) + r, 0.,
               MyFigure::Rectangle, hPen);
  //-----------------------------------------------------------------
  if ( internalLines )
  {
    for (auto i = 1U; i < c_cols; ++i)
        drawLine(hdc, { /* lft */ min_.x + i * c - c / 2., /* top */ min_.y - r / 2. },
                      { /* lft */ min_.x + i * c - c / 2., /* btm */ max_.y + r / 2. }, hPen);

    for (auto i = 1U; i < c_rows; ++i)
        drawLine(hdc, { /* lft */ min_.x - c / 2., /* btm */ min_.y + i * r - r / 2. },
                      { /* rgh */ max_.x + c / 2., /* btm */ min_.y + i * r - r / 2. }, hPen);
  }
  //-----------------------------------------------------------------
  if ( internalPoints )
  {
      for (auto p : coords_)
          drawCircle(hdc, p, internalPointsRadius, hPen);
  }
}
//------------------------------------------------------------------------------
