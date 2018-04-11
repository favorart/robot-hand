#include "StdAfx.h"

#ifndef  _DRAW_H_
#define  _DRAW_H_

#include "WindowHeader.h"
#include "Robo.h"
//------------------------------------------------------------------------------
void  drawDecardsCoordinates(HDC hdc);
void  drawTrajectory(HDC hdc, const Robo::Trajectory &trajectory, HPEN hPen);

void  drawLine(HDC hdc, const Point &s, const Point &e, HPEN hPen);

enum class MyFigure : uint8_t { EmptyFig=0, Ellipse=1, Rectangle=2 };
void  drawMyFigure(HDC hdc, const Point &center, double w, double h, double angle, MyFigure figure, HPEN hPen);
//------------------------------------------------------------------------------
inline void  drawCircle(HDC hdc, const Point &center, double radius)
{
    Ellipse(hdc, Tx(-radius + center.x), Ty(+radius + center.y),
                 Tx(+radius + center.x), Ty(-radius + center.y));
}

inline void  drawCircle(HDC hdc, const Point &center, double radius, HPEN hPen)
{
    HPEN Pen_old = (HPEN)SelectObject(hdc, hPen);
    Ellipse(hdc, Tx(-radius + center.x), Ty(+radius + center.y),
                 Tx(+radius + center.x), Ty(-radius + center.y));
    SelectObject(hdc, Pen_old);
}
//------------------------------------------------------------------------------
using color_interval_t = std::pair<COLORREF, COLORREF>;
using color_gradient_t = std::vector<COLORREF>;

void  makeGradient(color_interval_t colors, size_t color_gradations, color_gradient_t &gradient);
//------------------------------------------------------------------------------
#endif // _DRAW_H_
