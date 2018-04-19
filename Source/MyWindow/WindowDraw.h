#include "StdAfx.h"

#ifndef  _DRAW_H_
#define  _DRAW_H_

#include "WindowHeader.h"
#include "Robo.h"
//------------------------------------------------------------------------------
void drawDecardsCoordinates(HDC hdc);
void drawCoordinates(HDC hdc, bool show_marks);

//------------------------------------------------------------------------------
void drawTrajectory(HDC hdc, const Robo::Trajectory &trajectory, HPEN hPen);
inline void drawTrajectory(HDC hdc, const Robo::Trajectory &trajectory)
{ drawTrajectory(hdc, trajectory, (HPEN)GetStockObject(BLACK_PEN)); }

//------------------------------------------------------------------------------
void drawLine(HDC hdc, const Point &s, const Point &e, HPEN hPen);
inline void drawLine(HDC hdc, const Point &s, const Point &e)
{ drawLine(hdc, s, e, (HPEN)GetStockObject(BLACK_PEN)); }

//------------------------------------------------------------------------------
void drawCircle(HDC hdc, const Point &center, double radius, HPEN hPen);
inline void drawCircle(HDC hdc, const Point &center, double radius)
{ drawCircle(hdc, center, radius, (HPEN)GetStockObject(BLACK_PEN)); }

//------------------------------------------------------------------------------
enum class MyFigure : uint8_t { EmptyFig = 0, Ellipse = 1, Rectangle = 2 };
void drawMyFigure(HDC hdc, const Point &center, double w, double h, double angle, MyFigure figure, HPEN hPen);

//------------------------------------------------------------------------------
using color_interval_t = std::pair<COLORREF, COLORREF>;
using color_gradient_t = std::vector<COLORREF>;

void makeGradient(color_interval_t colors, size_t color_gradations, color_gradient_t &gradient);
//------------------------------------------------------------------------------
#endif // _DRAW_H_
