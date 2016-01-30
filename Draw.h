#include "StdAfx.h"

#ifndef  _DRAW_H_
#define  _DRAW_H_

#include "MyWindow.h"
//------------------------------------------------------------------------------
void  DrawDecardsCoordinates (HDC hdc);
void  DrawTrajectory (HDC hdc, std::list<Point> &walk_through, HPEN hPen);

typedef enum { ellipse = 1, rectangle } figure_t;
void  DrawAdjacency (HDC hdc, const Point &center, double radius,
                     figure_t figure, HPEN hPen_cian);
//------------------------------------------------------------------------------
#endif // _DRAW_H_
