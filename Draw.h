#include "StdAfx.h"

#ifndef  _DRAW_H_
#define  _DRAW_H_

#include "MyWindow.h"
//------------------------------------------------------------------------------
void  DrawDecardsCoordinates (HDC hdc);
void  DrawTrajectory (HDC hdc, std::list<Point> &walk_through, HPEN hPen);
void  DrawTrajectory (HDC hdc, std::list<std::shared_ptr<Point>> &trajectory, HPEN hPen);

typedef enum { emptyfig=0, ellipse=1, rectangle=2 } figure_t;
void  DrawAdjacency (HDC hdc, const Point &center, double radius, figure_t figure, HPEN hPen);
//------------------------------------------------------------------------------
#endif // _DRAW_H_
