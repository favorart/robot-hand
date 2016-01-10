#include "StdAfx.h"

#ifndef  _DRAW_H_
#define  _DRAW_H_

#include "MyWindow.h"

ulong_t   random_input (ulong_t max, ulong_t min = (ulong_t)0);
//------------------------------------------------------------------------------
//template <class integer>
//integer   random_input (integer max, integer min = (integer) 0);
//------------------------------------------------------------------------------
void  draw_decards_coordinates (HDC &hdc);
void  draw_simple_hand_moving  (HDC &hdc   /* контекст, куда отрисовывать */, 
	                              ulong_t time /* номер кадра */ );
void  draw_trajectory (HDC &hdc, std::list<Point> &walk_through, const HPEN hPen);
//------------------------------------------------------------------------------
#endif
