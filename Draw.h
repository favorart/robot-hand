#include "StdAfx.h"

#ifndef  _DRAW_H_
#define  _DRAW_H_

#include "MyWindow.h"
//------------------------------------------------------------------------------
void  draw_decards_coordinates (HDC hdc);
void  draw_simple_hand_moving  (HDC hdc   /* ��������, ���� ������������ */, 
	                              ulong_t time /* ����� ����� */ );
void  draw_trajectory (HDC hdc, std::list<Point> &walk_through, HPEN hPen);
//------------------------------------------------------------------------------
#endif // _DRAW_H_
