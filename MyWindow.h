#include "StdAfx.h"

#ifndef  _WINDOW_H_
#define  _WINDOW_H_
//--------------------------------------------------------------------------------
#define   SCALE      1/100
#define   MARGIN     10
#define   WIDTH      950
#define   HIGHT      600
//--------------------------------------------------------------------------------

typedef struct win_point_ win_point;
struct win_point_ { uint_t x, y; };

// ���������� ���������� - ������ ���������� ������� ����
win_point*  WindowSize (void);

// ���������� ���������� ������� ���� ��� ���������
void  SetWindowSize (int Width_, int Height_);

// ������������ ���������� ����������     ( -1 < x <1;    -1<y<1     ) 
// ����������� � ���������� ���� Windows  (  0 < x <WIDTH; 0<y<HIGHT )
int  Tx (double logic_x);
int  Ty (double logic_y);

// ��������: ���������� Windows -> ���������� ����������
Point  logic_coord (win_point* coord);
//-------------------------------------------------------------------------------
// ��������� ��������� ���������
LRESULT CALLBACK  WndProc (HWND, UINT, WPARAM, LPARAM);
//-------------------------------------------------------------------------------
#endif // _WINDOW_H_
