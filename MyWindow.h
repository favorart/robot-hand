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

// глобальные переменные - хранят актуальные размеры окна
win_point*  WindowSize (void);

// установить актуальные размеры окна при изменении
void  SetWindowSize (int Width_, int Height_);

// вещественные логические координаты     ( -1 < x <1;    -1<y<1     ) 
// преобразуем в координаты окна Windows  (  0 < x <WIDTH; 0<y<HIGHT )
int  Tx (double logic_x);
int  Ty (double logic_y);

// наоборот: координаты Windows -> логические координаты
Point  logic_coord (win_point* coord);
//-------------------------------------------------------------------------------
// Процедуры обработки сообщений
LRESULT CALLBACK  WndProc (HWND, UINT, WPARAM, LPARAM);
//-------------------------------------------------------------------------------
#endif // _WINDOW_H_
