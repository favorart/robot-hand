#include "StdAfx.h"

#ifndef  _WINDOW_H_
#define  _WINDOW_H_

#include <commdlg.h>
//--------------------------------------------------------------------------------
#define   SCALE      1/100
#define   MARGIN     10
#define   WIDTH      950
#define   HIGHT      600
//--------------------------------------------------------------------------------
struct win_point { uint_t x, y; };

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
/* Return TRUE if file 'fileName' exists */
inline bool  isFileExists (const TCHAR *fileName)
{ DWORD  fileAttr = GetFileAttributes (fileName);
  return (0xFFFFFFFF != fileAttr);
}

tstring   OpenFileDialog (HWND hWnd);
tstring   SaveFileDialog (HWND hWnd);

tstring   CurrentTimeToString (tstring format, std::time_t *the_time=NULL);

tstring   GetLastErrorToString ();

tstring   GetTextToString (HWND hWnd);
//-------------------------------------------------------------------------------
template <typename INTEGER>
INTEGER  random (INTEGER max)
{ return  (max) ? (static_cast<INTEGER> (rand ()) % (max)) : (max); }
template <typename INTEGER>
INTEGER  random (INTEGER min, INTEGER max)
{ return  (max) ? (static_cast<INTEGER> (rand ()) % (max) + min) : (max); }

inline double  random (double max)
{ return  (static_cast<double> (rand ()) / RAND_MAX) * max; }
inline double  random (double min, double max)
{ return  (static_cast<double> (rand ()) / RAND_MAX) * (max - min) + min; }
//-------------------------------------------------------------------------------
class MyWindowData;

void RedirectIOToConsole ();
//-------------------------------------------------------------------------------
struct LabelsPositions
{
  size_t LabelsLeft;
  size_t LabelsWidth;
  size_t LabCanvTop;
  size_t LabCanvHeight;
  size_t LabHelpTop;
  size_t LabHelpHeight;
  size_t LabMAimTop;
  size_t LabMAimHeight;
  size_t LabTestTop;
  size_t LabTestHeight;
  size_t LabStatTop;
  size_t LabStatHeight;
};

void OnWindowCreate (HWND &hWnd, RECT &myRect,
                     HWND &hLabCanv, HWND &hLabHelp,
                     HWND &hLabMAim, HWND &hLabTest,
                     HWND &hLabStat, LabelsPositions &lp);
void OnWindowSize   (HWND &hWnd, RECT &myRect,
                     HWND &hLabCanv, HWND &hLabHelp,
                     HWND &hLabMAim, HWND &hLabTest,
                     HWND &hLabStat, LabelsPositions &lp);

void OnWindowPaint   (HWND &hWnd, RECT &myRect,
                      MyWindowData &wd);
void OnWindowKeyDown (HWND &hWnd, RECT &myRect,
                      WPARAM wParam, LPARAM lParam,
                      MyWindowData &wd);
//-------------------------------------------------------------------------------
inline void  DrawCircle (HDC hdc, const Point &center, double radius)
{ Ellipse (hdc, Tx (-radius + center.x), Ty ( radius + center.y),
                Tx ( radius + center.x), Ty (-radius + center.y));
}
inline void  DrawCircle (HDC hdc, const Point &center, double radius, HPEN hPen)
{
  HPEN Pen_old = (HPEN) SelectObject (hdc, hPen);
  Ellipse (hdc, Tx (-radius + center.x), Ty (radius + center.y),
                Tx (radius + center.x), Ty (-radius + center.y));
  SelectObject (hdc, Pen_old);
}
//-------------------------------------------------------------------------------
typedef  std::pair<COLORREF,COLORREF> color_interval_t;
typedef  std::vector<COLORREF> gradient_t;

void      MakeGradient (color_interval_t  colors,
                        size_t            n_levels,
                        gradient_t       &gradient);
//-------------------------------------------------------------------------------
#define _ANIMATION_

#endif // _WINDOW_H_
