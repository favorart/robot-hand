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
inline win_point* WindowSize()
{
    static win_point Width_Height;
    return &Width_Height;
}
// установить актуальные размеры окна при изменении
inline void setWindowSize(int Width_, int Height_)
{
    WindowSize()->x = Width_;
    WindowSize()->y = Height_;
}

// вещественные логические координаты     ( -1 < x <1;    -1<y<1     ) 
// преобразуем в координаты окна Windows  (  0 < x <WIDTH; 0<y<HIGHT )
uint_t Tx (double logic_x);
uint_t Ty (double logic_y);

Point LogicCoords (win_point* coord);
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

tstring   getCurrentTimeString (tstring format, std::time_t *the_time=NULL);
tstring   getWindowTitleString (HWND hWnd);
tstring   getLastErrorString ();
//-------------------------------------------------------------------------------
struct MyWindowData;

void  redirectConsoleIO ();
void  getConsoleArguments (tstring &config, tstring &database);
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

void onWindowCreate (HWND &hWnd, RECT &myRect,
                     HWND &hLabCanv, HWND &hLabHelp,
                     HWND &hLabMAim, HWND &hLabTest,
                     HWND &hLabStat, LabelsPositions &lp,
                     tstring &jointsHelp);
void onWindowSize   (HWND &hWnd, RECT &myRect,
                     HWND &hLabCanv, HWND &hLabHelp,
                     HWND &hLabMAim, HWND &hLabTest,
                     HWND &hLabStat, LabelsPositions &lp);

void onWindowPaint   (HWND &hWnd, RECT &myRect,
                      MyWindowData &wd);
void onWindowKeyDown (HWND &hWnd, RECT &myRect,
                      WPARAM wParam, LPARAM lParam,
                      MyWindowData &wd);
//-------------------------------------------------------------------------------
#endif // _WINDOW_H_
