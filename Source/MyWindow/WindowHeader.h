#include "StdAfx.h"

#ifndef  _WINDOW_H_
#define  _WINDOW_H_

#include <commdlg.h>
//--------------------------------------------------------------------------------
#define   SCALE      1/100
#define   MARGIN     10
#define   WIDTH      950
#define   HIGHT      600

#define  WM_USER_TIMER  WM_USER+3
#define  WM_USER_STORE  WM_USER+4
//--------------------------------------------------------------------------------
/// глобальные переменные - хранят актуальные размеры окна
inline PPOINT WindowSize()
{
    static POINT Width_Height;
    return &Width_Height;
}
/// установить актуальные размеры окна при изменении
inline void setWindowSize(int Width_, int Height_)
{
    WindowSize()->x = Width_;
    WindowSize()->y = Height_;
}

// вещественные логические координаты     ( -1 < x <1;    -1<y<1     ) 
// преобразуем в координаты окна Windows  (  0 < x <WIDTH; 0<y<HIGHT )

LONG Tx (double logic_x);
LONG Ty (double logic_y);

Point LogicCoords (PPOINT coord);
//-------------------------------------------------------------------------------
/// Процедуры обработки сообщений
LRESULT CALLBACK  WndProc (HWND, UINT, WPARAM, LPARAM);
//-------------------------------------------------------------------------------
/// \return TRUE if file with fileName exists on disk
inline bool isFileExists (const TCHAR *fileName)
{ DWORD  fileAttr = GetFileAttributes (fileName);
  return (0xFFFFFFFF != fileAttr);
}

inline SIZE textLength(HDC hdc, const tstring &s)
{ 
    SIZE sz{};
    if (!GetTextExtentPoint32(hdc, s.c_str(), s.length(), &sz))
        throw std::runtime_error("!GetTextExtentPoint32A");
    return sz;
}

enum class CGradient { None, Longz, Dense, Strats, _Last_ };

tstring   OpenFileDialog (HWND hWnd);
tstring   SaveFileDialog (HWND hWnd);

tstring   getCurrentTimeString (tstring format, std::time_t *the_time=NULL);
tstring   getWindowTitleString (HWND hWnd);
//-------------------------------------------------------------------------------
void  redirectConsoleIO ();
void  getConsoleArguments (tstring &config, tstring &database);
//-------------------------------------------------------------------------------
struct MyWindowData;

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

void onWindowCreate  (HWND hWnd, MyWindowData &wd);
void onWindowSize    (HWND hWnd, MyWindowData &wd);
void onWindowPaint   (HWND hWnd, MyWindowData &wd);
void onWindowTimer   (HWND hWnd, MyWindowData &wd, WPARAM wParam);
void onWindowStoreSz (HWND hWnd, MyWindowData &wd);
void onWindowMouse   (HWND hWnd, MyWindowData &wd, WPARAM wParam, LPARAM lParam);
void onWindowChar    (HWND hWnd, MyWindowData &wd, WPARAM wParam, LPARAM lparam);
void onWindowKeyDown (HWND hWnd, MyWindowData &wd, WPARAM wParam);
void onWindowMsWheel (HWND hWnd, MyWindowData &wd, WPARAM WParam, LPARAM LParam);
//-------------------------------------------------------------------------------
inline LONG width(PRECT rect)
{ return abs(rect->right - rect->left); }
inline LONG height(PRECT rect)
{ return abs(rect->bottom - rect->top); }
inline bool inside(PRECT rect, PPOINT pt)
{
    return  (pt->x > rect->left && pt->x < rect->right &&
             pt->y > rect->top  && pt->y < rect->bottom);
}
//-------------------------------------------------------------------------------

#endif // _WINDOW_H_
