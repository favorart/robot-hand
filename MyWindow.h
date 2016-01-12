#include "StdAfx.h"

#ifndef  _WINDOW_H_
#define  _WINDOW_H_
//--------------------------------------------------------------------------------
#define   SCALE      1/100
#define   MARGIN     10
#define   WIDTH      950
#define   HIGHT      600
//--------------------------------------------------------------------------------
struct win_point { uint_t x, y; };

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
/* Return TRUE if file 'fileName' exists */
inline bool  isFileExists (const TCHAR *fileName)
{ DWORD  fileAttr = GetFileAttributes (fileName);
  return (0xFFFFFFFF != fileAttr);
}
//-------------------------------------------------------------------------------
template <typename T>
T  random (T max)
{ return  (T (rand ()) % (max + 1)); }
template <typename T>
T  random (T min, T max)
{ return  (min + T (rand ()) % (max + 1)); }
//-------------------------------------------------------------------------------
class MyWindowData;
//-------------------------------------------------------------------------------
void OnWindowCreate (HWND &hWnd, RECT &myRect,
                     HWND &hLabCanv, HWND &hLabHelp);
void OnWindowSize   (HWND &hWnd, RECT &myRect,
                     HWND &hLabCanv, HWND &hLabHelp);

void OnPaintMyLogic (HDC hdc,
                     MyWindowData &params);
void OnWindowPaint  (HWND &hWnd, RECT &myRect,
                     MyWindowData &wd);

void OnWindowKeyDown (HWND &hWnd, RECT &myRect,
                      WPARAM wParam, LPARAM lParam,
                      MyWindowData &wd);
//-------------------------------------------------------------------------------
void  OnWindowTimer (MyWindowData &wd);
void  OnWindowMouse (MyWindowData &wd);

void  OnRandomTest  (MyWindowData &wd);
void  OnCoverTest   (MyWindowData &wd);

void  OnShowTrajectory (MyWindowData &wd);

void  OnShowDBPoints       (MyWindowData &wd);
void  OnShowDBTrajectories (MyWindowData &wd);
//-------------------------------------------------------------------------------
#endif // _WINDOW_H_
