#include "StdAfx.h"
#include "MyWindow.h"
#include "WindowData.h"
#pragma warning (disable: 4996)

using namespace std;
using namespace HandMoves;

#define  WM_USER_TIMER  WM_USER+3
//------------------------------------------------------------------------------
LRESULT CALLBACK  WndProc (HWND hWnd, UINT messg, WPARAM wParam, LPARAM lParam)
{
  static HWND  hLabHelp;
  static HWND  hLabCanv; // Canvas
  static RECT    myRect;
  //------------------------------------------------------------------------------
  static MyWindowData  *wd;
  //------------------------------------------------------------------------------
  switch ( messg )
  {
    case WM_CREATE:
    { //=======================
      OnWindowCreate (hWnd, myRect, hLabCanv, hLabHelp);

      wd = new MyWindowData ();
      //=======================
      break;
    }

    case WM_PAINT:
    { 
      if ( wd->store.size () )
        SetWindowText (hWnd, 
                       boost::str (boost::wformat (_T ("Store size = %1%")) 
                       % wd->store.size ()).c_str ());
      //=======================
      OnWindowPaint (hWnd, myRect, *wd);
      //=======================
      break;
    }

    // make background color of labels
    case WM_CTLCOLORSTATIC:
    { HDC hdcStatic = (HDC) (wParam);
      SetBkMode (hdcStatic, TRANSPARENT);
      return (LRESULT) CreateSolidBrush (RGB (255, 255, 255));
    }

    case WM_ERASEBKGND:
      return 1;

    case WM_QUIT:
      SendMessage (hWnd, WM_DESTROY, 0, 0);
      break;

    case WM_DESTROY:
      delete wd;
      PostQuitMessage (0);
      break;

    case WM_MOVE:
    case WM_SIZE:
      OnWindowSize (hWnd, myRect, hLabCanv, hLabHelp);
      break;

    case WM_GETMINMAXINFO:
      ((MINMAXINFO*) lParam)->ptMinTrackSize.x = WIDTH;
      ((MINMAXINFO*) lParam)->ptMinTrackSize.y = HIGHT;
      break;

    case WM_USER_TIMER:
      KillTimer (hWnd, IDT_TIMER);
      break;

    case WM_TIMER:
    { //=======================
      OnWindowTimer (*wd);
      //=======================
      InvalidateRect (hWnd, &myRect, TRUE);
      break;
    }

    case WM_HOTKEY:
      switch ( wParam )
      { case HK_EXIT:
          SendMessage (hWnd, WM_QUIT, 0, 0);
          break;
      }
      break;
      
    case WM_LBUTTONDOWN: /* Если был щелчок левой кнопкой */
    {	/* узнаём координаты */
      wd->mouse_coords.x = LOWORD (lParam);
      wd->mouse_coords.y = HIWORD (lParam);
      //=======================
      OnWindowMouse (*wd);
      //=======================
      InvalidateRect (hWnd, &myRect, 0);
      break;
    }

    case WM_CHAR:
    { //=======================
      OnWindowKeyDown (hWnd, myRect, wParam, lParam, *wd);
      //=======================
      break;
    }

    default:
      return (DefWindowProc (hWnd, messg, wParam, lParam));
  }

  return (0);
}
//------------------------------------------------------------------------------
