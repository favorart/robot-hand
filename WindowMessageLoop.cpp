#include "StdAfx.h"
#include "MyWindow.h"
#include "WindowData.h"
#pragma warning (disable: 4996)

using namespace std;
using namespace HandMoves;

#define  WM_USER_TIMER  WM_USER+3
#define  WM_USER_STORE  WM_USER+4

//------------------------------------------------------------------------------
LRESULT CALLBACK  WndProc (HWND hWnd, UINT messg, WPARAM wParam, LPARAM lParam)
{
  static HWND  hLabHelp, hLabCanv /* Canvas */, hLabMAim, hLabTest, hLabStat;
  static LabelsPositions lp;
  static RECT    myRect;
  //------------------------------------------------------------------------------
  static MyWindowData  *wd;
  //------------------------------------------------------------------------------
  switch ( messg )
  {
    case WM_CREATE:
    { 
      OnWindowCreate (hWnd, myRect, hLabCanv, hLabHelp,
                      hLabMAim, hLabTest, hLabStat, lp);
      //=======================
      // RedirectIOToConsole ();
      //=======================
      wd = new MyWindowData (hLabMAim, hLabTest, hLabStat);
      //=======================
      SendMessage (hWnd, WM_USER_STORE, NULL, NULL);
      break;
    }

    case WM_PAINT:
    { //=======================
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
      OnWindowSize (hWnd, myRect, hLabCanv, hLabHelp,
                    hLabMAim, hLabTest, hLabStat, lp);
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
      if ( wd->store_size != wd->store.size () )
        SendMessage (hWnd, WM_USER_STORE, NULL, NULL);

      InvalidateRect (hWnd, &myRect, TRUE);
      break;
    }

    case WM_USER_STORE:
    { //=======================
      {
        // tstringstream buffer;
        // buffer << _T ("Storage size ") << wd->store.size () << _T("  ");

        wd->store_size = wd->store.size ();
        tstring str_size = str (boost::wformat (_T ("Storage size %1%  ")) % wd->store_size);

        /* Setting the Label's text */
        SendMessage (hLabStat,         /* Label Stat */
                     WM_SETTEXT,       /* Message    */
                     (WPARAM) NULL,    /* Unused     */
                     (LPARAM) str_size.c_str ()); // buffer.str ().c_str ());
      }
      //=======================
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
    { /* узнаём координаты */
      wd->mouse_coords.x = LOWORD (lParam);
      wd->mouse_coords.y = HIWORD (lParam);
      //=======================
      if ( (LONG) wd->mouse_coords.x > myRect.left
        && (LONG) wd->mouse_coords.x < myRect.right
        && (LONG) wd->mouse_coords.y > myRect.top
        && (LONG) wd->mouse_coords.y < myRect.bottom )
      { OnWindowMouse (*wd); }
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
