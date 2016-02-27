﻿#include "StdAfx.h"
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
      // RedirectIOToConsole ();
      //=======================
      wd = new MyWindowData (hLabMAim, hLabTest, hLabStat);
      //=======================
      OnWindowCreate (hWnd, myRect, hLabCanv, hLabHelp,
                      hLabMAim, hLabTest, hLabStat, lp);

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
      wd->hStaticBitmapChanged = true;
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
      { SendMessage (hWnd, WM_USER_STORE, NULL, NULL);
        if ( !wd->testing )  wd->hStaticBitmapChanged = true;
      }
      InvalidateRect (hWnd, &myRect, TRUE);
      break;
    }

    case WM_USER_STORE:
    { //=======================
      {
        // tstringstream ss;
        // ss << _T ("Storage size ") << wd->store.size () << _T("  ");

        wd->store_size = wd->store.size ();
        tstring str_size = str (boost::wformat (_T ("Storage size %1%  ")) % wd->store_size);

        /* Setting the Label's text */
        SendMessage (hLabStat,         /* Label Stat */
                     WM_SETTEXT,       /* Message    */
                     (WPARAM) NULL,    /* Unused     */
                     (LPARAM) str_size.c_str ()); // ss.str ().c_str ());
      }
      //=======================
      break;
    }

    case WM_HOTKEY:
      switch ( wParam )
      {
        case HK_EXIT:
          SendMessage (hWnd, WM_QUIT, 0, 0);
          break;

        case HK_OPEN: // open input text file
        { 
          tstring  FileName = OpenFileDialog (hWnd);
          tstring  DefaultName = wd->CurFileName;

          if ( !FileName.empty () )
          {
            WorkerThreadRunStoreTask (*wd, _T ("  *** loading ***  "),
                                      [FileName, DefaultName](HandMoves::Store &store)
                                      {
                                        if ( !store.empty () )
                                        { storeSave (store, DefaultName);
                                          store.clear ();
                                        }
                                        storeLoad (store, FileName);
                                      
                                      });
            wd->CurFileName = FileName;
          }
          break;
        }

        case HK_SAVE:
        { 
          // tstring  FileName = SaveFileDialog (hWnd);
          if ( !wd->store.empty () )
          { tstringstream ss;
            ss << HAND_NAME
               << CurrentTimeToString (_T ("_%d-%m-%Y_%I-%M-%S")) 
               << _T ("_moves.bin");

            WorkerThreadRunStoreTask (*wd, _T ("  *** saving ***  "),
                                      storeSave, ss.str ());

            /* Setting the Label's text */
            SendMessage (hLabStat,         /* Label Stat */
                         WM_SETTEXT,       /* Message    */
                         (WPARAM) NULL,    /* Unused     */
                         (LPARAM) _T ("  *** saved ***  "));
          }          
          break;
        }
        
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
