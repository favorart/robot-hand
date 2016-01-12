#include "StdAfx.h"
#include "WindowData.h"
#include "Draw.h"
#include "target.h"
#pragma warning (disable: 4996)

using namespace std;
using namespace HandMoves;
//------------------------------------------------------------------------------
win_point*  WindowSize (void)
{ static win_point Width_Height;
  return &Width_Height;
}
void     SetWindowSize (int Width_, int Height_)
{ WindowSize ()->x = Width_;
  WindowSize ()->y = Height_;
}

int  Tx (double logic_x)
{ return  (int) (MARGIN + ( 1.0 / 2) * (logic_x + 1) *
                 (WindowSize ()->x - 2 * MARGIN));
}
int  Ty (double logic_y)
{ return  (int) (MARGIN + (-1.0 / 2) * (logic_y - 1) * 
                 (WindowSize ()->y - 2 * MARGIN));
}

Point  logic_coord (win_point* coord)
{ Point p;
  p.x = ((coord->x - MARGIN) / (( 1.0 / 2)*(WindowSize ()->x - 2 * MARGIN))) - 1;
  p.y = ((coord->y - MARGIN) / ((-1.0 / 2)*(WindowSize ()->y - 2 * MARGIN))) + 1;
  return p;
}
//-------------------------------------------------------------------------------
void OnWindowCreate (HWND &hWnd, RECT &myRect,
                     HWND &hLabCanv, HWND &hLabHelp)
{
  RECT Rect;

  GetClientRect (hWnd, &Rect);
  myRect.left = Rect.left;
  myRect.top = Rect.top;
  myRect.bottom = Rect.bottom;
  myRect.right = Rect.left + (Rect.bottom - Rect.top);

  // Create a Static Label control
  hLabCanv = CreateWindow (_T ("STATIC"),					 	/* The name of the static control's class */
                           _T ("Canvas  "),							                      /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT,            /* Styles (continued) */
                           Rect.bottom - Rect.top,                          /* X co-ordinates */
                           Rect.top,                                        /* Y co-ordinates */
                           Rect.right - (Rect.bottom - Rect.top),                    /* Width */
                           25,                                                      /* Height */
                           hWnd,                                               /* Parent HWND */
                           (HMENU) IDL_CANVAS,											     		/* The Label's ID */
                           NULL,                             /* The HINSTANCE of your program */
                           NULL);                               /* Parameters for main window */
  if ( !hLabCanv )
    MessageBox (hWnd, _T ("Could not create Label1."), _T ("Error"), MB_OK | MB_ICONERROR);

  // Create a Static Label control
  hLabHelp = CreateWindow (_T ("STATIC"),						/* The name of the static control's class */
                           _T ("Help"),								 				                /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT,            /* Styles (continued) */
                           Rect.bottom - Rect.top,                          /* X co-ordinates */
                           Rect.top + 25,                                   /* Y co-ordinates */
                           Rect.right - (Rect.bottom - Rect.top),                    /* Width */
                           Rect.bottom - Rect.top - 25,                             /* Height */
                           hWnd,                                               /* Parent HWND */
                           (HMENU) IDL_HELP,						     				     		/* The Label's ID */
                           NULL,                             /* The HINSTANCE of your program */
                           NULL);                               /* Parameters for main window */
  if ( !hLabHelp )
    MessageBox (hWnd, _T ("Could not create Label2."), _T ("Error"), MB_OK | MB_ICONERROR);

  // Generate the help string
  { TCHAR string_help[1024];

  _stprintf (string_help,
             _T ("Клавиши управления:  \r\rEsc - выход  \rEnter - авто-тест  \rR - сбросить  \r")
             _T ("Z - двинуть ключицей вправо  \rX - сомкнуть плечо  \rС - сомкнуть локоть  \r")
             _T ("A - двинуть ключицей влево   \rS - раскрыть плечо  \rD - раскрыть локоть  \r\r")
             _T ("Повторное нажатие на кнопку во время движения  \rостанавливает соответствующее движение.  \r\r")
             _T ("T - для продолжения обучения  \r\rКвадрат цели 10x10 точек  \r\rДля выбора цели отрисовки  \r")
             //_T ("M + !no!/%2u + Enter,  \rN + !no!/%2u + Enter   \r, где 0 <= !no! - номер строки/столбца"),
             //tgRowsCount,
             //tgColsCount
             );

  // Setting the Label's text
  SendMessage (hLabHelp,         /* Label   */
               WM_SETTEXT,       /* Message */
               (WPARAM) NULL,    /* Unused  */
               (LPARAM) string_help);
  }

  RegisterHotKey (hWnd, HK_EXIT, (UINT) NULL, 0x1B); // 'Esc'							

  SetTimer (hWnd,							 /* Handle to main window */
            IDT_TIMER,         /* timer identifier 			*/
            100,               /* 1-second interval 		*/
            (TIMERPROC) NULL); /* no timer callback			*/
}

void OnWindowSize (HWND &hWnd, RECT &myRect,
                   HWND &hLabCanv, HWND &hLabHelp)
{
  RECT Rect;

  GetClientRect (hWnd, &Rect);
  myRect.left = Rect.left;
  myRect.top = Rect.top;
  myRect.bottom = Rect.bottom;
  myRect.right = Rect.left + (Rect.bottom - Rect.top);

  SetWindowSize ( /* Rect.right - Rect.left */
                 Rect.bottom - Rect.top,
                 Rect.bottom - Rect.top);

  /* Set position noCanvas label */
  SetWindowPos (hLabCanv, NULL,
                Rect.bottom - Rect.top,                   /* X co-ordinates */
                Rect.top,                                 /* Y co-ordinates */
                Rect.right - (Rect.bottom - Rect.top),            /*  Width */
                25,                                               /* Height */
                (UINT) NULL);
  /* Set position help label */
  SetWindowPos (hLabHelp, NULL,
                Rect.bottom - Rect.top,                   /* X co-ordinates */
                Rect.top + 25,                            /* Y co-ordinates */
                Rect.right - (Rect.bottom - Rect.top),            /* Width  */
                Rect.bottom - Rect.top - 25,                      /* Height */
                (UINT) NULL);
}

void OnWindowPaint (HWND &hWnd, RECT &myRect,
                    MyWindowData &wd)
{
  PAINTSTRUCT ps;
  HBITMAP     hBmp;
  HDC         hdc, hCmpDC;

  hdc = BeginPaint (hWnd, &ps);
  //-------------------------------------------
  /* Создание теневого контекста для двойной буфферизации */
  hCmpDC = CreateCompatibleDC (hdc);
  hBmp = CreateCompatibleBitmap (hdc, myRect.right - myRect.left,
                                      myRect.bottom - myRect.top);
  SelectObject (hCmpDC, hBmp);

  /* Закраска фона рабочей области */
  { HBRUSH brush = CreateSolidBrush (RGB (235, 235, 255) /* RGB (255,204,238) background color */);
    FillRect (hCmpDC, &myRect, brush);
    DeleteObject (brush);
  }

  /* set transparent brush to fill shapes */
  SelectObject (hCmpDC, wd.hBrush_null);
  // SetBkMode (hCmpDC, TRANSPARENT);
  //-------------------------------------
  /* Здесь рисуем на контексте hCmpDC */
  draw_decards_coordinates (hCmpDC);
  wd.target.draw (hCmpDC, wd.hPen_grn);

  OnPaintMyLogic (hCmpDC, wd);
  //-------------------------------------
  /* Копируем изображение из теневого контекста на экран */
  SetStretchBltMode (hdc, COLORONCOLOR);
  BitBlt (hdc, 0, 0,
          myRect.right  - myRect.left,
          myRect.bottom - myRect.top,
          hCmpDC, 0, 0,
          SRCCOPY);

  /* Удаляем ненужные системные объекты */
  DeleteDC (hCmpDC);
  DeleteObject (hBmp);
  hCmpDC = NULL;
  //---------------------------------------------
  EndPaint (hWnd, &ps);
}
//-------------------------------------------------------------------------------
void OnWindowKeyDown (HWND &hWnd, RECT &myRect,
                      WPARAM wParam, LPARAM lparam,
                      MyWindowData &wd)
{
  switch ( wParam )
  {
    // case 0x0D: /* Process a carriage return */
    //   fm = 0; fn = 0;
    //   /* Фон не будет переписовываться */
    //   InvalidateRect (hWnd, &myRect, TRUE /*0*/);
    //   break;

    case 't':
    { //========================================
      OnRandomTest (wd);
      MessageBox (hWnd, _T ("Done\n"), 
                  _T ("RandomTest"), MB_OK);
      //========================================
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    }

    case 'o':
    { //========================================
      OnShowTrajectory (wd);
      //========================================
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    }

    case 'p':
    {
      //========================================
      OnCoverTest (wd);

      std::wstring message;
      for ( auto t : wd.testing_trajectories )
        message += std::wstring (t->back ()) + std::wstring (_T ("\n"));

      MessageBox (hWnd, message.c_str (),
                  str (boost::wformat (_T ("CoverTest %1%")) %
                  wd.testing_trajectories.size ()).c_str (),
                  MB_OK);
      //========================================
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    }

    //========================================
    /* Clavicle */
    case 'z': wd.hand.step (Hand::ClvclCls); break; /* двинуть ключицей вправо */
    case 'a': wd.hand.step (Hand::ClvclOpn); break; /* двинуть ключицей влево */
    /* Sholder */
    case 'x': wd.hand.step (Hand::ShldrCls); break;
    case 's': wd.hand.step (Hand::ShldrOpn); break;
    /* Elbow */
    case 'c': wd.hand.step (Hand::ElbowCls); break;
    case 'd': wd.hand.step (Hand::ElbowOpn); break;
    /* Reset */
    case 'r': 
      wd.hand.set (Hand::Clvcl | Hand::Shldr | Hand::Elbow, { 50., 50., 50. });
      
      wd.pointsDB.clear ();
      wd.trajectoriesDB.clear ();

      wd.mouse_haved = false;

      wd.trajectory_frames_show = false;
      wd.testing_trajectories.clear ();
      wd.trajectory_frames.clear ();
      break;
    //========================================
    // case 'm': if ( !fn ) fm = 1; break;
    // case 'n': if ( !fm ) fn = 1; break;

    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
      //      if ( fm ) p_x = p_x * 10U + ((uchar_t) wParam - 0x30);
      // else if ( fn ) p_y = p_y * 10U + ((uchar_t) wParam - 0x30);

      // if ( p_x >= tgRowsCount ) p_x = tgRowsCount - 1U;
      // if ( p_y >= tgColsCount ) p_y = tgColsCount - 1U;
      //========================================
      break;
  }
}
//-------------------------------------------------------------------------------
