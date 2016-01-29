#include "StdAfx.h"
#include "WindowData.h"
#include "Draw.h"


#pragma warning (disable: 4996) // allow ANSI C functions
#define _CRT_SECURE_NO_WARNINGS

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
                     HWND &hLabCanv, HWND &hLabHelp,
                     HWND &hLabMAim, HWND &hLabTest,
                     HWND &hLabStat, LabelsPositions &lp)
{
  RECT Rect;

  GetClientRect (hWnd, &Rect);
  myRect.left = Rect.left;
  myRect.top = Rect.top;
  myRect.bottom = Rect.bottom;
  myRect.right = Rect.left + (Rect.bottom - Rect.top);

  lp.LabelsLeft    = Rect.bottom - Rect.top;
  lp.LabelsWidth   = Rect.right - (Rect.bottom - Rect.top);
  lp.LabCanvTop    = Rect.top;
  lp.LabCanvHeight = 24U;
  lp.LabHelpTop    = Rect.top + 25U;
  lp.LabHelpHeight = (Rect.bottom - Rect.top) * 2U / 3U - 1U;
  lp.LabMAimTop    = Rect.top +  25U + (Rect.bottom - Rect.top) * 2U / 3U;
  lp.LabMAimHeight = 49U;
  lp.LabTestTop    = Rect.top +  75U + (Rect.bottom - Rect.top) * 2U / 3U;
  lp.LabTestHeight = 49U;
  lp.LabStatTop    = Rect.top + 125U + (Rect.bottom - Rect.top) * 2U / 3U;
  lp.LabStatHeight = Rect.bottom - ((Rect.bottom - Rect.top) * 2U / 3U - 125U);

  // Create a Static Label control
  hLabCanv = CreateWindow (_T ("STATIC"),          /* The name of the static control's class */
                           _T ("Canvas  "),                              /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT | WS_BORDER,       /* Styles (continued) */
                           lp.LabelsLeft,                                    /* X co-ordinates */
                           lp.LabCanvTop,                                    /* Y co-ordinates */
                           lp.LabelsWidth,                                            /* Width */
                           lp.LabCanvHeight,                                         /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_CANVAS,                        /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabCanv )
    MessageBox (hWnd, _T ("Could not create hLabCanv."), _T ("Error"), MB_OK | MB_ICONERROR);

  // Create a Static Label control
  hLabHelp = CreateWindow (_T ("STATIC"),        /* The name of the static control's class */
                           _T ("Help"),                              /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT | WS_BORDER,             /* Styles (continued) */
                           lp.LabelsLeft,                                    /* X co-ordinates */
                           lp.LabCanvTop,                                    /* Y co-ordinates */
                           lp.LabelsWidth,                                            /* Width */
                           lp.LabCanvHeight,                                         /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_HELP,                       /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabHelp )
    MessageBox (hWnd, _T ("Could not create hLabHelp."), _T ("Error"), MB_OK | MB_ICONERROR);

  // Create a Static Label control
  hLabMAim = CreateWindow (_T ("STATIC"),        /* The name of the static control's class */
                           _T (" "),                                /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT | WS_BORDER,             /* Styles (continued) */
                           lp.LabelsLeft,                                    /* X co-ordinates */
                           lp.LabCanvTop,                                    /* Y co-ordinates */
                           lp.LabelsWidth,                                            /* Width */
                           lp.LabCanvHeight,                                         /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_HELP,                        /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabMAim )
    MessageBox (hWnd, _T ("Could not create hLabMAim."), _T ("Error"), MB_OK | MB_ICONERROR);

  // Create a Static Label control
  hLabTest = CreateWindow (_T ("STATIC"),        /* The name of the static control's class */
                           _T (" "),                                /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT | WS_BORDER,             /* Styles (continued) */
                           lp.LabelsLeft,                                    /* X co-ordinates */
                           lp.LabCanvTop,                                    /* Y co-ordinates */
                           lp.LabelsWidth,                                            /* Width */
                           lp.LabCanvHeight,                                         /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_HELP,                        /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabTest )
    MessageBox (hWnd, _T ("Could not create hLabTest."), _T ("Error"), MB_OK | MB_ICONERROR);

  // Create a Static Label control
  hLabStat = CreateWindow (_T ("STATIC"),        /* The name of the static control's class */
                           _T (" "),                                /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT | WS_BORDER,             /* Styles (continued) */
                           lp.LabelsLeft,                                    /* X co-ordinates */
                           lp.LabCanvTop,                                    /* Y co-ordinates */
                           lp.LabelsWidth,                                            /* Width */
                           lp.LabCanvHeight,                                         /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_HELP,                        /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabStat )
    MessageBox (hWnd, _T ("Could not create hLabStat."), _T ("Error"), MB_OK | MB_ICONERROR);

  // Generate the help string
  { TCHAR string_help[1024];

  _stprintf (string_help,
             _T ("Клавиши управления:  \r\rEsc - выход  \r\rR - сбросить всё  \r\r") // Enter - авто-тест  \r
             _T ("Z - двинуть ключицей вправо  \rX - сомкнуть плечо  \rС - сомкнуть локоть  \r")
             _T ("A - двинуть ключицей влево  \rS - раскрыть плечо  \rD - раскрыть локоть  \r\r")
             _T ("Повторное нажатие на кнопку во время движения  \rостанавливает соответствующее движение.  \r\r")
             _T ("U - нарисовать рабочую область руки  \rO - нарисовать случайную траекторию  \r\r")
             _T ("P - Cover Test  \rT - Random Test  \r\r")
             //_T ("Квадрат цели 10x10 точек  \r\rДля выбора цели отрисовки  \r")
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

  SetTimer (hWnd,        /* Handle to main window */
            IDT_TIMER,         /* timer identifier    */
            50,         // sec/2      /* 1-second interval   */
            (TIMERPROC) NULL); /* no timer callback   */
}

void OnWindowSize (HWND &hWnd, RECT &myRect,
                   HWND &hLabCanv, HWND &hLabHelp,
                   HWND &hLabMAim, HWND &hLabTest,
                   HWND &hLabStat, LabelsPositions &lp)
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

  lp.LabelsLeft = Rect.bottom - Rect.top;
  lp.LabelsWidth = Rect.right - (Rect.bottom - Rect.top);
  lp.LabCanvTop = Rect.top;
  lp.LabCanvHeight = 24U;
  lp.LabHelpTop = Rect.top + 25U;
  lp.LabHelpHeight = (Rect.bottom - Rect.top) * 2U / 3U - 1U;
  lp.LabMAimTop = Rect.top + 25U + (Rect.bottom - Rect.top) * 2U / 3U;
  lp.LabMAimHeight = 49U;
  lp.LabTestTop = Rect.top + 75U + (Rect.bottom - Rect.top) * 2U / 3U;
  lp.LabTestHeight = 49U;
  lp.LabStatTop = Rect.top + 125U + (Rect.bottom - Rect.top) * 2U / 3U;
  lp.LabStatHeight = Rect.bottom - ((Rect.bottom - Rect.top) * 2U / 3U - 125U);

  /* Set position noCanvas label */
  SetWindowPos (hLabCanv, NULL,
                lp.LabelsLeft,     /* X co-ordinates */
                lp.LabCanvTop,     /* Y co-ordinates */
                lp.LabelsWidth,             /* Width */
                lp.LabCanvHeight,          /* Height */
                (UINT) NULL);   
  /* Set position help label */ 
  SetWindowPos (hLabHelp, NULL, 
                lp.LabelsLeft,     /* X co-ordinates */
                lp.LabHelpTop,     /* Y co-ordinates */
                lp.LabelsWidth,             /* Width */
                lp.LabHelpHeight,          /* Height */
                (UINT) NULL);   
  /* Set position help label */ 
  SetWindowPos (hLabMAim, NULL, 
                lp.LabelsLeft,     /* X co-ordinates */
                lp.LabMAimTop,     /* Y co-ordinates */
                lp.LabelsWidth,             /* Width */
                lp.LabMAimHeight,          /* Height */
                (UINT) NULL);
  /* Set position help label */
  SetWindowPos (hLabTest, NULL, 
                lp.LabelsLeft,     /* X co-ordinates */
                lp.LabTestTop,     /* Y co-ordinates */
                lp.LabelsWidth,             /* Width */
                lp.LabTestHeight,          /* Height */
                (UINT) NULL);   
  /* Set position help label */ 
  SetWindowPos (hLabStat, NULL, 
                lp.LabelsLeft,     /* X co-ordinates */
                lp.LabStatTop,     /* Y co-ordinates */
                lp.LabelsWidth,             /* Width */
                lp.LabStatHeight,          /* Height */
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
void  /*HandMoves::*/ testCoverTarget (Store &store, Hand &hand, RecTarget &target);

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
      // MessageBox (hWnd, _T ("Done\n"), 
      //             _T ("RandomTest"), MB_OK);
      //========================================
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    }

    case 'o':
    { //========================================
      // OnShowTrajectory (wd);
      
      wd.trajectory_frames.clear ();
      wd.hand.SET_DEFAULT;

      wd.trajectory_frames_muscle = selectHandMove (random (HandMovesCount));
      wd.trajectory_frames_lasts = random (wd.hand.maxMuscleLast (wd.trajectory_frames_muscle));

      // wd.trajectory_frames_muscle = Hand::ShldrCls;
      // wd.trajectory_frames_lasts = 25U;

      wd.hand.move (wd.trajectory_frames_muscle, wd.trajectory_frames_lasts, wd.trajectory_frames);

      auto aim = wd.hand.position;
      auto rec = Record (aim, aim, { wd.trajectory_frames_muscle },
                         { 0U }, { wd.trajectory_frames_lasts },
                         1U, wd.trajectory_frames);
      wd.store.insert (rec);

      wd.trajectory_frames.clear ();
      wd.trajectory_frames_muscle = Hand::EmptyMov;
      wd.trajectory_frames_lasts = 0;
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

    case 'y':
    {
      testCoverTarget (wd.store, wd.hand, wd.target);
      break;
    }

    case 'q':
    { //========================================
      OnShowDBPoints (wd);
      //========================================
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    }

    case 'i':
    { int muscle, last;
      cin >> muscle >> last;
      wd.hand.move ((Hand::MusclesEnum)muscle, (Hand::time_t)last);
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    }

    case 'u':
    {

      Hand::MusclesEnum muscle;
      wd.hand.set (Hand::Clvcl | Hand::Shldr | Hand::Elbow, { 0.,0.,0. });

      muscle = Hand::ClvclOpn;
      wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.trajectory_frames);

      muscle = Hand::ShldrCls;
      wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.trajectory_frames);
      
      muscle = Hand::ClvclCls;
      wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.trajectory_frames);

      muscle = Hand::ElbowCls;
      wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.trajectory_frames);
      
      muscle = Hand::ShldrOpn;
      wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.trajectory_frames);
       
      muscle = Hand::ElbowOpn;
      wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.trajectory_frames);
      
      wd.hand.SET_DEFAULT;
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
      wd.hand.SET_DEFAULT;
      
      wd.reach = false;

      wd.pointsDB.clear ();
      wd.trajectoriesDB.clear ();

      wd.mouse_haved = false;

      wd.trajectory_frames_show = false;
      wd.testing_trajectories.clear ();
      wd.trajectory_frames.clear ();

      /* Setting the Label's text */
      SendMessage (wd.hLabTest,      /* Label   */
                   WM_SETTEXT,       /* Message */
                   (WPARAM) NULL,    /* Unused  */
                   (LPARAM) _T (" "));

      /* Setting the Label's text */
      SendMessage (wd.hLabMAim,      /* Label   */
                   WM_SETTEXT,       /* Message */
                   (WPARAM) NULL,    /* Unused  */
                   (LPARAM) _T (" "));

      /* Setting the Label's text */
      SendMessage (wd.hLabStat,      /* Label   */
                   WM_SETTEXT,       /* Message */
                   (WPARAM) NULL,    /* Unused  */
                   (LPARAM) _T (" "));
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
