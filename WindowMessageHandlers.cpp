#include "StdAfx.h"
#include "WindowData.h"
#include "Draw.h"


#pragma warning (disable: 4996) // allow ANSI C functions
#define _CRT_SECURE_NO_WARNINGS

using namespace std;
using namespace HandMoves;


bool zoom = false;
//------------------------------------------------------------------------------
win_point*  WindowSize (void)
{ static win_point Width_Height;
  return &Width_Height;
}
void     SetWindowSize (int Width_, int Height_)
{ WindowSize ()->x = Width_;
  WindowSize ()->y = Height_;
}


int  T2x  (double logic_x)
{ return  (int) (MARGIN + ( 0.5) * (logic_x + 1.) * (WindowSize ()->x - 2. * MARGIN)); }
int  T2y  (double logic_y)
{ return  (int) (MARGIN + (-0.5) * (logic_y - 1.) * (WindowSize ()->y - 2. * MARGIN)); }

/* zoom */
int  T1x (double logic_x)
{ return  (int) (MARGIN + ( 1.) * (logic_x + 0.5) * (WindowSize ()->x - 2. * MARGIN)); }
int  T1y (double logic_y)
{ return  (int) (MARGIN + (-1.) * (logic_y - 0.0) * (WindowSize ()->y - 2. * MARGIN)); }

int  Tx (double logic_x)
{ return  (zoom) ? T1x (logic_x) : T2x (logic_x); }
int  Ty (double logic_y)
{ return  (zoom) ? T1y (logic_y) : T2y (logic_y); }

Point  logic_coord (win_point* coord)
{ Point p;
  p.x = ((coord->x - MARGIN) / (( 1.0 / 2)*(WindowSize ()->x - 2 * MARGIN))) - 1;
  p.y = ((coord->y - MARGIN) / ((-1.0 / 2)*(WindowSize ()->y - 2 * MARGIN))) + 1;
  return p;
}
//-------------------------------------------------------------------------------
/* Create a string with last error message */
tstring  GetLastErrorToString ()
{
  DWORD  error = GetLastError ();
  if ( error )
  {
    LPVOID lpMsgBuf;
    DWORD bufLen = FormatMessageW ( FORMAT_MESSAGE_ALLOCATE_BUFFER |
                                    // FORMAT_MESSAGE_IGNORE_INSERTS |
                                    FORMAT_MESSAGE_FROM_SYSTEM,
                                    NULL,
                                    error,
                                    MAKELANGID (LANG_NEUTRAL, SUBLANG_DEFAULT),
                                    (LPTSTR) &lpMsgBuf,
                                    0, NULL);
    if ( bufLen )
    {
      LPCSTR lpMsgStr = (LPCSTR) lpMsgBuf;
      tstring result (lpMsgStr, lpMsgStr + bufLen);

      LocalFree (lpMsgBuf);
      return result;
    }
  }
  return  tstring ();
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
  hLabCanv = CreateWindow (_T ("STATIC"),            /* The name of the static control's class */
                           _T ("Canvas  "),                                    /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT | WS_BORDER, /* Styles (continued) */
                           (int) lp.LabelsLeft,                              /* X co-ordinates */
                           (int) lp.LabCanvTop,                              /* Y co-ordinates */
                           (int) lp.LabelsWidth,                                      /* Width */
                           (int) lp.LabCanvHeight,                                   /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_CANVAS,                               /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabCanv )
    MessageBox (hWnd, _T ("Could not create hLabCanv."), _T ("Error"), MB_OK | MB_ICONERROR);

  // Create a Static Label control
  hLabHelp = CreateWindow (_T ("STATIC"),            /* The name of the static control's class */
                           _T ("Help"),                                        /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT | WS_BORDER, /* Styles (continued) */
                           (int) lp.LabelsLeft,                              /* X co-ordinates */
                           (int) lp.LabHelpTop,                              /* Y co-ordinates */
                           (int) lp.LabelsWidth,                                      /* Width */
                           (int) lp.LabHelpHeight,                                   /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_HELP,                                 /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabHelp )
    MessageBox (hWnd, _T ("Could not create hLabHelp."), _T ("Error"), MB_OK | MB_ICONERROR);

  // Create a Static Label control
  hLabMAim = CreateWindow (_T ("STATIC"),            /* The name of the static control's class */
                           _T (" "),                                           /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT | WS_BORDER, /* Styles (continued) */
                           (int) lp.LabelsLeft,                              /* X co-ordinates */
                           (int) lp.LabTestTop,                              /* Y co-ordinates */
                           (int) lp.LabelsWidth,                                      /* Width */
                           (int) lp.LabTestHeight,                                   /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_MAIM,                                 /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabMAim )
    MessageBox (hWnd, _T ("Could not create hLabMAim."), _T ("Error"), MB_OK | MB_ICONERROR);
  
  // Create a Static Label control
  hLabTest = CreateWindow (_T ("STATIC"),            /* The name of the static control's class */
                           _T (" "),                                           /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT | WS_BORDER, /* Styles (continued) */
                           (int) lp.LabelsLeft,                              /* X co-ordinates */
                           (int) lp.LabStatTop,                              /* Y co-ordinates */
                           (int) lp.LabelsWidth,                                      /* Width */
                           (int) lp.LabStatHeight,                                   /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_TEST,                                 /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabTest )
    MessageBox (hWnd, _T ("Could not create hLabTest."), _T ("Error"), MB_OK | MB_ICONERROR);
  
  // Create a Static Label control
  hLabStat = CreateWindow (_T ("STATIC"),            /* The name of the static control's class */
                           _T (" "),                                           /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_RIGHT | WS_BORDER, /* Styles (continued) */
                           (int) lp.LabelsLeft,                              /* X co-ordinates */
                           (int) lp.LabCanvTop,                              /* Y co-ordinates */
                           (int) lp.LabelsWidth,                                      /* Width */
                           (int) lp.LabCanvHeight,                                   /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_STAT,                                 /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabStat )
    MessageBox (hWnd, _T ("Could not create hLabStat."), _T ("Error"), MB_OK | MB_ICONERROR);

  // Generate the help string
  { TCHAR string_help[1024];

  _stprintf (string_help,
             _T ("Клавиши управления:  \rEsc - выход  \rR - сбросить всё  \r\r") // Enter - авто-тест  \r
             _T ("Z - двинуть ключицей вправо  \rX - сомкнуть плечо  \rС - сомкнуть локоть  \rV - сомкнуть ладонь  \r")
             _T ("A - двинуть ключицей влево  \rS - раскрыть плечо  \rD - раскрыть локоть  \rF - раскрыть ладонь  \r\r")
             _T ("Повторное нажатие на кнопку во время движения  \rостанавливает соответствующее движение.  \r\r")
             _T ("U - нарисовать рабочую область руки  \rO - нарисовать случайную траекторию  \r\r")
             _T ("P - Cover Test  \rT - Random Test  \r")
             _T ("Y - TargetCoverTest  \r\rG - Show scales  \r\r")
             _T ("Ctrl+O - OpenFile  \rCtrl+S - SaveFile  \r\r")
             //_T ("Квадрат цели 10x10 точек  \r\rДля выбора цели отрисовки  \r")
             //_T ("M + !no!/%2u + Enter,  \rN + !no!/%2u + Enter   \r, где 0 <= !no! - номер строки/столбца"),
             );

  // Setting the Label's text
  SendMessage (hLabHelp,         /* Label   */
               WM_SETTEXT,       /* Message */
               (WPARAM) NULL,    /* Unused  */
               (LPARAM) string_help);
  }

  // RegisterHotKey (NULL /* hWnd */, HK_OPEN, MOD_CONTROL | MOD_NOREPEAT, 0x4f); // 'O'
  // RegisterHotKey (NULL /* hWnd */, HK_SAVE, MOD_CONTROL | MOD_NOREPEAT, 0x53); // 'S'
  // RegisterHotKey (NULL /* hWnd */, HK_EXIT,               MOD_NOREPEAT, 0x1B); // 'Esc'       

  // SetTimer (hWnd,                   /* Handle to main window */
  //           IDT_TIMER_STROKE,       /* Timer identifier      */
  //           USER_TIMER_MINIMUM,     /* 1/200-second interval */
  //           (TIMERPROC) NULL);      /* No Timer callback     */

  SetTimer (hWnd,                   /* Handle to main window */
            IDT_TIMER_VISION,       /* Timer identifier      */
            30,                     /* 1/10-second interval  */
            (TIMERPROC) NULL);      /* No Timer callback     */
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
                (int) lp.LabelsLeft,     /* X co-ordinates */
                (int) lp.LabCanvTop,     /* Y co-ordinates */
                (int) lp.LabelsWidth,             /* Width */
                (int) lp.LabCanvHeight,          /* Height */
                (UINT) NULL);   
  /* Set position help label */ 
  SetWindowPos (hLabHelp, NULL, 
                (int) lp.LabelsLeft,     /* X co-ordinates */
                (int) lp.LabHelpTop,     /* Y co-ordinates */
                (int) lp.LabelsWidth,             /* Width */
                (int) lp.LabHelpHeight,          /* Height */
                (UINT) NULL);   
  /* Set position help label */ 
  SetWindowPos (hLabMAim, NULL, 
                (int) lp.LabelsLeft,     /* X co-ordinates */
                (int) lp.LabMAimTop,     /* Y co-ordinates */
                (int) lp.LabelsWidth,             /* Width */
                (int) lp.LabMAimHeight,          /* Height */
                (UINT) NULL);
  /* Set position help label */
  SetWindowPos (hLabTest, NULL, 
                (int) lp.LabelsLeft,     /* X co-ordinates */
                (int) lp.LabTestTop,     /* Y co-ordinates */
                (int) lp.LabelsWidth,             /* Width */
                (int) lp.LabTestHeight,          /* Height */
                (UINT) NULL);   
  /* Set position help label */ 
  SetWindowPos (hLabStat, NULL, 
                (int) lp.LabelsLeft,     /* X co-ordinates */
                (int) lp.LabStatTop,     /* Y co-ordinates */
                (int) lp.LabelsWidth,             /* Width */
                (int) lp.LabStatHeight,          /* Height */
                (UINT) NULL);   
}

void OnWindowPaint (HWND &hWnd, RECT &myRect,
                    MyWindowData &wd)
{
  PAINTSTRUCT ps = {};
  
  HDC  hdc = BeginPaint (hWnd, &ps);
  //-------------------------------------------
  if ( wd.hStaticBitmapChanged && !wd.testing )
  {
    /* Создание ещё одного теневого контекста
     * для отрисовки неизменной и
     * (в некоторых случаях ресурсоёмкой)
     * части картинки единожды.
     */
    if ( !wd.hStaticDC )
    {
      wd.hStaticDC = CreateCompatibleDC (hdc);
      if ( !wd.hStaticDC )
      {
        MessageBox (hWnd, GetLastErrorToString ().c_str (),
                    _T ("ERROR"), MB_OK | MB_ICONERROR);
      }
    }

    /* Удаляем старый объект */
    if ( wd.hStaticBitmap )
    {
      DeleteObject (wd.hStaticBitmap);
      wd.hStaticBitmap = NULL;
    }
    /* Создаём новый растровый холст */
    wd.hStaticBitmap = CreateCompatibleBitmap (hdc, myRect.right - myRect.left,
                                               myRect.bottom - myRect.top);
    if ( !wd.hStaticBitmap )
    {
      MessageBox (hWnd, GetLastErrorToString ().c_str (),
                  _T ("ERROR"), MB_OK | MB_ICONERROR);
    }
    SelectObject (wd.hStaticDC, wd.hStaticBitmap);

    /* Рисуем всё заново */
    //======================================
    /* Закраска фона рабочей области */
    FillRect (wd.hStaticDC, &myRect, wd.hBrush_back);
    /* set transparent brush to fill shapes */
    SelectObject (wd.hStaticDC, wd.hBrush_null);
    SetBkMode    (wd.hStaticDC, TRANSPARENT);
    //-------------------------------------
    DrawDecardsCoordinates (wd.hStaticDC);
    wd.target.draw (wd.hStaticDC, wd.hPen_grn, true,
                    false, false);
                    // true, false);

    //-------------------------------------
    /* Здесь рисуем на контексте hCmpDC */
    OnPaintStaticFigures (wd.hStaticDC, wd);
    wd.hStaticBitmapChanged = false;
    //======================================
  }
  //-------------------------------------
  /* Создание теневого контекста для двойной буфферизации */
  HDC   hCmpDC = CreateCompatibleDC (hdc);
  if ( !hCmpDC )
  { MessageBox (hWnd, GetLastErrorToString ().c_str (),
                _T ("ERROR"), MB_OK | MB_ICONERROR);
  }

  HBITMAP  hBmp = CreateCompatibleBitmap (hdc, myRect.right  - myRect.left,
                                               myRect.bottom - myRect.top);
  if ( !hBmp )
  { MessageBox (hWnd, GetLastErrorToString ().c_str (),
                _T ("ERROR"), MB_OK | MB_ICONERROR);
  }
  SelectObject (hCmpDC, hBmp);
  //-------------------------------------
  if ( !wd.testing )
  {
    SetStretchBltMode (hCmpDC, COLORONCOLOR);
    BitBlt (hCmpDC, 0, 0,
            myRect.right - myRect.left,
            myRect.bottom - myRect.top,
            wd.hStaticDC, 0, 0,
            SRCCOPY);
    //-------------------------------------
    /* set transparent brush to fill shapes */
    SelectObject (hCmpDC, wd.hBrush_null);
    SetBkMode (hCmpDC, TRANSPARENT);
  }
  else
  {
    /* Рисуем всё заново */
    //======================================
    /* Закраска фона рабочей области */
    FillRect (hCmpDC, &myRect, wd.hBrush_back);
    /* set transparent brush to fill shapes */
    SelectObject (hCmpDC, wd.hBrush_null);
    SetBkMode (hCmpDC, TRANSPARENT);
    //-------------------------------------
    DrawDecardsCoordinates (hCmpDC);
    wd.target.draw (hCmpDC, wd.hPen_grn,
                    true /*false*/, false, false);
    //-------------------------------------
  }
  //-------------------------------------
  /* Здесь рисуем на контексте hCmpDC */
  OnPainDynamicFigures (hCmpDC, wd);
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

    case 'q':
    {
      //========================================
      // OnShowDBPoints (wd);
      wd.allPointsDB_show = !wd.allPointsDB_show;
      wd.hStaticBitmapChanged = true;
      //========================================
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    }

    case 'o':
    { 
      //========================================
      wd.trajectory_frames.clear ();
      wd.hand.SET_DEFAULT;

      // wd.trajectory_frames_muscle = selectHandMove (random (HandMovesCount));
      wd.trajectory_frames_muscle = wd.hand.selectControl ();
      wd.trajectory_frames_lasts = random (1U, wd.hand.maxMuscleLast (wd.trajectory_frames_muscle));

#ifndef   _ANIMATION_
      wd.hand.move (wd.trajectory_frames_muscle,
                    wd.trajectory_frames_lasts,
                    wd.trajectory_frames);

      auto   hand_pos = wd.hand.position;
      Point  hand_base = wd.trajectory_frames.front ();
      // wd.trajectory_frames.pop_front ();
      auto rec = Record (hand_pos, hand_base, hand_pos,
                         { wd.trajectory_frames_muscle },
                         { 0U }, { wd.trajectory_frames_lasts },
                         1U, wd.trajectory_frames);
      wd.store.insert (rec);

      wd.trajectory_frames.clear ();
      wd.trajectory_frames_muscle = Hand::EmptyMov;
      wd.trajectory_frames_lasts = 0;
#else  // _ANIMATION_
      wd.trajectory_frames_show = true;
      wd.hand.step (wd.trajectory_frames_muscle,
                    wd.trajectory_frames_lasts);
#endif // _ANIMATION_

      //========================================
      InvalidateRect (hWnd, &myRect, TRUE);
      break;
    }
    
    case 't':
    { //========================================
      OnRandomTest (wd);
      // MessageBox (hWnd, _T ("Done\n"), 
      //             _T ("RandomTest"), MB_OK);
      //========================================
      /* Фон не будет переписовываться */
      InvalidateRect (hWnd, &myRect, TRUE);
      break;
    }

    case 'p':
    {
      //========================================
      OnCoverTest (wd);

      // tstring message;
      // for ( auto &t : wd.testing_trajectories )
      //   message += tstring (t.back ()) + tstring (_T ("\n"));
      // MessageBox (hWnd, message.c_str (),
      //             str (boost::wformat (_T ("CoverTest %1%")) %
      //             wd.testing_trajectories.size ()).c_str (),
      //             MB_OK);
      //========================================
      InvalidateRect (hWnd, &myRect, TRUE);
      break;
    }

    case 'y':
    {
      //========================================
      // WorkerThreadRunStoreTask (wd, _T (" *** target cover test ***  "),
      //                           Positions::testCoverTarget, wd.hand, wd.target);
      // Positions::testCoverTarget (wd.store, wd.hand, wd.target, wd.testing_trajectories);
      WorkerThreadRunStoreTask (wd, _T (" *** stage 1 test ***  "),
                                // Positions::test_1_stage
                                [](Store &store, Hand &hand, RecTarget &target)
                                {
                                  Positions::LearnMovements lm;
                                  lm.testStage1 (store, hand, target);
                                }
                                , wd.hand, wd.target);
      WorkerThreadTryJoin (wd);
      //========================================
      InvalidateRect (hWnd, &myRect, TRUE);
      break;
    }

    case 'k':
    {
      // static std::list<int> step;
      // // step.clear ();
      // std::shared_ptr<trajectory_t> trajectory = make_shared<trajectory_t> (new trajectory_t());
      // wd.testing_trajectories.push_back (trajectory);

      // HandMoves::testCover (wd.store, wd.hand, Hand::EmptyMov, wd.testing_trajectories); // , step, wd.testing_trajectories.back ());
      
      Point center = Point (((wd.target.min) ().x + (wd.target.max) ().x) / 2.,
                            ((wd.target.min) ().y + (wd.target.max) ().y) / 2.);
      // Positions::getTargetCenter (wd.hand, center);

      WorkerThreadRunStoreTask (wd, _T (" *** stage 2 test ***  "),
                                [] (Store &store, Hand &hand, RecTarget &target)
                                { 
                                  Positions::LearnMovements lm;
                                  lm.testStage2 (store, hand, target);
                                }
                                , wd.hand, wd.target);
      WorkerThreadTryJoin (wd);
      
      // lm.testStage3 (wd.store, wd.hand, wd.target, wd.uncoveredPoints);
      //========================================
      InvalidateRect (hWnd, &myRect, TRUE);
      break;
    }

    case 'j':
    {
      Positions::LearnMovements lm;
      lm.testStage3 (wd.store, wd.hand, wd.target, wd.uncoveredPoints);
      //========================================
      InvalidateRect (hWnd, &myRect, TRUE);
      break;
    }

    case 'n':
    {
      // Positions::LearnMovements lm;
      // lm.STAGE2 (wd.store, wd.hand, wd.target);

      WorkerThreadRunStoreTask (wd, _T (" *** stage 2 test ***  "),
                                [](Store &store, Hand &hand, RecTarget &target)
                                {
                                  Positions::LearnMovements lm;
                                  lm.STAGE2 (store, hand, target);
                                }
                                , wd.hand, wd.target);
      WorkerThreadTryJoin (wd);
      //========================================
      InvalidateRect (hWnd, &myRect, TRUE);
      break;
    }

    case 'i':
    { // int muscle, last;
      // cin >> muscle >> last;
      // wd.hand.move ((Hand::MusclesEnum)muscle, (Hand::time_t)last);
      
      // testLittleCorrectives (wd.store, wd.hand, wd.target,
      //                        max(wd.target.x_distance,
      //                            wd.target.y_distance));

      littleTest (wd/*, 2. * max (wd.target.x_distance,
                                wd.target.y_distance)*/);
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    }

    case 'u':
    {
      if ( !wd.working_space.size () )
      {
        Hand::MusclesEnum muscle;
        wd.hand.set (Hand::Clvcl | Hand::Shldr | Hand::Elbow, { 0.,0.,0. });
    
        muscle = Hand::ClvclOpn;
        wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.working_space);
    
        muscle = Hand::ShldrCls;
        wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.working_space);
    
        muscle = Hand::ClvclCls;
        wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.working_space);
    
        muscle = Hand::ElbowCls;
        wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.working_space);
    
        muscle = Hand::ShldrOpn;
        wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.working_space);
    
        muscle = Hand::ElbowOpn;
        wd.hand.move (muscle, wd.hand.maxMuscleLast (muscle), wd.working_space);
    
        wd.hand.SET_DEFAULT;
      }
      wd.working_space_show = !wd.working_space_show;
      wd.hStaticBitmapChanged = true;
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    }

    case 'l':
    {
      if ( wd.pWorkerThread )
      {
        wd.pWorkerThread->interrupt ();
        WorkerThreadTryJoin (wd);
      }
      break;
    }

    case 'g':
    {
      //========================================
      wd.scaleLetters.show = !wd.scaleLetters.show;
      //========================================
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    } 

    case 'b':
    {
      //========================================
      zoom = !zoom;
      //========================================
      wd.hStaticBitmapChanged = true;
      InvalidateRect (hWnd, &myRect, FALSE);
      break;
    }

    case 'h':
    {
      //========================================
      wd.uncovered_show = !wd.uncovered_show;
      wd.hStaticBitmapChanged = true;
      //========================================
      InvalidateRect (hWnd, &myRect, TRUE);
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
    /* Wrist */
    case 'v': wd.hand.step (Hand::WristCls); break;
    case 'f': wd.hand.step (Hand::WristOpn); break;
    /* Reset */
    case 'r': 
    {
      wd.hand.SET_DEFAULT;

      wd.reach = false;

      wd.adjPointsDB.clear ();
      wd.trajectoriesDB.clear ();

      wd.mouse_haved = false;

      wd.trajectory_frames.clear ();
      wd.trajectory_frames_show = false;
      wd.testing_trajectories.clear ();

      wd.uncoveredPoints.clear ();

      /* Setting the Label's text */
      SendMessage (wd.hLabMAim,      /* Label   */
                   WM_SETTEXT,       /* Message */
                   (WPARAM) NULL,    /* Unused  */
                   (LPARAM) _T (" "));
      break;
    }
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
tstring   OpenFileDialog (HWND hWnd)
{
  /* common dialog box structure */
  OPENFILENAME  OpenFileName = {};
  TCHAR         szFilePath[MAX_PATH];  /* buffer for file name */
  TCHAR         szFileName[MAX_PATH];

  /* Initialize OpenFileName */
  OpenFileName.lStructSize = sizeof (OPENFILENAME);
  OpenFileName.hwndOwner = hWnd;

  OpenFileName.lpstrFileTitle = szFileName;
  OpenFileName.nMaxFileTitle = sizeof (szFileName);
  OpenFileName.lpstrFile = szFilePath;
  OpenFileName.nMaxFile = sizeof (szFilePath);

  /*  GetOpenFileName does not use the
  *  contents to initialize itself. 
  */
  OpenFileName.lpstrFile[0] = '\0';
  OpenFileName.lpstrFileTitle[0] = '\0';

  OpenFileName.lpstrDefExt = _T ("bin");
  OpenFileName.lpstrFilter = _T ("Binary\0*.bin\0All\0*.*\0");
  OpenFileName.nFilterIndex = 1;
  OpenFileName.lpstrInitialDir = NULL;
  OpenFileName.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_EXPLORER;

  // Display the Open dialog box. 
  if ( !GetOpenFileName (&OpenFileName) )
  { return  _T (""); }
  return  OpenFileName.lpstrFile;
}
tstring   SaveFileDialog (HWND hWnd)
{
  OPENFILENAME  SaveFileName = {};
  TCHAR         szFileName[MAX_PATH] = {};
  TCHAR         szFilePath[MAX_PATH] = {};

  SaveFileName.lStructSize = sizeof (OPENFILENAME);
  SaveFileName.hwndOwner = hWnd;
  
  SaveFileName.lpstrFile = szFilePath;
  SaveFileName.nMaxFile = sizeof (szFilePath);
  SaveFileName.lpstrFileTitle = szFileName;
  SaveFileName.nMaxFileTitle = sizeof (szFileName);

  SaveFileName.lpstrFile[0] = '\0';
  SaveFileName.lpstrFileTitle[0] = '\0';

  SaveFileName.lpstrDefExt = _T ("bin");
  SaveFileName.lpstrFilter = _T ("Binary\0*.bin\0All\0*.*\0");
  SaveFileName.nFilterIndex = 1;
  SaveFileName.lpstrInitialDir = NULL;

  SaveFileName.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_EXPLORER;

  // Display the Open dialog box. 
  if ( GetSaveFileName (&SaveFileName) )
  { return _T(""); }
  return SaveFileName.lpstrFile;
}
//-------------------------------------------------------------------------------
tstring  GetTextToString (HWND hWnd)
{
  int  len = GetWindowTextLength (hWnd) + 1U;
  std::vector<TCHAR>  buffer (len);
  GetWindowText (hWnd, buffer.data (), len);
  // SendMessage (hWnd, WM_GETTEXT, (WPARAM) len, (LPARAM) buffer.data ());
  return  tstring (buffer.begin (), buffer.end () - 1);
}
//-------------------------------------------------------------------------------
tstring   CurrentTimeToString (tstring format, std::time_t *the_time)
{
  std::time_t rawtime;
  if ( !the_time )
  { rawtime = std::time (nullptr); }
  else
  { rawtime = *the_time; }
  struct tm  *TimeInfo = std::localtime (&rawtime);

  tstringstream ss;
  ss << std::put_time (TimeInfo, format.c_str ());
  return ss.str ();
}
//-------------------------------------------------------------------------------
void      MakeGradient (color_interval_t  colors, size_t n_levels,
                        gradient_t  &gradient)
{ 
  gradient.clear ();
  gradient.resize (n_levels);

  COLORREF cf = colors.first, cs = colors.second;
  /* loop to create the gradient */
  for ( size_t i = 0U; i < n_levels; ++i )
  {
    unsigned char  r, g, b;
    /* Determine the colors */
    r = static_cast<unsigned char> (GetRValue (cf) + (i * (GetRValue (cs) - GetRValue (cf)) / n_levels));
    g = static_cast<unsigned char> (GetGValue (cf) + (i * (GetGValue (cs) - GetGValue (cf)) / n_levels));
    b = static_cast<unsigned char> (GetBValue (cf) + (i * (GetBValue (cs) - GetBValue (cf)) / n_levels));
    /* Append new color */
    gradient[i] = RGB (r, g, b);
  }
}
//-------------------------------------------------------------------------------
void      OnEraseBackGround_WithGradient (HWND hwnd)
{
  /* Vars */
  HDC dc; /* Standard Device Context; used to do the painting */

          /* rect = Client Rect of the window;
          Temp = Temparary rect tangle for the color bands */
  RECT rect, temp;
  HBRUSH color; /* A brush to do the painting with */

                /* Get the dc for the wnd */
  dc = GetDC (hwnd);

  /* Get the client rect */
  GetClientRect (hwnd, &rect);

  /* Start color; Change the R,G,B values
  to the color of your choice */
  int r1 = 255, g1 = 0, b1 = 0;

  /* End Color; Change the R,G,B values
  to the color of your choice */
  int r2 = 255, g2 = 255, b2 = 0;

  /* loop to create the gradient */
  for ( int i = 0; i<rect.right; i++ )
  {
    /* Color ref. for the gradient */
    int r, g, b;
    /* Determine the colors */
    r = r1 + (i * (r2 - r1) / rect.right);
    g = g1 + (i * (g2 - g1) / rect.right);
    b = b1 + (i * (b2 - b1) / rect.right);

    /* Fill in the rectangle information */

    /* The uper left point of the rectangle
    being painted; uses i as the starting point*/
    temp.left = i;
    /* Upeer Y cord. Always start at the top */
    temp.top = 0;
    /* Okay heres the key part,
    create a rectangle thats 1 pixel wide */
    temp.right = i + 1;
    /* Height of the rectangle */
    temp.bottom = rect.bottom;

    /* Create a brush to draw with;
    these colors are randomized */
    color = CreateSolidBrush (RGB (r, g, b));

    /* Finally fill in the rectange */
    FillRect (dc, &temp, color);
  }
}
//-------------------------------------------------------------------------------
