#include "StdAfx.h"
#include "MyWindow.h"
#include "Hand.h"
#include "Draw.h"
#include "target.h"
#include "HandMovesStore.h"

#define  WM_USER_TIMER  WM_USER+3
//------------------------------------------------------------------------------
win_point*  WindowSize (void)
{ static win_point Width_Height;
  return &Width_Height;
}
void     SetWindowSize (int Width_, int Height_)
{ WindowSize ()->x = Width_;
  WindowSize ()->y = Height_;
}

int  Tx (double logic_x) { return  (int) (MARGIN + ( 1.0 / 2)*(logic_x + 1)*(WindowSize ()->x - 2 * MARGIN)); }
int  Ty (double logic_y) { return  (int) (MARGIN + (-1.0 / 2)*(logic_y - 1)*(WindowSize ()->y - 2 * MARGIN)); }

Point  logic_coord (win_point* coord)
{
  Point p;
  p.x = ((coord->x - MARGIN) / ((1.0 / 2)*(WindowSize ()->x - 2 * MARGIN))) - 1;
  p.y = ((coord->y - MARGIN) / ((-1.0 / 2)*(WindowSize ()->y - 2 * MARGIN))) + 1;
  return p;
}
//------------------------------------------------------------------------------
LRESULT CALLBACK  WndProc (HWND hWnd, UINT messg, WPARAM wParam, LPARAM lParam)
{
  // static uint_t        time;
  // static DWORD    colors[3];
  // static uchar_t    flag[3];
  // static uchar_t    flag_timer;  // старт пользовательского таймера

  static win_point      coord;      // координаты мыши в пикселях
  static HWND        hLabHelp;
  static HWND        hLabCanv;      // Canvas
  static RECT        myRect;

  //------------------------------------------------------------------------------
  // const uint_t  tgRowsCount = 12U;
  // const uint_t  tgColsCount = 15U;

  // static RecTarget  T (tgColsCount, tgRowsCount, -0.39, 0.62, -0.01, -0.99);

  const uint_t  tgRowsCount = 35U;
  const uint_t  tgColsCount = 30U;
  const uchar_t jointPercent[] = { 0U, 0U, 100U };

  static RecTarget  T (tgColsCount, tgRowsCount, -0.70, 0.90, 0.90, -0.99);
  static Hand       hand;

  static HandMoves::Store store; // (&T)

  static bool print_trajectories;
  static std::list< std::list<Point> > trajectories;

  static std::list<Point> last_trajectory;
  static ulong_t last;
  static Hand::MusclesEnum mv;

  static int   fm, fn;
  static int  p_x, p_y;
  //------------------------------------------------------------------------------
  switch ( messg )
  {
    case WM_CREATE:
    {
      { RECT Rect;

        GetClientRect (hWnd, &Rect);
        myRect.left   = Rect.left;
        myRect.top    = Rect.top;
        myRect.bottom = Rect.bottom;
        myRect.right  = Rect.left + (Rect.bottom - Rect.top);
        
        print_trajectories = false;

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
                   _T ("M + !no!/%2u + Enter,  \rN + !no!/%2u + Enter   \r, где 0 <= !no! - номер строки/столбца"),
                   tgRowsCount,
                   tgColsCount);
        
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
      //=======================
      p_x = 0; p_y = 0; fm = 0; fn = 0;
      random_input (0U);

      // Hm.gain ();
      hand.set (jointPercent);
      //=======================
      break;
    }

    case WM_PAINT:
    { PAINTSTRUCT ps;
      HBITMAP     hBmp;
      HDC         hdc, hCmpDC;
      HPEN        hPen_red, hPen_grn;
      
      hdc = BeginPaint (hWnd, &ps);
      //-------------------------------------------
      // Создание теневого контекста для двойной буфферизации
      hCmpDC = CreateCompatibleDC (hdc);
      hBmp = CreateCompatibleBitmap (hdc, myRect.right - myRect.left, myRect.bottom - myRect.top);
      SelectObject (hCmpDC, hBmp);
      
      // Закраска фона рабочей области
      { HBRUSH brush = CreateSolidBrush (RGB (235, 235, 255) /* RGB (255,204,238) background color */);
        FillRect (hCmpDC, &myRect, brush);
        DeleteObject (brush);
      }
      
      // set transparent brush to fill shapes
      // SetBkMode (hCmpDC, TRANSPARENT);
      SelectObject (hCmpDC, (HBRUSH) GetStockObject (NULL_BRUSH));
      
      // создаем ручку
      hPen_grn = CreatePen (PS_SOLID, 1, RGB (100, 180, 50));
      hPen_red = CreatePen (PS_SOLID, 2, RGB (255, 000, 00));
      //-------------------------------------
      // Здесь рисуем на контексте hCmpDC
      draw_decards_coordinates (hCmpDC);
      
      // Отрисовка фигуры
      T.draw (hCmpDC, hPen_grn);
      hand.draw (hCmpDC, hPen_red);

      draw_trajectory (hCmpDC, last_trajectory, hPen_red /* hPen_blu */);

      if ( print_trajectories )
      {
        int i = 0;
        for ( auto trajectory : trajectories )
        {
          HPEN hPen_blu = CreatePen (PS_SOLID, 2, RGB (000, 000, 255));
          draw_trajectory (hCmpDC, trajectory, hPen_blu);
          DeleteObject (hPen_blu);
        }
      }
      //-------------------------------------
      // Копируем изображение из теневого контекста на экран
      SetStretchBltMode (hdc, COLORONCOLOR);
      BitBlt (hdc, 0, 0,
              myRect.right  - myRect.left,
              myRect.bottom - myRect.top,
              hCmpDC, 0, 0,
              SRCCOPY );

      // очищаем ручку
      DeleteObject (hPen_grn);
      DeleteObject (hPen_red);
      // Удаляем ненужные системные объекты
      DeleteDC (hCmpDC);
      DeleteObject (hBmp);
      hCmpDC = NULL;
      //---------------------------------------------
      EndPaint (hWnd, &ps);
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
      //=======================
      //Hm.Dump ();
      //=======================
      PostQuitMessage (0);
      break;

    case WM_MOVE:
    case WM_SIZE:
    { RECT Rect;

      GetClientRect (hWnd, &Rect);
      myRect.left   = Rect.left;
      myRect.top    = Rect.top;
      myRect.bottom = Rect.bottom;
      myRect.right  = Rect.left + (Rect.bottom - Rect.top);
      
      SetWindowSize ( /* Rect.right - Rect.left */ Rect.bottom - Rect.top, Rect.bottom - Rect.top);
      // Set position help label
      SetWindowPos (hLabHelp, NULL,
                    Rect.bottom - Rect.top,                   /* X co-ordinates */
                    Rect.top + 25,                            /* Y co-ordinates */
                    Rect.right - (Rect.bottom - Rect.top),    /* Width  */
                    Rect.bottom - Rect.top - 25,              /* Height */
                    (UINT) NULL);
      // Set position noCanvas label
      SetWindowPos (hLabCanv, NULL,
                    Rect.bottom - Rect.top,                   /* X co-ordinates */
                    Rect.top,                                 /* Y co-ordinates */
                    Rect.right - (Rect.bottom - Rect.top),            /*  Width */
                    25,                                               /* Height */
                    (UINT) NULL);
      break;
    }

    case WM_GETMINMAXINFO:
    { ((MINMAXINFO*) lParam)->ptMinTrackSize.x = WIDTH;
      ((MINMAXINFO*) lParam)->ptMinTrackSize.y = HIGHT;
      break;
    }

    case WM_USER_TIMER:
      KillTimer (hWnd, IDT_TIMER);
      break;

    case WM_TIMER:
    { //=======================
      last--;
      if ( !last )
      {
        hand.step (mv);
      }

      hand.step ();
      last_trajectory.push_back (hand.position);

      InvalidateRect (hWnd, &myRect, TRUE);
      //=======================
      break;
    }

    case WM_HOTKEY:
    { switch ( wParam )
      { 
        case HK_EXIT:
          SendMessage (hWnd, WM_QUIT, 0, 0);
          break;
      }
      break;
    }

    case WM_CHAR:
    {
      bool v[Hand::musclesCount] = { 0 };
      switch ( wParam )
      {
        case 0x08: // Process a backspace. 
          break;

        case 0x09:  // Process a tab.
          break;

        case 0x0D: // Process a carriage return
          fm = 0; fn = 0;
          // Фон не будет переписовываться
          InvalidateRect (hWnd, &myRect, TRUE /*0*/);
          break;

        case 't':
        { //========================================
          uchar_t jointPercent[] = { 50U, 50U, 50U };
          hand.set  (jointPercent);

          //test_random (Hm, H, 100U); // 10001000100

          InvalidateRect (hWnd, &myRect, FALSE);
          Sleep (10);
          //========================================
          MessageBoxW (hWnd, _T("Done"), _T(""), MB_OK);
          break;
        }

        case 'o':
        {
          // hand.reset ();
          uchar_t jointPercent[] = { 50U, 50U, 50U };
          hand.set (jointPercent);
          last_trajectory.clear ();
          
          // mv   = HandMoves::switch_move (random_input (26UL));
          // last = random_input (hand.maxElbowMoveFrames, hand.minJStopMoveFrames);

          //hand.move ()
          hand.step (mv);
          last_trajectory.push_back (hand.position);
        }

        case 'p':
        {
          // hand.reset ();
          // uchar_t jointPercent[] = { 50U, 50U, 50U };
          // hand.set (jointPercent);
          
          trajectories.clear ();

          HandMoves::test_cover (store, hand, 0., trajectories, 1);
          print_trajectories = true;
          //hand.move ()
          //hand.step (mv);
          //last_trajectory.push_back (hand.position);
        }

        //=====================
        // clavicle
        case 'z':  // symbol
          v[0] = 1;
          hand.step (v);
          break;

        case 'a':  // symbol
          v[1] = 1;
          hand.step (v);
          break;

          // sholder
        case 'x':  // symbol
          v[2] = 1;
          hand.step (v);
          break;

        case 's':  // symbol
          v[3] = 1;
          hand.step (v);
          break;

          // elbow
        case 'c':  // symbol
          v[4] = 1;
          hand.step (v);
          break;

        case 'd':  // symbol
          v[5] = 1;
          hand.step (v);
          break;

          // reset
        case 'r':  // symbol
          hand.reset ();
          break;
        //=====================

        //========================================
        case 'm': if ( !fn ) fm = 1; break;
        case 'n': if ( !fm ) fn = 1; break;

        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9':
             if ( fm ) p_x = p_x * 10U + ((uchar_t) wParam - 0x30);
        else if ( fn ) p_y = p_y * 10U + ((uchar_t) wParam - 0x30);

        if ( p_x >= tgRowsCount ) p_x = tgRowsCount - 1U;
        if ( p_y >= tgColsCount ) p_y = tgColsCount - 1U;
        //========================================
        break;
      }
    }

    default:
      return (DefWindowProc (hWnd, messg, wParam, lParam));
  }

  return (0);
}
