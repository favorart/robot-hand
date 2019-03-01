#include "WindowData.h"
#include "WindowDraw.h"
#include "WindowDrawLetters.h"

#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboLearnMoves.h"
#include "RoboPosApprox.h"
#include "RoboMuscles.h"


using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
/* normal */
LONG Tx_norm (double logic_x)
{ return MARGIN + static_cast<LONG>((logic_x + 1) * (WindowSize ()->x - 2 * MARGIN)) / 2; }
LONG Ty_norm (double logic_y)
{ return MARGIN - static_cast<LONG>((logic_y - 1) * (WindowSize ()->y - 2 * MARGIN)) / 2; }

/* zoom */
LONG Tx_zoom (double logic_x)
{ return MARGIN + static_cast<LONG>((logic_x + 0.5) * (WindowSize ()->x - 2 * MARGIN)); }
LONG Ty_zoom (double logic_y)
{ return MARGIN - static_cast<LONG>((logic_y - 0.0) * (WindowSize ()->y - 2 * MARGIN)); }

Point windowCenter{ 0., 0. };
double currWheelSize = 0.;
/*
*    viewRect
*    *-----------------------------------------------------------------------*
*    |                       ^                                               |
*    |                       | d_up                                          |
*    |        zoomedRect     v                                               |
*    |      *-----------------------------------------*                      |
*    |d_left|                                         |       d_right        |
*    |<---->|                mousePos                 |<-------------------->|
*    |      |                    +                    |                      |
*    |      |                                         |                      |
*    |      |                                         |                      |
*    |      *-----------------------------------------*                      |
*    |                       ^                                               |
*    |                       |                                               |
*    |                       |                                               |
*    |                       | d_down                                        |
*    |                       |                                               |
*    |                       v                                               |
*    *-----------------------------------------------------------------------*
*
*    The origin of rects is the upper left corner.
*/

/* wheel */
LONG Tx_wheel (double logic_x)
{ return MARGIN + static_cast<LONG>((logic_x + (1. - windowCenter.x) ) * (WindowSize()->x - 2 * MARGIN) / (2. - currWheelSize)); }
LONG Ty_wheel (double logic_y)
{ return MARGIN - static_cast<LONG>((logic_y - (1. + windowCenter.y) ) * (WindowSize()->y - 2 * MARGIN) / (2. - currWheelSize)); }

LONG Tx(double logic_x)
{
    switch (MyWindowData::zoom)
    {
    case MyWindowData::Zoom::NONE:
        return Tx_norm(logic_x);
    case MyWindowData::Zoom::STATIC:
        return Tx_zoom(logic_x);
    case MyWindowData::Zoom::WHEEL:
        return Tx_wheel(logic_x);
    default:
        CERROR(_T("Invalid zoom"));
        return 0;
    }
}
LONG Ty(double logic_y)
{
    switch (MyWindowData::zoom)
    {
    case MyWindowData::Zoom::NONE:
        return Ty_norm(logic_y);
    case MyWindowData::Zoom::STATIC:
        return Ty_zoom(logic_y);
    case MyWindowData::Zoom::WHEEL:
        return Ty_wheel(logic_y);
    default:
        CERROR(_T("Invalid zoom"));
        return 0;
    }
}

// наоборот: координаты Windows -> логические координаты
Point LogicCoords (PPOINT coord)
{
  Point p;
  switch (MyWindowData::zoom)
  {
  case MyWindowData::Zoom::NONE:
      p.x = +2. * (coord->x - MARGIN) / (WindowSize()->x - 2 * MARGIN) - 1.;
      p.y = -2. * (coord->y - MARGIN) / (WindowSize()->y - 2 * MARGIN) + 1.;
      break;
  case MyWindowData::Zoom::STATIC:
      p.x = +1. * (coord->x - MARGIN) / (WindowSize()->x - 2 * MARGIN) - 0.5;
      p.y = -1. * (coord->y - MARGIN) / (WindowSize()->y - 2 * MARGIN) + 0.0;
      break;
  case MyWindowData::Zoom::WHEEL:
      p.x = +((2. - currWheelSize) * (coord->x - MARGIN)) / (WindowSize()->x - 2 * MARGIN) - (1. - windowCenter.x);
      p.y = -((2. - currWheelSize) * (coord->y - MARGIN)) / (WindowSize()->y - 2 * MARGIN) + (1. + windowCenter.y);
      break;
  default: CERROR(_T("Invalid zoom"));
  }
  return p;
}
//-------------------------------------------------------------------------------
tstring getJointsHelpString(const Robo::RoboI& robo)
{
    tstringstream ss;
    std::pair<TCHAR, TCHAR> buttons[] = { { 'A', 'Z' },{ 'S', 'X' },{ 'D', 'C' },{ 'F', 'V' } };
    for (joint_t joint = 0; joint < robo.jointsCount(); ++joint)
    {
        tstring m_open, m_close;
        if (typeid(robo) == typeid(Robo::NewHand::Hand))
        {
            auto &hand = dynamic_cast<const Robo::NewHand::Hand&>(robo);
            m_open  = muscleHelp(hand.M(RoboI::muscleByJoint(joint, false)));
            m_close = muscleHelp(hand.M(RoboI::muscleByJoint(joint, true)));
        }
        else if (typeid(robo) == typeid(Robo::Mobile::Tank))
        {
            auto &tank = dynamic_cast<const Robo::Mobile::Tank&>(robo);
            m_open  = muscleHelp(tank.M(RoboI::muscleByJoint(joint, false)));
            m_close = muscleHelp(tank.M(RoboI::muscleByJoint(joint, true)));
        }
        else throw std::exception("Not Implemented");

        ss << _T("  ") << buttons[joint].first  << _T(" - ") << m_open  << _T("  \r")
           << _T("  ") << buttons[joint].second << _T(" - ") << m_close << _T("  \r");
    }
    return ss.str();
}
//-------------------------------------------------------------------------------
void onWindowCreate (HWND hWnd, MyWindowData &wd)
{
  RECT Rect;
  RECT &myRect = wd.canvas.myRect;

  GetClientRect (hWnd, &Rect);
  myRect.left = Rect.left;
  myRect.top = Rect.top;
  myRect.bottom = Rect.bottom;
  myRect.right = Rect.left + (Rect.bottom - Rect.top);

  auto &lp = wd.canvas.lp;

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

  auto &hLabCanv = wd.canvas.hLabCanvas;
  auto &hLabHelp = wd.canvas.hLabHelp;
  auto &hLabMAim = wd.canvas.hLabMAim;
  auto &hLabTest = wd.canvas.hLabTest;
  auto &hLabStat = wd.canvas.hLabStat;

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
  if ( !hLabCanv ) CERROR("Could not create hLabCanv");

  // Create a Static Label control
  hLabHelp = CreateWindow (_T ("STATIC"),            /* The name of the static control's class */
                           _T ("Help"),                                        /* Label's Text */
                           WS_CHILD | WS_VISIBLE | SS_LEFT  | WS_BORDER, /* Styles (continued) */
                           (int) lp.LabelsLeft,                              /* X co-ordinates */
                           (int) lp.LabHelpTop,                              /* Y co-ordinates */
                           (int) lp.LabelsWidth,                                      /* Width */
                           (int) lp.LabHelpHeight,                                   /* Height */
                           hWnd,                                                /* Parent HWND */
                           (HMENU) IDL_HELP,                                 /* The Label's ID */
                           NULL,                              /* The HINSTANCE of your program */
                           NULL);                                /* Parameters for main window */
  if ( !hLabHelp ) CERROR("Could not create hLabHelp");

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
  if ( !hLabMAim ) CERROR("Could not create hLabMAim");
  
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
  if ( !hLabTest ) CERROR("Could not create hLabTest");
  
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
  if ( !hLabStat ) CERROR("Could not create hLabStat");

  // Generate the help string
  tstringstream ss;
  ss << _T ("  Клавиши управления:  \r\r") // "Enter - авто-тест  \r")
     << _T ("  Ctrl+O - OpenFile  |  Ctrl+S - SaveFile  |  Esc - выход  \r\r")
     << _T ("  R - сбросить состояние руки  \r\r")
     << getJointsHelpString(*wd.pRobo)
     << _T ("  Повторное нажатие на кнопку во время движения  \r")
     << _T ("  останавливает соответствующее движение.  \r\r")
     << _T ("  Q - показать все конечные точки в БД  \r")
     << _T ("  W - нарисовать рабочую область руки  \r")
     << _T ("  E - прервать работу алгоритма  \r")
     << _T ("  T - нарисовать случайную траекторию  \r")
     << _T ("  Y - приблизить, показать только мишень  \r")
     << _T ("  U - посчитать непокрытые точки мишени  \r")
     << _T ("  H - показать  непокрытые точки мишени  \r")
     << _T ("  G - показать масштаб и размеры  \r")
     << _T ("  J - сменить градиент точек БД  \r\r")
     << _T ("  I - test approx  \r")
     << _T ("  M - write config  \r")
     << _T ("  N - read config  \r")
     << _T ("  O - Random Test,   P - Cover Test  \r")
     << _T ("  1 - STAGE,  2 - STAGE,  3 - STAGE  \r")
     << _T ("  0 - free storage  \r\r")
     << _T ("  \r\r");

  // Setting the Label's text
  SendMessage (hLabHelp,         /* Label   */
               WM_SETTEXT,       /* Message */
               (WPARAM) NULL,    /* Unused  */
               (LPARAM) ss.str ().c_str ());

  SetTimer (hWnd,                   /* Handle to main window */
            IDT_TIMER_VISION,       /* Timer identifier      */
            30,                     /* 1/10-second interval  */
            (TIMERPROC) NULL);      /* No Timer callback     */
  
  wd.pRobo->reset();

  if (bfs::exists(wd.currFileName))
  {
      WorkerThreadRunTask(wd, _T("  *** loading ***  "),
                          [](MyWindowData &wd, const tstring &filename) {
                              wd.pStore->pick_up(filename, wd.pRobo, Store::Format(wd.store_save_load_format));
                              /// wd.load(filename); TODO:
                          }, std::ref(wd), wd.currFileName);
      WorkerThreadTryJoin(wd);
  }

  SendMessage(hWnd, WM_USER_STORE, NULL, NULL);
}
//------------------------------------------------------------------------------
void onWindowSize (HWND hWnd, MyWindowData &wd)
{
  RECT Rect;
  RECT &myRect = wd.canvas.myRect;

  GetClientRect (hWnd, &Rect);
  myRect.left = Rect.left;
  myRect.top = Rect.top;
  myRect.bottom = Rect.bottom;
  myRect.right = Rect.left + (Rect.bottom - Rect.top);

  setWindowSize ( /* Rect.right - Rect.left */
                 Rect.bottom - Rect.top,
                 Rect.bottom - Rect.top);

  auto &lp = wd.canvas.lp;

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

  auto &hLabCanv = wd.canvas.hLabCanvas;
  auto &hLabHelp = wd.canvas.hLabHelp;
  auto &hLabMAim = wd.canvas.hLabMAim;
  auto &hLabTest = wd.canvas.hLabTest;
  auto &hLabStat = wd.canvas.hLabStat;

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
  
  wd.canvas.hStaticBitmapChanged = true;
  wd.canvas.hDynamicBitmapChanged = true;
}
//------------------------------------------------------------------------------
void onWindowPaint (HWND hWnd, MyWindowData &wd)
{
  if ( !wd.canvas.hStaticBitmapChanged && !wd.canvas.hDynamicBitmapChanged )
      return;

  PAINTSTRUCT ps = {};
  RECT &myRect = wd.canvas.myRect;
  
  HDC  hdc = BeginPaint (hWnd, &ps);
  //-------------------------------------------
  if ( wd.canvas.hStaticBitmapChanged && !wd.testing )
  {
    /*  Создание ещё одного теневого контекста для отрисовки неизменной
     *  и (в некоторых случаях ресурсоёмкой) части картинки единожды.
     */
    if ( !wd.canvas.hStaticDC )
    {
      wd.canvas.hStaticDC = CreateCompatibleDC (hdc);
      if ( !wd.canvas.hStaticDC ) CERROR("!hStaticDC");
    }

    /* Удаляем старый объект */
    if ( wd.canvas.hStaticBitmap )
    {
      DeleteObject (wd.canvas.hStaticBitmap);
      wd.canvas.hStaticBitmap = NULL;
    }
    /* Создаём новый растровый холст */
    wd.canvas.hStaticBitmap = CreateCompatibleBitmap (hdc, myRect.right  - myRect.left,
                                                           myRect.bottom - myRect.top);
    if ( !wd.canvas.hStaticBitmap ) CERROR("!hStaticBitmap");
    SelectObject (wd.canvas.hStaticDC, wd.canvas.hStaticBitmap);

    /* Рисуем всё заново */
    //======================================
    /* Закраска фона рабочей области */
    FillRect (wd.canvas.hStaticDC, &myRect, wd.canvas.hBrush_back);
    /* set transparent brush to fill shapes */
    SelectObject (wd.canvas.hStaticDC, wd.canvas.hBrush_null);
    SetBkMode    (wd.canvas.hStaticDC, TRANSPARENT);
    //-------------------------------------
    onPaintStaticBckGrnd (wd.canvas.hStaticDC, wd);
    //-------------------------------------
    /* Здесь рисуем на контексте hCmpDC */
    onPaintStaticFigures (wd.canvas.hStaticDC, wd);
    wd.canvas.hStaticBitmapChanged = false;
    //======================================
  }
  //-------------------------------------
  /* Создание теневого контекста для двойной буфферизации */
  HDC   hCmpDC = CreateCompatibleDC (hdc);
  if ( !hCmpDC ) CERROR("!hCmpDC");

  HBITMAP hBmp = CreateCompatibleBitmap (hdc, myRect.right  - myRect.left,
                                              myRect.bottom - myRect.top);
  if ( !hBmp ) CERROR("!hBmp");
  SelectObject (hCmpDC, hBmp);
  //-------------------------------------
  if ( !wd.testing )
  {
    SetStretchBltMode (hCmpDC, COLORONCOLOR);
    BitBlt (hCmpDC, 0, 0,
            myRect.right - myRect.left,
            myRect.bottom - myRect.top,
            wd.canvas.hStaticDC, 0, 0,
            SRCCOPY);
    //-------------------------------------
    /* set transparent brush to fill shapes */
    SelectObject (hCmpDC, wd.canvas.hBrush_null);
    SetBkMode (hCmpDC, TRANSPARENT);
  }
  else
  {
    /* Рисуем всё заново */
    //======================================
    /* Закраска фона рабочей области */
    FillRect (hCmpDC, &myRect, wd.canvas.hBrush_back);
    /* set transparent brush to fill shapes */
    SelectObject (hCmpDC, wd.canvas.hBrush_null);
    SetBkMode (hCmpDC, TRANSPARENT);
    //-------------------------------------
    onPaintStaticBckGrnd (hCmpDC, wd);
    //-------------------------------------
  }
  //-------------------------------------
  /* Здесь рисуем на контексте hCmpDC */
  onPainDynamicFigures (hCmpDC, wd);
  wd.canvas.hDynamicBitmapChanged = false;
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
//------------------------------------------------------------------------------
void onWindowTimer(HWND hWnd, MyWindowData &wd, WPARAM wParam)
{
    switch (wParam)
    {
    case IDT_TIMER_STROKE:
    {
        //=======================
        onWindowTimer(wd);
        //=======================
        break;
    }
    case IDT_TIMER_VISION:
    {
        //=======================
        onWindowTimer(wd);
        //=======================
        if (wd.canvas.store_size != wd.pStore->size())
        {
            SendMessage(hWnd, WM_USER_STORE, NULL, NULL);
            if (!wd.testing)
            {
                wd.canvas.hStaticBitmapChanged = true;
                wd.canvas.hDynamicBitmapChanged = true;
            }
        }
        if (wd.testing)
        { WorkerThreadTryJoin(wd); }
        //=======================
        InvalidateRect(hWnd, &wd.canvas.myRect, TRUE);
        break;
    }
    }
}
//------------------------------------------------------------------------------
void onWindowStoreSz(HWND hWnd, MyWindowData &wd)
{
    tstringstream ss;
    ss << _T("Storage size: ") << wd.pStore->size() << _T("  ");
    wd.canvas.store_size = wd.pStore->size();
    /* Setting the Label's text */
    SendMessage(wd.canvas.hLabStat,  /* Label Stat */
                WM_SETTEXT,          /* Message    */
                (WPARAM)NULL,        /* Unused     */
                (LPARAM)ss.str().c_str());
}
//------------------------------------------------------------------------------
void onWindowMouse(HWND hWnd, MyWindowData &wd, WPARAM wParam, LPARAM lParam)
{
    // Если был щелчок левой кнопкой - узнаём координаты
    wd.mouse.coords.x = LOWORD(lParam);
    wd.mouse.coords.y = HIWORD(lParam);
    //=======================
    if (inside(&wd.canvas.myRect, &wd.mouse.coords))
    {
        wd.mouse.click = true;
        wd.mouse.aim = LogicCoords(&wd.mouse.coords);

        auto s = tstring{ wd.mouse.aim } + _T("  ");
        /* Setting the Label's text */
        SendMessage(wd.canvas.hLabMAim, WM_SETTEXT, NULL,
                    reinterpret_cast<WPARAM>(s.c_str()));
        // -------------------------------------------------
        if (!wd.testing)
            makeRoboMove(wd);
    }
    //=======================
    InvalidateRect(hWnd, &wd.canvas.myRect, TRUE);
}
//-------------------------------------------------------------------------------
void onWindowChar(HWND hWnd, MyWindowData &wd, WPARAM wParam, LPARAM lparam)
{
    RECT &myRect = wd.canvas.myRect;
    char symbol = static_cast<char>(wParam);
    //========================================
    switch (symbol)
    {
    case 0x0D: /* enter - invalidate Rect */
    {
        //========================================
        wd.canvas.hDynamicBitmapChanged = true;
        InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }

    case 'q': /* show storage end points */
    {
        //========================================
        // onShowDBPoints (wd);
        wd.canvas.allPointsDBShow = !wd.canvas.allPointsDBShow;
        //----------------------------------------
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'i': /* test approx */
    {
        //========================================
        //wd.pStore->near_passed_build_index();

        WorkerThreadRunTask(wd, _T("\n *** approx ***  "), [](Store &store, RoboI &robo) {
            for (int i = 1; i < 20; ++i)
            {
                tcout << i << _T(" approx sizing=") << i * 2 << std::endl;
                Approx approx(store.size(), 8, Approx::noize, [i]() {return i * 2; });
                approx.constructXY(store);
                //tcout << _T("writing") << std::endl;
                {
                    //tfstream ofs("approx.txt", std::ios_base::out);
                    //boost::archive::text_oarchive toa(ofs);
                    //toa & approx;

                    double sum_error = 0.;
                    for (auto & rec : store)
                    {
                        Point pred = approx.predict(rec.controls);
                        double err = boost_distance(rec.hit, pred);
                        //ofs << pred << " " << rec.hit << " " << err << std::endl;
                        sum_error += err;
                    }
                    //ofs << std::endl << "sum_error=" << sum_error / store.size() << std::endl;
                    std::cout << std::endl << "sum_error=" << sum_error / store.size() << std::endl;

                    for (auto i = 0; i < 1000; ++i)
                    {
                        Control c;
                        c.fillRandom(robo.musclesCount(), [&robo](muscle_t m) { return robo.muscleMaxLasts(m); });

                        robo.reset();
                        robo.move(c);

                        Point pred = approx.predict(c);
                        double err = boost_distance(robo.position(), pred);
                        //ofs << pred << " " << robo.position() << " " << err << std::endl;
                        sum_error += err;
                    }
                    //ofs << std::endl << "sum_error=" << sum_error / 1000 << std::endl;
                    std::cout << std::endl << "sum_error=" << sum_error / 1000 << std::endl;
                }
            }
        }, std::ref(*wd.pStore), std::ref(*wd.pRobo));
        //========================================
        break;
    }
    
    case 't': /* show one random test */
    {
        //========================================
        wd.trajFrames.clear();
        wd.pRobo->reset();

        Control controls;
        controls.fillRandom(wd.pRobo->musclesCount(), [&robo=*wd.pRobo](muscle_t m) { return robo.muscleMaxLasts(m); }, 70, 2, 4, true);
        CDEBUG(controls);

        wd.trajFrames.step(*wd.pStore, *wd.pRobo, controls);
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'o': /* random test */
    {
        //========================================
        const size_t tries = 3000;
        WorkerThreadRunTask(wd, _T("\n *** random test ***  "), testRandom,
                            std::ref(*wd.pStore), std::ref(*wd.pRobo), tries);
        if (WorkerThreadTryJoin(wd))
            InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }

    case 'p': /* cover test */
    {
        //========================================
        WorkerThreadRunTask(wd, _T("\n *** cover test ***  "), testCover,
                            std::ref(*wd.pStore), std::ref(dynamic_cast<Robo::RoboPhysics&>(*wd.pRobo)));
        if (WorkerThreadTryJoin(wd))
            InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }

    case '1': /* Stage 1 */
    {
        //========================================
        WorkerThreadRunTask(wd, _T(" *** STAGE 1 ***  "),
                            [](LearnMoves &lm) { lm.STAGE_1(); }
                            , std::ref(*wd.pLM));
        if (WorkerThreadTryJoin(wd))
            InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }

    case '2': /* Stage 2 */
    {
        //========================================
        WorkerThreadRunTask(wd, _T(" *** STAGE 2 ***  "),
                            [](LearnMoves &lm) { lm.STAGE_2(); },
                            std::ref(*wd.pLM));
        if (WorkerThreadTryJoin(wd))
            InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }

    case '3': /* Stage 3 */
    {
        //========================================
        WorkerThreadRunTask(wd, _T(" *** STAGE 3 ***  "),
                            [](LearnMoves &lm, Trajectory &uncovered) {
            try
            { lm.STAGE_3(uncovered); }
            catch (boost::thread_interrupted&)
            { CINFO("WorkingThread interrupted!"); }
        } , std::ref(*wd.pLM), std::ref(wd.canvas.uncoveredPointsList));
        if (WorkerThreadTryJoin(wd))
            InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }

    case '0': /* free storage */
    {
        //========================================
        wd.pStore->clear();
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }


    case 'u': /* calculate uncovered */
    {
        //========================================
        WorkerThreadRunTask(wd, _T(" *** Uncover ***  "),
                            [](LearnMoves &lm, Trajectory &uncovered) {
            try
            { lm.uncover(uncovered); }
            catch (boost::thread_interrupted&)
            { CINFO("WorkingThread interrupted!"); }
        } , std::ref(*wd.pLM), std::ref(wd.canvas.uncoveredPointsList));
        if (WorkerThreadTryJoin(wd))
            InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }

    case 'w': /* workspace */
    {
        //========================================
        if (!wd.canvas.workingSpaceTraj.size())
            wd.pRobo->getWorkSpace(wd.canvas.workingSpaceTraj);
        wd.canvas.workingSpaceShow = !wd.canvas.workingSpaceShow;
        //----------------------------------------
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'e': /* stop thread */
    {
        //========================================
        if (wd.pWorkerThread)
        {
            wd.pWorkerThread->interrupt();
            if (WorkerThreadTryJoin(wd))
                InvalidateRect(hWnd, &myRect, TRUE);
        }
        //========================================
        break;
    }

    case 'g': /* show units, scales */
    {
        //========================================
        wd.canvas.pLetters->show = !wd.canvas.pLetters->show;
        //----------------------------------------
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'y': /* zoom target */
    {
        //========================================
        MyWindowData::zoom = (MyWindowData::zoom != MyWindowData::Zoom::STATIC) ?
            MyWindowData::Zoom::STATIC : MyWindowData::Zoom::NONE;
        //----------------------------------------
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'n': /* read current wd config */
    {
        wd.read_config(wd._config);
        break;
    }

    case 'm': /* write current wd config */
    {
        tstringstream ss;
        ss << _T("config-") << wd.pRobo->getName()
           << '-' << getCurrentTimeString(_T("%Y.%m.%d-%H.%M"))
           << _T(".json");
        wd.write_config(ss.str());
        break;
    }

    case 'j': /* change color database points */
    {
        //========================================
        wd.canvas.cGradient = CGradient(std::max(1/*not None*/,(int(wd.canvas.cGradient) + 1) % int(CGradient::_Last_)));
        //----------------------------------------
        tstring strGradient[] = { _T("cGradient::None"), _T("cGradient::Longz"), _T("cGradient::Dense"), 
                                  _T("cGradient::Strats"), _T("cGradient::_Last_") };
        CDEBUG(strGradient[int(wd.canvas.cGradient)]);
        //----------------------------------------
        wd.canvas.hStaticBitmapChanged = true;
        //wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'k': /* wheel zoom */
    {
        //========================================
        MyWindowData::zoom = (MyWindowData::zoom != MyWindowData::Zoom::WHEEL) ?
            MyWindowData::Zoom::WHEEL : MyWindowData::Zoom::NONE;
        //----------------------------------------
        windowCenter = { 0., 0. }; currWheelSize = 0.;
        //----------------------------------------
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'l': /* vacant */
    {
        break;
    }

    case 'h': /* show|hide uncovered */
    {
        //========================================
        wd.canvas.uncoveredPointsShow = !wd.canvas.uncoveredPointsShow;
        //----------------------------------------
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }
    //========================================
    /* Wrist */ /* LTrack */
    case 'z': if (!wd.testing) wd.pRobo->step(bitset_t{1 << 0}, wd.pRobo->muscleMaxLasts(0)); break;
    case 'a': if (!wd.testing) wd.pRobo->step(bitset_t{1 << 1}, wd.pRobo->muscleMaxLasts(1)); break;
    /* Elbow */ /* RTrack */                          
    case 'x': if (!wd.testing) wd.pRobo->step(bitset_t{1 << 2}, wd.pRobo->muscleMaxLasts(2)); break;
    case 's': if (!wd.testing) wd.pRobo->step(bitset_t{1 << 3}, wd.pRobo->muscleMaxLasts(3)); break;
    /* Sholder */
    case 'c': if (!wd.testing) wd.pRobo->step(bitset_t{1 << 4}, wd.pRobo->muscleMaxLasts(4)); break;
    case 'd': if (!wd.testing) wd.pRobo->step(bitset_t{1 << 5}, wd.pRobo->muscleMaxLasts(5)); break;
    /* Clavicle */
    case 'v': if (!wd.testing) wd.pRobo->step(bitset_t{1 << 6}, wd.pRobo->muscleMaxLasts(6)); break; /* двинуть ключицей влево */
    case 'f': if (!wd.testing) wd.pRobo->step(bitset_t{1 << 7}, wd.pRobo->muscleMaxLasts(7)); break; /* двинуть ключицей вправо */
    /* Reset */
    case 'r':
    {
        if (wd.testing) break;
        //========================================
        //wd.frames = 0;
        wd.pRobo->reset();

        wd.mouse.click = false;
        wd.trajFrames.clear();

        wd.canvas.testingTrajsList.clear();
        wd.canvas.uncoveredPointsList.clear();
        wd.canvas.pointsDB.clear();
        wd.canvas.trajsDB.clear();

        /* Setting the Label's text */
        SendMessage(wd.canvas.hLabMAim,  /* Label   */
                    WM_SETTEXT,          /* Message */
                    (WPARAM)NULL,        /* Unused  */
                    (LPARAM)_T(" "));
        //----------------------------------------
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }
    }
}
//------------------------------------------------------------------------------
void onWindowKeyDown(HWND hWnd, MyWindowData &wd, WPARAM wParam)
{
    RECT &myRect = wd.canvas.myRect;
    //========================================
    switch (wParam)
    {
    case 'O':
        if (GetAsyncKeyState(VK_CONTROL))
        {
            tstring FileName = OpenFileDialog(hWnd);
            tstring DefaultName = wd.currFileName;
            //========================================
            if (!FileName.empty())
            {
                WorkerThreadRunTask(wd, _T("  *** loading ***  "),
                                    [FileName, DefaultName](RoboMoves::Store &store, MyWindowData &wd) {
                    //if (!store.empty()) // TODO:
                    //{
                    //    store.dump_off(DefaultName);
                    //    store.clear();
                    //}
                    store.pick_up(FileName, wd.pRobo, Store::Format(wd.store_save_load_format));
                }, std::ref(*wd.pStore), std::ref(wd));
                WorkerThreadTryJoin(wd);
                wd.currFileName = FileName;
            }
            //========================================
        }
        break;

    case 'S':
        if (GetAsyncKeyState(VK_CONTROL))
        {
            //========================================
            // tstring FileName = SaveFileDialog(hWnd);
            if (!wd.pStore->empty())
            {
                WorkerThreadRunTask(wd, _T("  *** saving ***  "),
                                    [](const RoboMoves::Store &store, MyWindowData &wd) {
                    store.dump_off(wd.getCurrFileName(), *wd.pRobo, Store::Format(wd.store_save_load_format)); // ??? wd->save();
                }, std::ref(*wd.pStore), std::ref(wd));
                WorkerThreadTryJoin(wd);
            }
            //========================================
        }
        break;

    case 'R':
        if (GetAsyncKeyState(VK_CONTROL))
        {
            //========================================
            wd.pStore->dump_off(wd.getCurrFileName(), *wd.pRobo, Store::Format(wd.store_save_load_format)); // ??? wd.save();
            //========================================
        }
        break;

    case 0x1B: // Esc
        SendMessage(hWnd, WM_QUIT, 0, 0);
        break;
    }

}
//-------------------------------------------------------------------------------
void onWindowMsMove(HWND hWnd, MyWindowData &wd, WPARAM wParam, LPARAM LParam)
{
    POINT pos;
    if (!GetCursorPos(&pos) || !ScreenToClient(hWnd, &pos) || !inside(&wd.canvas.myRect, &pos))
        return;

    Point mouse = LogicCoords(&pos);
    auto s = tstring{ mouse };// +_T("  ");
    SendMessage(wd.canvas.hLabTest, WM_SETTEXT, NULL,
                reinterpret_cast<WPARAM>(s.c_str()));
}
//-------------------------------------------------------------------------------
void onWindowMsWheel(HWND hWnd, MyWindowData &wd, WPARAM WParam, LPARAM LParam)
{
    if (wd.zoom != MyWindowData::Zoom::WHEEL)
        return;
    //========================================
    POINT pos;
    if (!GetCursorPos(&pos) || !ScreenToClient(hWnd, &pos) || !inside(&wd.canvas.myRect, &pos))
        return;
    Point mouse = LogicCoords(&pos);
    //----------------------------------------
    currWheelSize += 1. / (GET_WHEEL_DELTA_WPARAM(WParam) / 5);
    currWheelSize = (currWheelSize < 0.) ? 0. : currWheelSize;
    currWheelSize = (currWheelSize > 1.) ? 1. : currWheelSize;
    //----------------------------------------
    const auto c = currWheelSize;
    windowCenter = mouse;
    windowCenter.x = (0. > windowCenter.x) ? 0. : windowCenter.x;
    windowCenter.y = (-c > windowCenter.y) ? -c : windowCenter.y;
    windowCenter.x = (+c < windowCenter.x) ? +c : windowCenter.x;
    windowCenter.y = (0. < windowCenter.y) ? 0. : windowCenter.y;

    tcout << currWheelSize << ' ' << mouse << ' ' << windowCenter << std::endl;
    //----------------------------------------
    wd.canvas.hStaticBitmapChanged = true;
    wd.canvas.hDynamicBitmapChanged = true;
    //========================================
    InvalidateRect(hWnd, &wd.canvas.myRect, FALSE);
}
//-------------------------------------------------------------------------------
tstring   OpenFileDialog (HWND hWnd)
{
  /* common dialog box structure */
  OPENFILENAME  OpenFileName = {};
  TCHAR         szFilePath[MAX_PATH];  /* buffer for file name */
  TCHAR         szFileName[MAX_PATH];
  
  /* Initialize OpenFileName */
  OpenFileName.lStructSize = sizeof(OPENFILENAME);
  OpenFileName.hwndOwner = hWnd;
  
  OpenFileName.lpstrFileTitle = szFileName;
  OpenFileName.nMaxFileTitle = sizeof(szFileName);
  OpenFileName.lpstrFile = szFilePath;
  OpenFileName.nMaxFile = sizeof(szFilePath);
  
  /*  GetOpenFileName does not use the
   *  contents to initialize itself.
   */
  OpenFileName.lpstrFile[0] = '\0';
  OpenFileName.lpstrFileTitle[0] = '\0';
  
  OpenFileName.lpstrDefExt = _T("bin");
  OpenFileName.lpstrFilter = _T("Binary\0*.bin\0All\0*.*\0");
  OpenFileName.nFilterIndex = 1;
  OpenFileName.lpstrInitialDir = NULL;
  OpenFileName.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_EXPLORER;
  
  // Display the Open dialog box. 
  if (!GetOpenFileName(&OpenFileName))
  { return  _T(""); }
  return  OpenFileName.lpstrFile;
}
tstring   SaveFileDialog (HWND hWnd)
{
  OPENFILENAME  SaveFileName = {};
  TCHAR         szFileName[MAX_PATH] = {};
  TCHAR         szFilePath[MAX_PATH] = {};
  
  SaveFileName.lStructSize = sizeof(OPENFILENAME);
  SaveFileName.hwndOwner = hWnd;
  
  SaveFileName.lpstrFile = szFilePath;
  SaveFileName.nMaxFile = sizeof(szFilePath);
  SaveFileName.lpstrFileTitle = szFileName;
  SaveFileName.nMaxFileTitle = sizeof(szFileName);
  
  SaveFileName.lpstrFile[0] = '\0';
  SaveFileName.lpstrFileTitle[0] = '\0';
  
  SaveFileName.lpstrDefExt = _T("bin");
  SaveFileName.lpstrFilter = _T("Binary\0*.bin\0All\0*.*\0");
  SaveFileName.nFilterIndex = 1;
  SaveFileName.lpstrInitialDir = NULL;
  
  SaveFileName.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_EXPLORER;
  
  // Display the Open dialog box. 
  if (GetSaveFileName(&SaveFileName))
  { return _T(""); }
  return SaveFileName.lpstrFile;
}
//-------------------------------------------------------------------------------
tstring   getWindowTitleString (HWND hWnd)
{
  int  len = GetWindowTextLength(hWnd) + 1U;
  std::vector<TCHAR>  buffer(len);
  GetWindowText(hWnd, buffer.data(), len);
  // SendMessage(hWnd, WM_GETTEXT, (WPARAM) len, (LPARAM) buffer.data ());
  return tstring(buffer.begin(), buffer.end() - 1);
}
//-------------------------------------------------------------------------------
