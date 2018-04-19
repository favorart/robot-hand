#include "StdAfx.h"
#include "WindowData.h"
#include "WindowDraw.h"
#include "WindowDrawLetters.h"

#include "RoboPos.h"
#include "RoboMuscles.h"
#include "RoboLearnMoves.h"


using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
/* normal */
LONG Tx_norm (double logic_x)
{ return static_cast<LONG>(MARGIN + ( 0.5) * (logic_x + 1.) * (WindowSize ()->x - 2. * MARGIN)); }
LONG Ty_norm (double logic_y)
{ return static_cast<LONG>(MARGIN + (-0.5) * (logic_y - 1.) * (WindowSize ()->y - 2. * MARGIN)); }

/* zoom */
LONG Tx_zoom (double logic_x)
{ return static_cast<LONG>(MARGIN + ( 1.) * (logic_x + 0.5) * (WindowSize ()->x - 2. * MARGIN)); }
LONG Ty_zoom (double logic_y)
{ return static_cast<LONG>(MARGIN + (-1.) * (logic_y - 0.0) * (WindowSize ()->y - 2. * MARGIN)); }

LONG  Tx  (double logic_x)
{ return  (MyWindowData::zoom) ? Tx_zoom(logic_x) : Tx_norm(logic_x); }
LONG  Ty  (double logic_y)
{ return  (MyWindowData::zoom) ? Ty_zoom(logic_y) : Ty_norm(logic_y); }

// наоборот: координаты Windows -> логические координаты
Point LogicCoords (PPOINT coord)
{
  Point p;
  if ( MyWindowData::zoom )
  { p.x = ((coord->x - MARGIN) / (( 1.0) * (WindowSize ()->x - 2. * MARGIN))) - 0.5;
    p.y = ((coord->y - MARGIN) / ((-1.0) * (WindowSize ()->y - 2. * MARGIN))) + 0.0;
  }
  else
  { p.x = ((coord->x - MARGIN) / (( 0.5) * (WindowSize ()->x - 2. * MARGIN))) - 1.;
    p.y = ((coord->y - MARGIN) / ((-0.5) * (WindowSize ()->y - 2. * MARGIN))) + 1.;
  }
  return p;
}
//-------------------------------------------------------------------------------
/* Create a string with last error message */
tstring getLastErrorString()
{
  DWORD error = GetLastError();
  if (error)
  {
    LPVOID lpMsgBuf;
    DWORD bufLen = FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER |
                                  // FORMAT_MESSAGE_IGNORE_INSERTS |
                                  FORMAT_MESSAGE_FROM_SYSTEM,
                                  NULL,
                                  error,
                                  MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                                  (LPTSTR)&lpMsgBuf,
                                  0, NULL);
    if (bufLen)
    {
      LPCSTR lpMsgStr = (LPCSTR)lpMsgBuf;
      tstring result(lpMsgStr, lpMsgStr + bufLen);
      
      LocalFree(lpMsgBuf);
      return result;
    }
  }
  return tstring{};
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
  if ( !hLabCanv )
    MessageBox (hWnd, _T ("Could not create hLabCanv."), _T ("Error"), MB_OK | MB_ICONERROR);

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
     << _T ("  G - показать масштаб и размеры  \r\r")
     << _T ("  O - Random Test,   P - Cover Test  \r")
     << _T ("  1 - STAGE,  2 - STAGE,  3 - STAGE  \r\r")
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

  if (fs::exists(wd.currFileName))
  {
      WorkerThreadRunTask(wd, _T("  *** loading ***  "),
                          [](MyWindowData &wd, const tstring &filename) {
                              wd.pStore->pick_up(filename);
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
      if ( !wd.canvas.hStaticDC ) CERROR("");
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
    if ( !wd.canvas.hStaticBitmap ) CERROR("");
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
  if ( !hCmpDC ) CERROR("");

  HBITMAP hBmp = CreateCompatibleBitmap (hdc, myRect.right  - myRect.left,
                                              myRect.bottom - myRect.top);
  if ( !hBmp ) CERROR("");
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
        InvalidateRect(hWnd, &wd.canvas.myRect, TRUE); // FALSE
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
        /// TODO: (onWindowMouse): if (!wd.testing) { makeRoboMove(wd); }
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
    case 0x0D: /* Process a carriage return */
    {
        //========================================
        wd.canvas.hDynamicBitmapChanged = true;
        InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }

    case 'q':
    {
        //========================================
        // onShowDBPoints (wd);
        wd.canvas.allPointsDBShow = !wd.canvas.allPointsDBShow;
        //========================================
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'i':
    {
        //========================================
        wd.pStore->near_passed_build_index();
        //========================================
        break;
    }
    
    case 't':
    {
        //========================================
        wd.trajFrames.clear();
        wd.pRobo->reset();

        // // wd.trajFrames_muscle = selectRoboIMove (random (RoboIMovesCount));
        // wd.trajFrames_muscle = wd.pRobo->selectControl ();
        // wd.trajFrames_lasts = random (1U, wd.pRobo->muscleMaxLast (wd.trajFrames_muscle));
        Control controls;
        controls.fillRandom(wd.pRobo->musclesCount(),
                            [&robo=*wd.pRobo](muscle_t m) { return (robo.muscleMaxLast(m) / 2); }); /// Tank: 500, 2

        wd.trajFrames.step(*wd.pStore, *wd.pRobo, boost::optional<Control>{controls});
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'o':
    { //========================================
        const size_t  tries = 1000U;
        WorkerThreadRunTask(wd, _T("\n *** random test ***  "), testRandom,
                            std::ref(*wd.pStore), std::ref(*wd.pRobo), tries);
        if (WorkerThreadTryJoin(wd))
            InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }

    case 'p':
    {
        //========================================
        const size_t nested = 2U;
        WorkerThreadRunTask(wd, _T("\n *** cover test ***  "), testCover,
                            std::ref(*wd.pStore), std::ref(*wd.pRobo), nested);
        if (WorkerThreadTryJoin(wd))
            InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }

    case '1':
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

    case '2':
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

    case '3':
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

    case '0':
    {
        //========================================
        wd.pStore->clear();
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'j':
    {
        wd.pLM->testStage3(wd.canvas.uncoveredPointsList);  /* OLD */
        //========================================
        wd.canvas.hDynamicBitmapChanged = true;
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'k':
    {
        //========================================
        WorkerThreadRunTask(wd, _T(" *** Stage 2 test ***  "),
                            [](LearnMoves &lm) { lm.testStage2(); /* OLD */ },
                            std::ref(*wd.pLM));
        if (WorkerThreadTryJoin(wd))
            InvalidateRect(hWnd, &myRect, TRUE);
        //========================================
        break;
    }


    case 'u':
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

    case 'w':
    {
        //========================================
        if (!wd.canvas.workingSpaceTraj.size())
            wd.pRobo->drawWorkSpace(wd.canvas.workingSpaceTraj);
        wd.canvas.workingSpaceShow = !wd.canvas.workingSpaceShow;
        //========================================
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'e':
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

    case 'g':
    {
        //========================================
        wd.canvas.pLetters->show = !wd.canvas.pLetters->show;
        //========================================
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'y':
    {
        //========================================
        MyWindowData::zoom = !MyWindowData::zoom;
        //========================================
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'h':
    {
        //========================================
        wd.canvas.uncoveredPointsShow = !wd.canvas.uncoveredPointsShow;
        //========================================
        wd.canvas.hStaticBitmapChanged = true;
        wd.canvas.hDynamicBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }
    //========================================
    /* Wrist */ /* LTrack */
    case 'z': wd.pRobo->step(wd.frames, 0, wd.pRobo->muscleMaxLast(0)); ++wd.frames; break; /* двинуть ключицей влево */
    case 'a': wd.pRobo->step(wd.frames, 1, wd.pRobo->muscleMaxLast(1)); ++wd.frames; break; /* двинуть ключицей вправо */
    /* Elbow */ /* RTrack */                                                        
    case 'x': wd.pRobo->step(wd.frames, 2, wd.pRobo->muscleMaxLast(2)); ++wd.frames; break;
    case 's': wd.pRobo->step(wd.frames, 3, wd.pRobo->muscleMaxLast(3)); ++wd.frames; break;
    /* Sholder */                                                                   
    case 'c': wd.pRobo->step(wd.frames, 4, wd.pRobo->muscleMaxLast(4)); ++wd.frames; break;
    case 'd': wd.pRobo->step(wd.frames, 5, wd.pRobo->muscleMaxLast(5)); ++wd.frames; break;
    /* Clavicle */                                                                  
    case 'v': wd.pRobo->step(wd.frames, 6, wd.pRobo->muscleMaxLast(6)); ++wd.frames; break;
    case 'f': wd.pRobo->step(wd.frames, 7, wd.pRobo->muscleMaxLast(7)); ++wd.frames; break;
    /* Reset */
    case 'r':
    {
        //========================================
        wd.frames = 0;
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
        //========================================
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
                                    [FileName, DefaultName](RoboMoves::Store &store) {
                    /// TODO:
                    ///if (!store.empty())
                    ///{
                    ///    store.dump_off(DefaultName);
                    ///    store.clear();
                    ///}
                    store.pick_up(FileName);
                }, std::ref(*wd.pStore));
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
                                    [](const RoboMoves::Store &store, const tstring &filename) {
                    store.dump_off(filename); /// wd->save();
                }, std::ref(*wd.pStore), wd.getCurrFileName());
                WorkerThreadTryJoin(wd);
            }
            //========================================
        }
        break;

    case 'R':
        if (GetAsyncKeyState(VK_CONTROL))
        {
            //========================================
            wd.pStore->dump_off(wd.getCurrFileName()); /// wd.save();
            //========================================
        }
        break;

    case 0x1B: // Esc
        SendMessage(hWnd, WM_QUIT, 0, 0);
        break;
    }

}
//-------------------------------------------------------------------------------
/*!
*
*   \param[in]  viewRect    rectangle of the viewed area
*   \param[in]  zoomFactor  factor of zoom relative to viewRect, ex 1.1
*   \param[in]  mousePos    position of the mouse
*   \param[out] zoomedRect  viexRect after zoom
*
*    A little schema:
*
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
*    dX = d_left + d_right
*    dY = d_up + d_down
*
*    The origin of rects is the upper left corner.
*/
void zoomArea(IN Point &zoomCenter, IN double zoomFactor, OUT RECT &zoomedRect)
{
    double view_left = -1., view_right = 1., view_top = 1., view_bottom = -1.;
    /*
    *    First, find differences of size between zoomed rect and original rect
    *    Here, 1 / zoomFactor is used, because computations are made relative to the
    *    original view area, not the final rect):
    */
    double dX = 2. /*width(viewRect)*/ * (1. - 1. / zoomFactor);
    double dY = 2. /*height(viewRect)*/ * (1. - 1. / zoomFactor);

    /*
    *    Second, find d_* using the position of the mouse.
    *    pX = position of the mouse along X axis, relative to viewRect (percentage)
    *    pY = position of the mouse along Y axis, relative to viewRect (percentage)
    *    The value of d_right and d_down is not computed because is not directly needed
    *    in the final result.
    */
    
    double pX = (zoomCenter.x - view_left) / 2. /*width(viewRect)*/;
    double pY = (zoomCenter.y - view_top) / 2. /*height(viewRect)*/;

    double d_left = pX * dX;
    double d_up = pY * dY;

    /* Third and last, compute the output rect */
    zoomedRect.left = Tx(view_left + d_left);
    zoomedRect.top = Ty(view_top + d_up);
    zoomedRect.right = Tx(view_right - dX);
    zoomedRect.bottom = Ty(view_bottom - dY);
}

// That's it!
// For your problem, you need to separate the view(your window) 
// from the scene(objects that are drawed).
// You should have a function drawing a part of(or all) the scene :

// void drawScene(RECT viewArea);
// //and a function zooming an area(using the algorithm presented before) :
// RECT zoomArea(RECT rectToZoom, Point zoomCenter, double factor);
// // Now, your callback is a lot more simpler :

void onMouseWheel(HWND hWnd, MyWindowData &wd, WPARAM WParam, LPARAM LParam)
{
    POINT pos;
    if (!GetCursorPos(&pos))
        return;

    Point mousePos = LogicCoords(&pos);
    Point mousePosRelative;

    // Get the position of the mouse relative to the window (in percent)
    mousePosRelative.x = mousePos.x / 2. /*double(Window.GetClientWidth())*/;
    mousePosRelative.y = mousePos.y / 2. /*double(Window.GetClientHeight())*/;

    // // Get Mouse position in scene coordinates and not window coordinates.
    // // viewArea is in scene coordinates
    // // window = your window or your draw information on the scene
    // // The following assumes that you're using a scene with X left-to-right and Y top-to-bottom.
    // double XMouse = window.viewArea.width * mousePosRelative.x + window.viewArea.upperleft.X;
    // double YMouse = window.viewArea.height * mousePosRelative.y + window.viewArea.upperleft.Y;
    // 
    // // Zoom parameters
    // double zFactor = 0.1 * GET_WHEEL_DELTA_WPARAM(WParam);
    // 
    // RECT viewArea = getViewArea(); // or something like this
    // Point zCenter(XMouse, YMouse);
    // 
    // // Zoom
    // RECT zoomedRect = zoomArea(viewArea, zCenter, zFactor);
    // drawScene(zoomedRect);
}


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
tstring   getCurrentTimeString (tstring format, std::time_t *the_time)
{
  std::time_t rawtime;
  if (!the_time)
  { rawtime = std::time(nullptr); }
  else
  { rawtime = *the_time; }
  struct tm  *TimeInfo = std::localtime(&rawtime);
  
  tstringstream ss;
  ss << std::put_time(TimeInfo, format.c_str());
  return ss.str();
}
//-------------------------------------------------------------------------------
