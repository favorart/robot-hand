#include "StdAfx.h"
#include "WindowData.h"
#include "WindowDraw.h"
#include "WindowDrawLetters.h"

#include "RoboPos.h"
#include "RoboMuscles.h"
#include "RoboLearnMoves.h"


using namespace Robo;
using namespace RoboMoves;
//------------------------------------------------------------------------------
/* normal */
uint_t Tx_norm (double logic_x)
{ return static_cast<uint_t>(MARGIN + ( 0.5) * (logic_x + 1.) * (WindowSize ()->x - 2. * MARGIN)); }
uint_t Ty_norm (double logic_y)
{ return static_cast<uint_t>(MARGIN + (-0.5) * (logic_y - 1.) * (WindowSize ()->y - 2. * MARGIN)); }

/* zoom */
uint_t Tx_zoom (double logic_x)
{ return static_cast<uint_t>(MARGIN + ( 1.) * (logic_x + 0.5) * (WindowSize ()->x - 2. * MARGIN)); }
uint_t Ty_zoom (double logic_y)
{ return static_cast<uint_t>(MARGIN + (-1.) * (logic_y - 0.0) * (WindowSize ()->y - 2. * MARGIN)); }

uint_t  Tx  (double logic_x)
{ return  (MyWindowData::zoom) ? Tx_zoom(logic_x) : Tx_norm(logic_x); }
uint_t  Ty  (double logic_y)
{ return  (MyWindowData::zoom) ? Ty_zoom(logic_y) : Ty_norm(logic_y); }

// наоборот: координаты Windows -> логические координаты
Point LogicCoords (win_point* coord)
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
tstring  getLastErrorString ()
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
  return tstring();
}
//-------------------------------------------------------------------------------
tstring getJointsHelp(const Robo::RoboI& robo)
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
void onWindowCreate (HWND &hWnd, RECT &myRect,
                     HWND &hLabCanv, HWND &hLabHelp,
                     HWND &hLabMAim, HWND &hLabTest,
                     HWND &hLabStat, LabelsPositions &lp,
                     tstring &jointsHelp)
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
     << jointsHelp
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

  SetTimer (hWnd,                   /* RoboIle to main window */
            IDT_TIMER_VISION,       /* Timer identifier      */
            120 /*30*/,             /* 1/10-second interval  */
            (TIMERPROC) NULL);      /* No Timer callback     */
}

void onWindowSize (HWND &hWnd, RECT &myRect,
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

  setWindowSize ( /* Rect.right - Rect.left */
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

void onWindowPaint (HWND &hWnd, RECT &myRect,
                    MyWindowData &wd)
{
  PAINTSTRUCT ps = {};
  
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
      if ( !wd.canvas.hStaticDC )
        MessageBox (hWnd, getLastErrorString ().c_str (), _T("ERROR"), MB_OK | MB_ICONERROR);
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
    if ( !wd.canvas.hStaticBitmap )
      MessageBox (hWnd, getLastErrorString ().c_str (), _T("ERROR"), MB_OK | MB_ICONERROR);
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
  if ( !hCmpDC )
    MessageBox (hWnd, getLastErrorString ().c_str (), _T("ERROR"), MB_OK | MB_ICONERROR);

  HBITMAP hBmp = CreateCompatibleBitmap (hdc, myRect.right  - myRect.left,
                                              myRect.bottom - myRect.top);
  if ( !hBmp )
    MessageBox (hWnd, getLastErrorString ().c_str (), _T("ERROR"), MB_OK | MB_ICONERROR);
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
void onWindowKeyDown(HWND &hWnd, RECT &myRect,
                     WPARAM wParam, LPARAM lparam,
                     MyWindowData &wd)
{
    char symbol = static_cast<char>(wParam);
    switch (symbol)
    {
    case 0x0D: /* Process a carriage return */
    {
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'q':
    {
        //========================================
        // onShowDBPoints (wd);
        wd.canvas.allPointsDBShow = !wd.canvas.allPointsDBShow;
        wd.canvas.hStaticBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, FALSE);
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
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'o':
    { //========================================
        const size_t  tries = 1000U;
        WorkerThreadRunTask(wd, _T("\n *** random test ***  "), testRandom,
                            std::ref(*wd.pStore), std::ref(*wd.pRobo), tries);
        WorkerThreadTryJoin(wd);
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'p':
    {
        //========================================
        const size_t nested = 2U;
        WorkerThreadRunTask(wd, _T("\n *** cover test ***  "), testCover,
                            std::ref(*wd.pStore), std::ref(*wd.pRobo), nested);
        WorkerThreadTryJoin(wd);
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case '1':
    {
        //========================================
        WorkerThreadRunTask(wd, _T(" *** STAGE 1 ***  "),
                            [](RoboPos::LearnMoves &lm) { lm.STAGE_1(); }
                            , std::ref(*wd.pLM));
        WorkerThreadTryJoin(wd);
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case '2':
    {
        //========================================
        WorkerThreadRunTask(wd, _T(" *** STAGE 2 ***  "),
                            [](RoboPos::LearnMoves &lm) { lm.STAGE_2(); },
                            std::ref(*wd.pLM));
        WorkerThreadTryJoin(wd);
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case '3':
    {
        //========================================
        WorkerThreadRunTask(wd, _T(" *** STAGE 3 ***  "),
                            [](RoboPos::LearnMoves &lm, Trajectory &uncovered, size_t &complexity) {
            try
            { lm.STAGE_3(uncovered, complexity, false); }
            catch (boost::thread_interrupted&)
            { /* tcout << _T("WorkingThread interrupted!") << std::endl; */ }
        } , std::ref(*wd.pLM), std::ref(wd.canvas.uncoveredPointsList), std::ref(wd.complexity));
        WorkerThreadTryJoin(wd);
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case '0':
    {
        //========================================
        wd.pStore->clear();
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'j':
    {
        wd.pLM->testStage3(wd.canvas.uncoveredPointsList);  /* OLD */
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'k':
    {
        //========================================
        WorkerThreadRunTask(wd, _T(" *** Stage 2 test ***  "),
                            [](RoboPos::LearnMoves &lm) { lm.testStage2(); /* OLD */ },
                            std::ref(*wd.pLM));
        WorkerThreadTryJoin(wd);
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }


    case 'u':
    {
        //========================================
        WorkerThreadRunTask(wd, _T(" *** Uncover ***  "),
                            [](RoboPos::LearnMoves &lm, Trajectory &uncovered) {
            try
            { lm.uncover(uncovered); }
            catch (boost::thread_interrupted&)
            { /* tcout << _T("WorkingThread interrupted") << std::endl; */ }
        } , std::ref(*wd.pLM), std::ref(wd.canvas.uncoveredPointsList));
        WorkerThreadTryJoin(wd);
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'i':
    {
        //========================================
        /// ???
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }

    case 'w':
    {
        //========================================
        if (!wd.canvas.workingSpaceTraj.size())
            wd.pRobo->drawWorkSpace(wd.canvas.workingSpaceTraj);

        wd.canvas.workingSpaceShow = !wd.canvas.workingSpaceShow;
        wd.canvas.hStaticBitmapChanged = true;
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
            WorkerThreadTryJoin(wd);
        }
        //========================================
        break;
    }

    case 'g':
    {
        //========================================
        wd.canvas.pLetters->show = !wd.canvas.pLetters->show;
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
        InvalidateRect(hWnd, &myRect, FALSE);
        break;
    }

    case 'h':
    {
        //========================================
        wd.canvas.uncoveredPointsShow = !wd.canvas.uncoveredPointsShow;
        wd.canvas.hStaticBitmapChanged = true;
        //========================================
        InvalidateRect(hWnd, &myRect, TRUE);
        break;
    }
    //========================================
    /* Wrist */ /* LTrack */
    case 'z': wd.pRobo->step(wd.frames, 0, wd.pRobo->muscleMaxLast(0)); /*wd.pRobo->step(wd.frames);*/ ++wd.frames; break; /* двинуть ключицей влево */
    case 'a': wd.pRobo->step(wd.frames, 1, wd.pRobo->muscleMaxLast(1)); /*wd.pRobo->step(wd.frames);*/ ++wd.frames; break; /* двинуть ключицей вправо */
    /* Elbow */ /* RTrack */                                            /*                          */             
    case 'x': wd.pRobo->step(wd.frames, 2, wd.pRobo->muscleMaxLast(2)); /*wd.pRobo->step(wd.frames);*/ ++wd.frames; break;
    case 's': wd.pRobo->step(wd.frames, 3, wd.pRobo->muscleMaxLast(3)); /*wd.pRobo->step(wd.frames);*/ ++wd.frames; break;
    /* Sholder */                                                       /*                          */             
    case 'c': wd.pRobo->step(wd.frames, 4, wd.pRobo->muscleMaxLast(4)); /*wd.pRobo->step(wd.frames);*/ ++wd.frames; break;
    case 'd': wd.pRobo->step(wd.frames, 5, wd.pRobo->muscleMaxLast(5)); /*wd.pRobo->step(wd.frames);*/ ++wd.frames; break;
    /* Clavicle */                                                      /*                          */             
    case 'v': wd.pRobo->step(wd.frames, 6, wd.pRobo->muscleMaxLast(6)); /*wd.pRobo->step(wd.frames);*/ ++wd.frames; break;
    case 'f': wd.pRobo->step(wd.frames, 7, wd.pRobo->muscleMaxLast(7)); /*wd.pRobo->step(wd.frames);*/ ++wd.frames; break;
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
        break;
    }
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
