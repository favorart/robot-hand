#include "StdAfx.h"
#include "WindowData.h"
#include "RoboMovesStore.h"
#include "RoboMovesTarget.h"


#define  WM_USER_TIMER  WM_USER+3
#define  WM_USER_STORE  WM_USER+4
//------------------------------------------------------------------------------
LRESULT CALLBACK  WndProc(HWND hWnd, UINT messg, WPARAM wParam, LPARAM lParam)
{
  static HWND hLabHelp, hLabCanv /* Canvas */, hLabMAim, hLabTest, hLabStat;
  static RECT myRect;
  static LabelsPositions lp;
  // ----------------------
  static std::shared_ptr<MyWindowData> wd;
  // ----------------------
  try
  {
    switch (messg)
    {
    case WM_CREATE:
    {
      redirectConsoleIO();
      SetConsoleTitle(_T("cmd-robo-moves"));
      //=======================
      tstring config, database;
      getConsoleArguments(config, database);
      //=======================
      std::srand(112); /// (unsigned int)clock());
      //=======================
      wd = std::make_shared<MyWindowData>(config, database);
      //=======================
      onWindowCreate(hWnd, myRect, hLabCanv, hLabHelp,
                     hLabMAim, hLabTest, hLabStat, lp,
                     getJointsHelp(*wd->pRobo));
      //=======================
      wd->canvas.hLabMAim = hLabMAim;
      wd->canvas.hLabTest = hLabTest;
      wd->canvas.hLabStat = hLabStat;
      //=======================
      wd->pRobo->reset();
      //=======================
      WorkerThreadRunTask(*wd, _T("  *** loading ***  "),
                          [](RoboMoves::Store &store, const tstring &filename) { store.load(filename); },
                          std::ref(*wd->pStore), wd->currFileName);
      WorkerThreadTryJoin(*wd);
      //=======================
      SendMessage(hWnd, WM_USER_STORE, NULL, NULL);
      //=======================
      break;
    }

    case WM_PAINT:
    {
      //=======================
      onWindowPaint(hWnd, myRect, *wd);
      //=======================
      break;
    }

    case WM_CTLCOLORSTATIC:
    {
      /* Make background color of labels */
      HDC hdcStatic = (HDC)(wParam);
      SetBkMode(hdcStatic, TRANSPARENT);
      return (LRESULT)CreateSolidBrush(RGB(255, 255, 255));
    }

    case WM_ERASEBKGND:
    {
      return 1;
    }

    case WM_QUIT:
    {
      SendMessage(hWnd, WM_DESTROY, 0, 0);
      break;
    }

    case WM_DESTROY:
    {
      PostQuitMessage(0);
      break;
    }

    case WM_MOVE:
    case WM_SIZE:
    {
      onWindowSize(hWnd, myRect, hLabCanv, hLabHelp,
             hLabMAim, hLabTest, hLabStat, lp);
      wd->canvas.hStaticBitmapChanged = true;
      break;
    }

    case WM_GETMINMAXINFO:
    {
      ((MINMAXINFO*)lParam)->ptMinTrackSize.x = WIDTH;
      ((MINMAXINFO*)lParam)->ptMinTrackSize.y = HIGHT;
      break;
    }

    case WM_USER_TIMER:
    {
      KillTimer(hWnd, IDT_TIMER_VISION);
      break;
    }

    case WM_TIMER:
    {
      //-----------------------
      switch (wParam)
      {
      case IDT_TIMER_STROKE:
      {
        //=======================
        onWindowTimer(*wd);
        //=======================
        break;
      }
      case IDT_TIMER_VISION:
      {
        //=======================
        onWindowTimer(*wd);
        //=======================
        if (wd->canvas.store_size != wd->pStore->size())
        {
          SendMessage(hWnd, WM_USER_STORE, NULL, NULL);
          if (!wd->testing)
            wd->canvas.hStaticBitmapChanged = true;
        }
        if (wd->testing)
        { WorkerThreadTryJoin(*wd); }
        //=======================
        InvalidateRect(hWnd, &myRect, false);
        break;
      }
      }
      //-----------------------
      break;
    }

    case WM_USER_STORE:
    {
      //=======================
      {
        tstringstream ss;
        ss << _T("Storage size: ") << wd->pStore->size() << _T("  ");
        wd->canvas.store_size = wd->pStore->size();
        /* Setting the Label's text */
        SendMessage(wd->canvas.hLabStat, /* Label Stat */
                    WM_SETTEXT,          /* Message    */
                    (WPARAM)NULL,        /* Unused     */
                    (LPARAM)ss.str().c_str());
      }
      //=======================
      break;
    }

    case WM_LBUTTONDOWN:
    {
      /* Если был щелчок левой кнопкой */
      wd->mouse.coords.x = LOWORD(lParam);
      wd->mouse.coords.y = HIWORD(lParam);
      /* узнаём координаты */
      //=======================
      if ((LONG)wd->mouse.coords.x > myRect.left
       && (LONG)wd->mouse.coords.x < myRect.right
       && (LONG)wd->mouse.coords.y > myRect.top
       && (LONG)wd->mouse.coords.y < myRect.bottom)
      {
        onWindowMouse(*wd);
      }
      //=======================
      InvalidateRect(hWnd, &myRect, 0);
      break;
    }

    case WM_CHAR:
    {
      //=======================
      onWindowKeyDown(hWnd, myRect, wParam, lParam, *wd);
      //=======================
      break;
    }

    case WM_KEYDOWN:
    {
      switch (wParam)
      {
      case 'O':
        if (GetAsyncKeyState(VK_CONTROL))
        {
          tstring FileName = OpenFileDialog(hWnd);
          tstring DefaultName = wd->currFileName;

          if (!FileName.empty())
          {
            WorkerThreadRunTask(*wd, _T("  *** loading ***  "),
                                [FileName, DefaultName](RoboMoves::Store &store) {
              if (!store.empty())
              {
                store.save(DefaultName);
                store.clear();
              }
              store.load(FileName);
            }, std::ref(*wd->pStore));
            WorkerThreadTryJoin(*wd);
            wd->currFileName = FileName;
          }
        }
        break;

      case 'S':
        if (GetAsyncKeyState(VK_CONTROL))
        {
          // tstring FileName = SaveFileDialog(hWnd);
          if (!wd->pStore->empty())
          {
            tstringstream ss;
            ss << wd->pRobo->name() << _T("-robo")
               << getCurrentTimeString(_T("_%d-%m-%Y_%I-%M-%S"))
               << _T("_moves.bin");
            WorkerThreadRunTask(*wd, _T("  *** saving ***  "),
                                [](const RoboMoves::Store &store, const tstring &filename) { store.save(filename); },
                                std::ref(*wd->pStore), ss.str());
            WorkerThreadTryJoin(*wd);
          }
        }
        break;

      case 'R':
        if (GetAsyncKeyState(VK_CONTROL))
        {
          tstringstream ss;
          ss << wd->pRobo->name() << _T("-robo")
             << getCurrentTimeString(_T("_%d-%m-%Y_%I-%M-%S"))
             << _T("_moves.bin");
          wd->pStore->save(ss.str());
        }
        break;

      case 0x1B: // Esc
        SendMessage(hWnd, WM_QUIT, 0, 0);
        break;
      }
      break;
    }

    default:
      return (DefWindowProc(hWnd, messg, wParam, lParam));
    }
  }
  catch (std::exception &e)
  {
    tstring line = Utils::uni(std::string{ e.what() });
    MessageBox(hWnd, line.c_str(), _T("Error"), MB_OK | MB_ICONERROR);
    InvalidateRect(hWnd, &myRect, FALSE);
  }
  catch (...)
  {
    tcerr << _T("Error") << std::endl;
  }

  return (0);
}
//------------------------------------------------------------------------------
