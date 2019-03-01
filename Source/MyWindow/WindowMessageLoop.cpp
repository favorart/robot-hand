#include "StdAfx.h"
#include "WindowData.h"
#include "WindowHeader.h"
#include "RoboMovesStore.h"
#include "RoboMovesTarget.h"

//------------------------------------------------------------------------------
LRESULT CALLBACK WndProc(HWND hWnd, UINT messg, WPARAM wParam, LPARAM lParam)
{
  // получаем указатель на пользовательские данные из hWnd
  auto *wd = reinterpret_cast<MyWindowData*>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
  // ----------------------
  try
  {
    switch (messg)
    {
    case WM_NCCREATE:
    {
        auto lpcs = reinterpret_cast<LPCREATESTRUCT>(lParam);
        //=======================
        wd = reinterpret_cast<MyWindowData*>(lpcs->lpCreateParams);
        // кладём полученное из WinMain значение указателя в выделенное место в hWnd
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(wd));
        //=======================
        break;
    }

    case WM_CREATE:
    {
      //=======================
      onWindowCreate(hWnd, *wd);
      //=======================
      break;
    }

    case WM_PAINT:
    {
      //=======================
      onWindowPaint(hWnd, *wd);
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
      //=======================
      onWindowSize(hWnd, *wd);
      //=======================
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
      //=======================
      onWindowTimer(hWnd, *wd, wParam);
      //=======================
      break;
    }

    case WM_USER_STORE:
    {
      //=======================
      onWindowStoreSz(hWnd, *wd);
      //=======================
      break;
    }

    case WM_LBUTTONDOWN:
    {
      //=======================
      onWindowMouse(hWnd, *wd, wParam, lParam);
      //=======================
      break;
    }

    case WM_CHAR:
    {
      //=======================
      onWindowChar(hWnd, *wd, wParam, lParam);
      //=======================
      break;
    }

    case WM_KEYDOWN:
    {
      //=======================
      onWindowKeyDown(hWnd, *wd, wParam);
      //=======================
      break;
    }

    case WM_MOUSEMOVE:
    {
        //=======================
        onWindowMsMove(hWnd, *wd, wParam, lParam);
        //=======================
        break;
    }

    case WM_MOUSEWHEEL:
    {
        //=======================
        onWindowMsWheel(hWnd, *wd, wParam, lParam);
        //=======================
        break;
    }

    default:
      break;
    }
  }
  catch (const std::exception &e)
  {
    SHOW_CERROR(e.what());
    InvalidateRect(hWnd, &wd->canvas.myRect, FALSE);
  }

  return DefWindowProc(hWnd, messg, wParam, lParam);
}
//------------------------------------------------------------------------------
