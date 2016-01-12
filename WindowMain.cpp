#include "StdAfx.h"
#include "MyWindow.h"


int WINAPI  WinMain (HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{ HWND      hWnd;  
  MSG       lpMsg;
  WNDCLASS  wc;
	TCHAR     szClassName[] = _T("Graph interface window");

  // Заполняем структуру класса окна
  wc.style         = CS_HREDRAW | CS_VREDRAW;
  wc.lpfnWndProc   = WndProc;
  wc.cbClsExtra    = 0;
  wc.cbWndExtra    = 0;
  wc.hInstance     = hInstance;
  wc.hIcon         = NULL;      
  wc.hCursor       = LoadCursor (NULL, IDC_ARROW);
  wc.hbrBackground = (HBRUSH) GetStockObject (WHITE_BRUSH);
  wc.lpszMenuName  = NULL;
  wc.lpszClassName = szClassName;
  
  // Регистрируем класс окна
  if( !RegisterClass (&wc) )
  { MessageBox (NULL, _T("Can't register window class."), _T("Error"), MB_OK);
    return 0;
  }
  
  // Создаем основное окно приложения
  hWnd = CreateWindow ( szClassName,                // Имя класса                    
                        _T("robot-hand"),            // Текст заголовка 
                        WS_OVERLAPPEDWINDOW,        // Стиль окна                                              
                        0, 0,											 // Позиция левого верхнего угла   
                        WIDTH, HIGHT,               // Ширина и высота окна     
                        (HWND) NULL,                // Указатель на родительское окно NULL     
                        (HMENU) NULL,               // Используется меню класса окна               
                        (HINSTANCE) hInstance,      // Указатель на текущее приложение
                        NULL );					           // Передается в качестве lParam в событие WM_CREATE
  
  if( !hWnd ) 
  { MessageBox (NULL, _T("Can't create main window!"), _T("Error"), MB_OK);  
	  return 0;
  }
  
  // Показываем наше окно
  ShowWindow   (hWnd, nCmdShow); 
  UpdateWindow (hWnd);
  
  // Выполняем цикл обработки сообщений до закрытия приложения
  while ( GetMessage (&lpMsg, NULL, 0, 0) )  
  { TranslateMessage (&lpMsg);
     DispatchMessage (&lpMsg);
  }
  
  return (lpMsg.wParam);
}
