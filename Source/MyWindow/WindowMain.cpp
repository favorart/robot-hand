#include "StdAfx.h"
#include "WindowHeader.h"
#include "WindowData.h"


int WINAPI  WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
  HWND      hWnd;
  MSG       lpMsg;
  WNDCLASS  wc;
  TCHAR     szClassName[] = _T("Graph interface window");

  // Заполняем структуру класса окна
  wc.style         = CS_HREDRAW | CS_VREDRAW;
  wc.lpfnWndProc   = WndProc;
  wc.cbClsExtra    = 0;
  wc.hInstance     = hInstance;
  wc.hIcon         = NULL;      
  wc.hCursor       = LoadCursor (NULL, IDC_ARROW);
  wc.hbrBackground = (HBRUSH) GetStockObject (WHITE_BRUSH);
  wc.lpszMenuName  = NULL;
  wc.lpszClassName = szClassName;
  wc.cbWndExtra    = sizeof(MyWindowData*); // выделяем место под указатель на пользовательские данные в hWnd
  
  // Регистрируем класс окна
  if( !RegisterClass (&wc) )
  { MessageBox (NULL, _T("Can't register window class."), _T("Error"), MB_OK);
    return 0;
  }
  
  //=======================
  // Создаём дополнительное окно с коммандной строкой
  // Перенаправляем в неё стандартные потоки ввода/вывода
  redirectConsoleIO();
  // Изменяем заголовок консоли
  SetConsoleTitle(_T("cmd-robo-moves"));
  //=======================
  tstring config, database;
  // Получаем параметры командной строки
  getConsoleArguments(config, database);
  //=======================
  std::srand(112); /// (unsigned int)clock());
  //=======================
  // Инициализируем пользовательские данные
  MyWindowData wd(config, database);
  //=======================
  
  // Создаем основное окно приложения
  hWnd = CreateWindow ( szClassName,                // Имя класса                    
                        _T("robot-moves"),          // Текст заголовка 
                        WS_OVERLAPPEDWINDOW,        // Стиль окна                                              
                        0, 0,                       // Позиция левого верхнего угла   
                        WIDTH, HIGHT,               // Ширина и высота окна     
                        (HWND) NULL,                // Указатель на родительское окно NULL     
                        (HMENU) NULL,               // Используется меню класса окна               
                        (HINSTANCE) hInstance,      // Указатель на текущее приложение
                        (LPVOID) &wd );             // Передается в качестве lParam в событие WM_CREATE
  
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
  return int(lpMsg.wParam);
}
