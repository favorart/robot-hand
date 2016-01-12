#include "StdAfx.h"
#include "MyWindow.h"


int WINAPI  WinMain (HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{ HWND      hWnd;  
  MSG       lpMsg;
  WNDCLASS  wc;
	TCHAR     szClassName[] = _T("Graph interface window");

  // ��������� ��������� ������ ����
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
  
  // ������������ ����� ����
  if( !RegisterClass (&wc) )
  { MessageBox (NULL, _T("Can't register window class."), _T("Error"), MB_OK);
    return 0;
  }
  
  // ������� �������� ���� ����������
  hWnd = CreateWindow ( szClassName,                // ��� ������                    
                        _T("robot-hand"),            // ����� ��������� 
                        WS_OVERLAPPEDWINDOW,        // ����� ����                                              
                        0, 0,											 // ������� ������ �������� ����   
                        WIDTH, HIGHT,               // ������ � ������ ����     
                        (HWND) NULL,                // ��������� �� ������������ ���� NULL     
                        (HMENU) NULL,               // ������������ ���� ������ ����               
                        (HINSTANCE) hInstance,      // ��������� �� ������� ����������
                        NULL );					           // ���������� � �������� lParam � ������� WM_CREATE
  
  if( !hWnd ) 
  { MessageBox (NULL, _T("Can't create main window!"), _T("Error"), MB_OK);  
	  return 0;
  }
  
  // ���������� ���� ����
  ShowWindow   (hWnd, nCmdShow); 
  UpdateWindow (hWnd);
  
  // ��������� ���� ��������� ��������� �� �������� ����������
  while ( GetMessage (&lpMsg, NULL, 0, 0) )  
  { TranslateMessage (&lpMsg);
     DispatchMessage (&lpMsg);
  }
  
  return (lpMsg.wParam);
}
