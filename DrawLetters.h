#include "stdafx.h"
#include "MyWindow.h"

#ifndef  _DRAW_LETTERS_H_
#define  _DRAW_LETTERS_H_
//------------------------------------------------------------------------------
class CanvasScaleLetters
{
  LOGFONTW lf_ = {};
  HFONT Font_000, Font_090, Font_270;

  Point targetMin_;
  Point targetMax_;

  std::wstringstream ss_;

public:
  
  bool show;

  double RealScale;
  double NormScale;

  std::wstring txtTargetScale;
  std::wstring txtXScale;
  std::wstring txtYScale;

   CanvasScaleLetters (const Point &RecTargetMinPos,
                       const Point &RecTargetMaxPos);
  ~CanvasScaleLetters ()
  { DeleteObject (Font_000);
    DeleteObject (Font_090);
    DeleteObject (Font_270);
  }

  void  draw (HDC hdc, const std::vector<const Point*> &handJoints, const Point* handPos=nullptr);
};
//------------------------------------------------------------------------------
#endif // _DRAW_LETTERS_H_
