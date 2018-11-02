#include "stdafx.h"
#include "WindowHeader.h"

#ifndef  _DRAW_LETTERS_H_
#define  _DRAW_LETTERS_H_
//------------------------------------------------------------------------------
class CanvasScaleLetters
{
    LOGFONTW lf_ = {};
    HFONT Font_000, Font_090, Font_270;    
    Point targetMin_;
    Point targetMax_;    
    tstringstream ss_;

public:
    // ----------------------
    bool show;
    double RealScale;
    //double NormScale; // ???
    
    tstring txtTargetScale;
    tstring txtXScale;
    tstring txtYScale;
    
    // ----------------------
     CanvasScaleLetters(const Point &targetMin, const Point &targetMax);
    ~CanvasScaleLetters()
     {
         DeleteObject(Font_000);
         DeleteObject(Font_090);
         DeleteObject(Font_270);
     }
    void draw(HDC hdc, const Point *jointsCoords, int jointsN, bool centerCoords = false);
};
//------------------------------------------------------------------------------
#endif // _DRAW_LETTERS_H_
