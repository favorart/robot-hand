#include "stdafx.h"
#include "WindowHeader.h"
#include "WindowDrawLetters.h"
//------------------------------------------------------------------------------
CanvasScaleLetters::CanvasScaleLetters (const Point &targetMin, const Point &targetMax) :
  show(false), targetMin_(targetMin), targetMax_(targetMax)
{
  double NormXScale = 30.; // ????
  double NormYScale = 30.; // ????
  double RealXScale = bg::distance(Point{ targetMin_.x, targetMax_.y }, targetMax_);
  double RealYScale = bg::distance(Point{ targetMax_.x, targetMin_.y }, targetMax_);
  RealScale = RealXScale / NormXScale; // ????

  ss_ << std::fixed << std::setprecision (2);

  ss_ << NormXScale /* ??? */ << _T (" sm");
  txtTargetScale = ss_.str ();
  ss_.str (tstring ());
  ss_.clear ();

  ss_ << RealXScale << _T (" sm");
  txtXScale = ss_.str ();
  ss_.str (tstring ());
  ss_.clear ();

  ss_ << RealYScale << _T (" sm");
  txtYScale = ss_.str ();
  ss_.str (tstring ());
  ss_.clear ();

  lf_.lfHeight = 15; // ???
  lf_.lfWeight = FW_NORMAL;
  lstrcpy (lf_.lfFaceName, _T ("Tahoma"));

  lf_.lfEscapement = 00;
  lf_.lfOrientation = 00;
  Font_000 = CreateFontIndirect (&lf_);

  lf_.lfEscapement = 900;
  lf_.lfOrientation = 900;
  Font_090 = CreateFontIndirect (&lf_);

  lf_.lfEscapement = 2700;  /* 270 degreees rotated text */
  lf_.lfOrientation = 2700;
  Font_270 = CreateFontIndirect (&lf_);
}
//------------------------------------------------------------------------------
void CanvasScaleLetters::draw(HDC hdc, const Point *jointsPoses, int jointsN, bool centerCoords)
{
    const double targetXShift = 0.1;
    Point sx, sy;
    if (centerCoords)
    {
        sx = {  0.01, 0.13 };
        sy = { -0.13, 0.01 };
    }
    else
    {
        sx = { -0.90, -0.96 };
        sy = { -0.98, -0.90 };
    }
    // SetBkMode (hdc, TRANSPARENT);

    HFONT oldFont = (HFONT)SelectObject(hdc, Font_270);
    //---draw scale on Target---
    TextOut(hdc,
            Tx(targetMax_.x + targetXShift),  /* Location of the text */
            Ty((targetMax_.y + targetMin_.y) / 2 + targetXShift),
            txtTargetScale.c_str(),                  /* Text to print */
            (int)txtTargetScale.size());          /* Size of the text */

    //!!! lf_.lfHeight = 15; // SCALE WITH WINDOW
    //---draw scales on coord plate---
    SelectObject(hdc, Font_090);
    TextOut(hdc,
            Tx(sy.x), Ty(sy.y),      /* Location of the text */
            txtXScale.c_str(),              /* Text to print */
            (int)txtXScale.size()        /* Size of the text */
    );

    SelectObject(hdc, Font_000);
    TextOut(hdc,
            Tx(sx.x), Ty(sx.y),     /* Location of the text */
            txtYScale.c_str(),               /* Text to print */
            (int)txtYScale.size()         /* Size of the text */
    );

    //---draw scale on hand joints---
    if (jointsN > 1)
    {
        for (auto *p = jointsPoses, *next = p + 1; next < (jointsPoses + jointsN); p = next, ++next)
        {
            /* Calculate value of scale */
            ss_ << (bg::distance(*p, *next) / RealScale) << _T(" sm");
            tstring strHandScale = ss_.str();
            ss_.str(tstring());
            ss_.clear();
            //---------------------------------------
            POINT pos{ (Tx((next->x + p->x) / 2) - /*90 > angle < 270*/textLength(hdc, strHandScale).cx / 2), Ty((next->y + p->y) / 2) };
            double angle = atan2((next->y - p->y), (next->x - p->x));
            angle *= 180. / M_PI;
            //---------------------------------------
            LOGFONT lf = lf_;
            lf.lfEscapement = LONG(10 * angle);
            lf.lfOrientation = LONG(10 * angle);
            HFONT Font = CreateFontIndirect(&lf);
            SelectObject(hdc, Font);
            //---------------------------------------
            TextOut(hdc,
                    pos.x, pos.y,           /* Location of the text */
                    strHandScale.c_str(),          /* Text to print */
                    (int)strHandScale.size());  /* Size of the text */
        }
    }
    //-------------------------------
    SelectObject(hdc, oldFont);
}
//------------------------------------------------------------------------------
