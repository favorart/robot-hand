#include "stdafx.h"
#include "WindowHeader.h"
#include "DrawLetters.h"
//------------------------------------------------------------------------------
CanvasScaleLetters::CanvasScaleLetters (const Point &RecTargetMinPos,
                                        const Point &RecTargetMaxPos) :
  show (false), targetMin_ (RecTargetMinPos), targetMax_ (RecTargetMaxPos)
{
  NormScale = 30.;
  RealScale = boost_distance (targetMin_, targetMax_);

  ss_ << std::fixed << std::setprecision (2);

  ss_ << NormScale << _T (" sm");
  txtTargetScale = ss_.str ();
  ss_.str (tstring ());
  ss_.clear ();

  ss_ << 0.1 * NormScale / RealScale << _T (" sm");
  txtXScale = ss_.str ();
  ss_.str (tstring ());
  ss_.clear ();

  ss_ << 0.1 * NormScale / RealScale << _T (" sm");
  txtYScale = ss_.str ();
  ss_.str (tstring ());
  ss_.clear ();

  lf_.lfHeight = 15;
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
void  CanvasScaleLetters::draw (HDC hdc,
                                const std::vector<const Point*> &handJoints,
                                const Point* handPos)
{
  const double targetXShift = 0.1;
  HFONT oldFont;
  //---------------------------------------------------------------
  // SetBkMode (hdc, TRANSPARENT);

  oldFont = (HFONT) SelectObject (hdc, Font_270);
  //---draw scale on RecTarget-------------------------------------
  TextOut (hdc,
           Tx ( targetMax_.x + targetXShift), /* Location of the text */
           Ty ((targetMax_.y + targetMin_.y) * 0.5 + targetXShift),
           txtTargetScale.c_str (),                  /* Text to print */
           (int) txtTargetScale.size ()           /* Size of the text */
           );
  //---------------------------------------------------------------
  SelectObject (hdc, oldFont);

  oldFont = (HFONT) SelectObject (hdc, Font_090);
  //---draw scale on coord plate-----------------------------------
  TextOut (hdc,
           Tx (-0.13), Ty (0.01),   /* Location of the text */
           txtXScale.c_str (),             /* Text to print */
           (int) txtXScale.size ()      /* Size of the text */
           );
  //---------------------------------------------------------------
  SelectObject (hdc, oldFont);

  oldFont = (HFONT) SelectObject (hdc, Font_000);
  //---draw scale on coord plate-----------------------------------
  TextOut (hdc,
           Tx (0.01), Ty (0.13),    /* Location of the text */
           txtYScale.c_str (),             /* Text to print */
           (int) txtYScale.size ()      /* Size of the text */
           );
  //---------------------------------------------------------------
  SelectObject (hdc, oldFont);

  //---draw scale on hand joints-----------------------------------
  if ( handJoints.size () >= 2U )
  {
    /* Clvcl, Shldr, Elbow */
    auto it = handJoints.begin (), next = it + 1;
    do
    {
      /* Calculate value of scale */
      double Scale = boost_distance (**it, **next) / (RealScale * NormScale);

      ss_ << Scale << _T (" sm");
      tstring strHandScale = ss_.str ();
      ss_.str (tstring ());
      ss_.clear ();

      /* Calculate angle */
      double angle = atan2 (((*it)->y - (*next)->y),
                            ((*it)->x - (*next)->x)) * 180. / M_PI;

      /* Calculate position */
      Point pos (((*next)->x + (*it)->x) / 2., ((*next)->y + (*it)->y) / 2.);

      LOGFONT lf = lf_;
      lf.lfEscapement = LONG (10 * angle);
      lf.lfOrientation = LONG (10 * angle);
      HFONT Font = CreateFontIndirect (&lf);

      oldFont = (HFONT) SelectObject (hdc, Font);
      //---------------------------------------
      TextOut ( hdc,
                Tx (pos.x), Ty (pos.y),  /* Location of the text */
                strHandScale.c_str (),          /* Text to print */
                (int) strHandScale.size ()   /* Size of the text */
               );
      //---------------------------------------
      SelectObject (hdc, oldFont);

      ++it; ++next;
    } while ( next != handJoints.end () );
  }
  //---------------------------------------------------------------
  if ( handPos )
  {
    ss_ << tstring (*handPos);
    tstring strHandPos = ss_.str ();
    ss_.str (tstring ());
    ss_.clear ();

    oldFont = (HFONT) SelectObject (hdc, Font_000);
    //---draw scale on coord plate-----------------------------------
    TextOut (hdc,
             Tx (handPos->x),
             Ty (handPos->y),          /* Location of the text */
             strHandPos.c_str (),             /* Text to print */
             (int) strHandPos.size ()      /* Size of the text */
             );
    //---------------------------------------------------------------
    SelectObject (hdc, oldFont);
  }
}
//------------------------------------------------------------------------------
