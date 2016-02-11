#include "StdAfx.h"
#include "HandMotionLaw.h"

//------------------------------------------------------------------------------
std::vector<double>  NewHand::MotionLaws::generateJointMoveFrames (double a, double b, size_t n)
{
  std::vector<double> frames (n--);

  double  sum = 0.;
  for ( size_t i = 0U; i <= n; ++i )
  { /* diff velosity on start and end */
    frames[i] = (1. - cos (i * M_PI / n)) * 0.5;
    sum += frames[i]; /* calc normalization */
  }

  /* apply normalization */
  for ( size_t i = 0U; i <= n; ++i )
  { frames[i] = frames[i] * (b - a) / sum + a; }

  return frames;
}
std::vector<double>  NewHand::MotionLaws::generateJointStopFrames (double a, double b, size_t n)
    {
      std::vector<double> frames (n--);

      double  sum = 0.;
      for ( size_t i = 0U; i <= n; ++i )
      { /* diff velosity on start and end */
        // frames[i] = (a + b + (a - b)*cos ( ((n / 2. + (i+1) / 2.) * M_PI) / n)) / 2.;
        double d = double (i) / n;
        frames[i] = (1. - cos ((d + 1.) * M_PI)) * 0.5;
        sum += frames[i];  /* calc normalization */
      }
      /* apply normalization */
      for ( size_t i = 0U; i <= n; ++i )
      { frames[i] = frames[i] * (b - a) / sum + a; }

      return frames;
    }
//------------------------------------------------------------------------------