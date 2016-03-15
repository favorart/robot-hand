#include "StdAfx.h"
#include "HandMotionLaw.h"


const double acceler_percents = 0.45;
//------------------------------------------------------------------------------
std::vector<double>  NewHand::MotionLaws::generateJointMoveFrames (double a, double b, size_t n)
{
  std::vector<double> frames (n--);
  //------------------------------------------------------
  size_t  m = static_cast<size_t> (n * acceler_percents) + 1U;
  //------------------------------------------------------
  double  norm_sum = 0.;
  for ( size_t i = 0U; i < m; ++i )
  { /* diff velosity on start and end */
    frames[i] = (1. - cos (i * M_PI / m)) * 0.5;
    /* calculating a normalization */
    norm_sum += frames[i];
  }
  for ( size_t i = m; i <= n; ++i )
  { /* constant after getting the full speed */
    frames[i] = 1.;
    /* continue calculating a normalization */
    norm_sum += frames[i];
  }
  //------------------------------------------------------
  // double sum1 = 0.;
  for ( size_t i = 0U; i <= n; ++i )
  { /* apply the normalization */
    frames[i] = frames[i] * (b - (n + 1) * a) / norm_sum + a;
    // sum1 += frames[i];
  }
  //------------------------------------------------------
  return frames;
}
std::vector<double>  NewHand::MotionLaws::generateJointStopFrames (double a, double b, size_t n)
{
  std::vector<double> frames (n--);
  //------------------------------------------------------
  double  sum = 0.;
  for ( size_t i = 0U; i <= n; ++i )
  { /* diff velosity on start and end */
    // frames[i] = (a + b + (a - b) * cos ( ((n / 2. + (i + 1) / 2.) * M_PI) / n)) / 2.;
    double d = double (i) / n;
    frames[i] = (1. - cos ((d + 1.) * M_PI)) * 0.5;
    /* calc normalization */
    sum += frames[i];
  }
  /* apply normalization */
  for ( size_t i = 0U; i <= n; ++i )
  { frames[i] = frames[i] * (b - (n + 1) * a) / sum + a; }
  //------------------------------------------------------
  return frames;
}

std::vector<double>  NewHand::MotionLaws::generateJointMoveFrames_cos (double a, double b, size_t n)
{
  std::vector<double> frames (n--);
  //------------------------------------------------------
  double  norm_sum = 0.;
  for ( size_t i = 0U; i <= n; ++i )
  { /* diff velosity on start and end */
    frames[i] = (1. - cos (i * M_PI / n)) * 0.5;
    /* calc normalization */
    norm_sum += frames[i];
  }
  /* apply normalization */
  for ( size_t i = 0U; i <= n; ++i )
  { frames[i] = frames[i] * (b - (n + 1) * a) / norm_sum + a; }
  //------------------------------------------------------
  return frames;
}
//------------------------------------------------------------------------------