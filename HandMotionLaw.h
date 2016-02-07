#include "StdAfx.h"


#ifndef  _MOTION_LAWS_H_
#define  _MOTION_LAWS_H_
//------------------------------------------------------------------------------
namespace NewHand
{
  namespace MotionLaws
  {
    typedef std::function<std::vector<double> (double, double, size_t)> MotionLaw;

    std::vector<double>  generateJointMoveFrames (double left_border, double right_border, size_t frames_count);
    std::vector<double>  generateJointStopFrames (double left_border, double right_border, size_t frames_count);
  }
}
//------------------------------------------------------------------------------
#endif // _MOTION_LAWS_H_