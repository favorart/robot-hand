#include "stdafx.h"

#include "MyWindow.h"
#include "WindowData.h"
#include "Draw.h"

#ifndef _LTL_TESTS_H_
#define _LTL_TESTS_H_
// ------------------------------------------------------------------------------------
class LittleTest
{
  Point                                           aim;
  std::list<std::list<std::shared_ptr<Point>>> traj_s;
  std::list<std::shared_ptr<Point>>              pt_s;

  std::list<std::shared_ptr<std::list<Point>>> traj_s1;
  double radius;

public:
  LittleTest (const Point &aim, double radius=0.007) :
    aim (aim), radius (radius) {}

  void appendPts (std::list<std::shared_ptr<HandMoves::Record>> &range, bool append_traj)
  {
    for ( auto rec : range )
    {
      pt_s.push_back (std::make_shared<Point> (rec->aim));
      if ( append_traj )
        traj_s1.push_back (std::make_shared<HandMoves::trajectory_t> (rec->trajectory));
    }
  }
  void appendTraj (const std::list<std::shared_ptr<Point>> &traj)
  {
    traj_s.push_back (traj);
  }
  void appendTraj (const std::list<Point> &traj)
  {
    std::list<std::shared_ptr<Point>> nTraj;
    for ( auto pt : traj )
      nTraj.push_back (std::make_shared<Point> (pt));
    traj_s.push_back (nTraj);
  }

  void  draw (HDC hdc, MyWindowData &wd) const;
};
// ------------------------------------------------------------------------------------
#endif // _LTL_TESTS_H_
