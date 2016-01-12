#include "StdAfx.h"

// #pragma once

#ifndef  _WINDOW_DATA_H_
#define  _WINDOW_DATA_H_

#include "MyWindow.h"
#include "Hand.h"
#include "HandMovesStore.h"
#include "target.h"

//------------------------------------------------------------------------------
class MyWindowData
{
public:
  HPEN   hPen_red, hPen_grn, hPen_blue, hPen_cian;
  HBRUSH hBrush_white, hBrush_null;

  win_point  user_coords;
  /* координаты мыши в пикселях */
  bool      mouse_haved;
  win_point mouse_coords;
  Point     mouse_aim;

  const double radius = 0.1;

  // --- show_frames_trajectory ------
  std::list<Point>   trajectory_frames;
  Hand::MusclesEnum  trajectory_frames_muscle;
  Hand::time_t       trajectory_frames_lasts;
  bool               trajectory_frames_show;
  // ---------------------------------

  // --- adjacency -------------------
  std::list<std::shared_ptr<HandMoves::Record>> pointsDB;
  std::list<std::shared_ptr<HandMoves::trajectory_t>> trajectoriesDB;
  // ---------------------------------

  RecTarget  target;

  Hand hand;
  HandMoves::Store store;

  // ---------------------------------
  bool testing;

  std::list<std::shared_ptr<HandMoves::trajectory_t>>  testing_trajectories;
  bool testing_trajectories_show; // ?? CheckBox
  
  size_t  testing_trajectories_animation_num_iteration = 1;
  bool    testing_trajectories_animation_show = false;
  // ---------------------------------

  // boost::thread WorkerThread;

  /* Набор пользователем чисел */
  // static int  flag_m, flag_n;
  // static int  p_x, p_y;

  const uint_t  targetRowsCount = 35U;
  const uint_t  targetColsCount = 30U;

  MyWindowData ();
 ~MyWindowData ();
};

void  OnShowTrajectory (MyWindowData &wd);
void  OnShowDBPoints (MyWindowData &wd);
void  OnShowDBTrajectories (MyWindowData &wd);

typedef enum { ellipse = 1, rectangle } figure_t;
void  draw_adjacency (HDC hdc, const Point &pt, double r, figure_t figure, HPEN hPen_cian);

//------------------------------------------------------------------------------
#endif // _WINDOW_DATA_H_