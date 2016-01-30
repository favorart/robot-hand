#include "StdAfx.h"

// #pragma once

#ifndef  _WINDOW_DATA_H_
#define  _WINDOW_DATA_H_

#include "HandMovesStore.h"
#include "target.h"
#include "DrawLetters.h"

//------------------------------------------------------------------------------
class MyWindowData
{
public:
  HWND    hLabMAim, hLabTest, hLabStat;
  HPEN    hPen_red, hPen_grn, hPen_blue, hPen_cian, hPen_orng;
  HBRUSH  hBrush_white, hBrush_null;

  // win_point  user_coords;
  /* координаты мыши в пикселях */
  bool       mouse_haved;
  win_point  mouse_coords;
  Point      mouse_aim;

  const double  radius = 0.1;

  size_t store_size;

  // --- show_frames_trajectory ------
  std::list<Point>   trajectory_frames;
  Hand::MusclesEnum  trajectory_frames_muscle;
  Hand::time_t       trajectory_frames_lasts;
  bool               trajectory_frames_show;
  // ---------------------------------

  bool working_space_show;
  HandMoves::trajectory_t  working_space;

  // --- adjacency -------------------

  bool allPointsDB_show;

  std::list<std::shared_ptr<HandMoves::Record>>           adjPointsDB;
  std::list<std::shared_ptr<HandMoves::trajectory_t>>  trajectoriesDB;
  // ---------------------------------

  RecTarget  target;
  CanvasScaleLetters scaleLetters;
  
  // ---------------------------------
  Hand hand;
  HandMoves::Store store;

  // ---------------------------------
  bool testing;
  bool reach;

  std::list<std::shared_ptr<HandMoves::trajectory_t>>  testing_trajectories;
  bool testing_trajectories_show; // ?? CheckBox
  
  size_t  testing_trajectories_animation_num_iteration = 1;
  bool    testing_trajectories_animation_show = false;

  boost::thread *pWorkerThread;
  // ---------------------------------

  /* Набор пользователем чисел */
  // static int  flag_m, flag_n;
  // static int  p_x, p_y;
  // ---------------------------------

  MyWindowData (HWND hLabMAim, HWND hLabTest, HWND hLabStat);
 ~MyWindowData ();
};

void  OnShowTrajectory      (MyWindowData &wd);
void  OnShowDBPoints        (MyWindowData &wd);
void  OnShowDBTrajectories  (MyWindowData &wd);
//------------------------------------------------------------------------------
#endif // _WINDOW_DATA_H_