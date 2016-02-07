#include "StdAfx.h"

// #pragma once

#ifndef  _WINDOW_DATA_H_
#define  _WINDOW_DATA_H_

#include "HandMovesStore.h"
#include "target.h"
#include "DrawLetters.h"
#include "littleTests.h"

void  /*HandMoves::*/ testLittleCorrectives (HandMoves::Store &store, Hand &hand, RecTarget &target,
                                             double radius, /* minimal distance between 2 neighbour points of target */
                                             double epsilont = EPS);
void littleTest (MyWindowData &wd, double radius);
//------------------------------------------------------------------------------
class MyWindowData
{
public:
  HWND    hLabMAim, hLabTest, hLabStat;
  HPEN    hPen_red, hPen_grn, hPen_blue, hPen_cian, hPen_orng;
  HBRUSH  hBrush_white, hBrush_null, hBrush_back;

  HBITMAP hStaticBitmap = NULL;
  bool    hStaticBitmapChanged = true;
  
  HDC hDC = NULL;
  HDC hStaticDC = NULL;

  // win_point  user_coords;
  /* ���������� ���� � �������� */
  bool       mouse_haved;
  win_point  mouse_coords;
  Point      mouse_aim;

  const double  radius = 0.1;

  tstring  CurFileName = tstring(HAND_NAME) + tstring (_T ("_moves.bin"));

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

  LittleTest *lt;
  size_t no;

  /* ����� ������������� ����� */
  // static int  flag_m, flag_n;
  // static int  p_x, p_y;
  // ---------------------------------

  MyWindowData (HWND hLabMAim, HWND hLabTest, HWND hLabStat);
 ~MyWindowData ();
};
//-------------------------------------------------------------------------------
template<typename Function, typename... Args>
inline void  WorkerThreadRunStoreTask (MyWindowData &wd, tstring message, Function task, Args... args)
{
  if ( !wd.testing && !wd.pWorkerThread )
  {
    wd.testing = true;
    /* Setting the Label's text */
    SendMessage (wd.hLabTest,      /* Label   */
                 WM_SETTEXT,       /* Message */
                 (WPARAM) NULL,    /* Unused  */
                 (LPARAM) message.c_str ());

    wd.pWorkerThread = new boost::thread (task, std::ref (wd.store), args...);
  }
}
       void  WorkerThreadTryJoin      (MyWindowData &wd);
//-------------------------------------------------------------------------------
void  OnPaintStaticFigures (HDC hdc, MyWindowData &wd);
void  OnPainDynamicFigures (HDC hdc, MyWindowData &wd);
//-------------------------------------------------------------------------------
void  OnWindowTimer (MyWindowData &wd);
void  OnWindowMouse (MyWindowData &wd);
//-------------------------------------------------------------------------------
/* inline */ void  OnRandomTest (MyWindowData &wd);
/* inline */ void  OnCoverTest (MyWindowData &wd);
//-------------------------------------------------------------------------------
void  OnShowTrajectoryFrames (MyWindowData &wd);
//-------------------------------------------------------------------------------
void  OnShowDBPoints (MyWindowData &wd);
void  OnShowDBTrajectories (MyWindowData &wd);
//------------------------------------------------------------------------------
#endif // _WINDOW_DATA_H_