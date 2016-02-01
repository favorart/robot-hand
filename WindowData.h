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

  /* Набор пользователем чисел */
  // static int  flag_m, flag_n;
  // static int  p_x, p_y;
  // ---------------------------------

  MyWindowData (HWND hLabMAim, HWND hLabTest, HWND hLabStat);
 ~MyWindowData ();
};

// template<typename Function, typename... Args>
// inline void  WorkerThreadRunTask (MyWindowData &wd, tstring message, HandMoves::Store &store, /* std::function<void()> task */ Function task, Args... args)
// {
//   if ( !wd.testing && !wd.pWorkerThread )
//   {
//     wd.testing = true;
//     /* Setting the Label's text */
//     SendMessage (wd.hLabTest,      /* Label   */
//                  WM_SETTEXT,       /* Message */
//                  (WPARAM) NULL,    /* Unused  */
//                  (LPARAM) message.c_str ());
// 
//     wd.pWorkerThread = new boost::thread (task, args...);
//   }
// }
inline void  WorkerThreadRunStore (MyWindowData &wd, tstring message,
                                   std::function<void (HandMoves::Store&, tstring)> storeSerialize,
                                   tstring FileName)
{
  if ( !wd.testing && !wd.pWorkerThread )
  {
    wd.testing = true;
    /* Setting the Label's text */
    SendMessage (wd.hLabTest,      /* Label   */
                 WM_SETTEXT,       /* Message */
                 (WPARAM) NULL,    /* Unused  */
                 (LPARAM) message.c_str ());

    wd.pWorkerThread = new boost::thread (storeSerialize, std::ref (wd.store), FileName);
  } // end if
}
inline void  WorkerThreadRunStoreO (MyWindowData &wd, tstring message,
                                    std::function<void (HandMoves::Store&, tstring, tstring)> storeSerialize,
                                    tstring FileName, tstring DefaultName)
{
  if ( !wd.testing && !wd.pWorkerThread )
  {
    wd.testing = true;
    /* Setting the Label's text */
    SendMessage (wd.hLabTest,      /* Label   */
                 WM_SETTEXT,       /* Message */
                 (WPARAM) NULL,    /* Unused  */
                 (LPARAM) message.c_str ());

    wd.pWorkerThread = new boost::thread (storeSerialize, std::ref (wd.store), FileName, DefaultName);
  } // end if
}
inline void  WorkerThreadRunTest (MyWindowData &wd, tstring message, 
                                  std::function<void (HandMoves::Store&, Hand&, size_t)> testFunction,
                                  size_t param)
{
  if ( !wd.testing && !wd.pWorkerThread )
  {
    wd.testing = true;
    /* Setting the Label's text */
    SendMessage (wd.hLabTest,      /* Label   */
                 WM_SETTEXT,       /* Message */
                 (WPARAM) NULL,    /* Unused  */
                 (LPARAM) message.c_str ());

    wd.pWorkerThread = new boost::thread (testFunction, std::ref (wd.store), wd.hand, param);
  }
}

void  WorkerThreadTryJoin (MyWindowData &wd);

void  OnShowTrajectory      (MyWindowData &wd);
void  OnShowDBPoints        (MyWindowData &wd);
void  OnShowDBTrajectories  (MyWindowData &wd);
//------------------------------------------------------------------------------
#endif // _WINDOW_DATA_H_