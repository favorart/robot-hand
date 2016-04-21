#include "StdAfx.h"

#ifndef  _WINDOW_DATA_H_
#define  _WINDOW_DATA_H_

#include "Store.h"
#include "Target.h"
#include "DrawLetters.h"

// ====================================================
// !?!?!?!?!?! КОНФИГУРАЦИОННЫЕ ФАЙЛЫ !?!?!?!?
// !?!?!?!?!?! ФАЙЛЫ, как интерфейс управления !?!?!?!?
//------------------------------------------------------------------------------
class MyWindowData
{
  // ---------------------------------
public:
  HWND    hLabMAim, hLabTest, hLabStat;
  HPEN    hPen_red, hPen_grn, hPen_blue, hPen_cian, hPen_orng;
  HBRUSH  hBrush_white, hBrush_null, hBrush_back;

  // ---------------------------------
  HDC     hStaticDC = NULL;
  HBITMAP hStaticBitmap = NULL;
  bool    hStaticBitmapChanged = true;  
  // ---------------------------------
  /* координаты мыши в пикселях */
  bool       mouse_haved;
  win_point  mouse_coords;
  Point      mouse_aim;
    
  // ---------------------------------

  const double  radius = 0.05;
  const size_t  skip_show_steps = 15U;

  tstring  CurFileName = tstring(HAND_NAME) + tstring (_T ("_moves.bin"));
  size_t   store_size;

  // --- show_frames_trajectory ------

  HandMoves::trajectory_t   trajectory_frames;
  Hand::MusclesEnum         trajectory_frames_muscle;
  Hand::time_t              trajectory_frames_lasts;
  bool                      trajectory_frames_show;
  // ---------------------------------

  bool  working_space_show;
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

  // !!!!!!!!!!!! ЗАМЕНИЛЛЛЛЛ !!!!! проверить везде
  std::list<HandMoves::trajectory_t>  testing_trajectories;
  bool testing_trajectories_show; // ?? CheckBox
  
  size_t  testing_trajectories_animation_num_iteration = 1;
  bool    testing_trajectories_animation_show = false;

  boost::thread *pWorkerThread;
  // ---------------------------------
  // LittleTest *lt;
  // size_t no;
  // ---------------------------------

  std::list<Point> uncoveredPoints;
  bool             uncovered_show = true;

  Positions::DirectionPredictor  pd;

  /* Набор пользователем чисел */
  // static int  flag_m, flag_n;
  // static int  p_x, p_y;
  // ---------------------------------
  MyWindowData ();
 ~MyWindowData ();
};
//-------------------------------------------------------------------------------
template<typename Function, typename... Args>
inline void  WorkerThreadRunStoreTask (MyWindowData &wd, tstring message, Function task, Args... args)
{
  if ( !wd.testing && !wd.pWorkerThread )
  {
    wd.testing = true;
    /* Set text of label 'Stat'  */
    SendMessage (wd.hLabTest, WM_SETTEXT, NULL,
                 reinterpret_cast<LPARAM> (message.c_str ()));

    wd.pWorkerThread = new boost::thread (task, std::ref (wd.store), args...);
  }
}
       void  WorkerThreadTryJoin      (MyWindowData &wd);
//-------------------------------------------------------------------------------
void  OnPaintStaticBckGrnd (HDC hdc, MyWindowData &wd);
void  OnPaintStaticFigures (HDC hdc, MyWindowData &wd);
void  OnPainDynamicFigures (HDC hdc, MyWindowData &wd);
//-------------------------------------------------------------------------------
void  OnWindowTimer (MyWindowData &wd);
void  OnWindowMouse (MyWindowData &wd);
//-------------------------------------------------------------------------------
void  OnShowTrajectoryFrames (MyWindowData &wd);
//-------------------------------------------------------------------------------
void  OnShowDBPoints (MyWindowData &wd);
// void  OnShowDBTrajectories (MyWindowData &wd);
//------------------------------------------------------------------------------
#endif // _WINDOW_DATA_H_