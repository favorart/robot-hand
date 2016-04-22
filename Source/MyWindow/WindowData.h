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
// #define _TESTING_TRAJECTORIES_ANIMATION_ 1
// #define _DRAW_STORE_GRADIENT_
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
  bool       mouse_haved = false;
  win_point  mouse_coords;
  Point      mouse_aim;
  // ---------------------------------
  const double  radius = 0.05;
  const double  side   = 0.001;

  // ---------------------------------
  boost::thread  *pWorkerThread;
  // ---------------------------------
  bool testing;

  // --- show_frames_trajectory ------
  class TrajectoryFrames
  {
    bool                      show_;
    HandMoves::trajectory_t   trajectory_;
    // ---------------------------------
    const size_t  skip_show_steps = 15U;
    // ---------------------------------
    boost::optional<HandMoves::controling_t>         controls_;
    boost::optional<Hand::frames_t>                      time_;
    boost::optional<HandMoves::controling_t::iterator>   iter_;
    // ---------------------------------

  public:
    // ---------------------------------
    bool animation = true;
    
    // ---------------------------------
    /* Microsoft specific: C++ properties */
    __declspec(property(get=_get_traj, put=_put_traj)) const HandMoves::trajectory_t &trajectory;
    const HandMoves::trajectory_t& _get_traj () const { return trajectory_; }
    void _put_traj (const HandMoves::trajectory_t& traj) { show_ = true; trajectory_ = traj; }

    // ---------------------------------
    void  step (HandMoves::Store &store, Hand &hand,
                const boost::optional<HandMoves::controling_t> controls=boost::none);
    void  draw (HDC hdc, HPEN hPen) const;
    void  clear ();
    // ---------------------------------

  } trajectory_frames;
  // ---------------------------------

  bool                      working_space_show;
  HandMoves::trajectory_t   working_space;

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
  size_t   store_size;
  tstring  CurFileName = tstring (HAND_NAME) + tstring (_T ("_moves.bin"));

  // --------------------------------

  HandMoves::trajectories_t  testing_trajectories;
  bool                       testing_trajectories_show;

#ifdef    _TESTING_TRAJECTORIES_ANIMATION_
  size_t  testing_trajectories_animation_num_iteration = 1;
  bool    testing_trajectories_animation_show = false;
#endif // _TESTING_TRAJECTORIES_ANIMATION_

  // ---------------------------------
  // LittleTest *lt;
  // size_t no;
  // ---------------------------------

  std::list<Point> uncoveredPoints;
  bool             uncovered_show = true;

  /* Набор пользователем чисел */
  // static int  flag_m, flag_n;
  // static int  p_x, p_y;
  // ---------------------------------
  MyWindowData ();
 ~MyWindowData ();
};
//-------------------------------------------------------------------------------
template<typename Function, typename... Args> inline
void  WorkerThreadRunStoreTask (MyWindowData &wd, tstring message, Function task, Args... args)
{
  if ( !wd.testing && !wd.pWorkerThread )
  {
    wd.testing = true;
    /* Set text of label 'Stat'  */
    SendMessage (wd.hLabTest, WM_SETTEXT, NULL, reinterpret_cast<LPARAM> (message.c_str ()));

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
void  OnShowDBPoints (MyWindowData &wd);
void  OnShowDBTrajes (MyWindowData &wd);
//------------------------------------------------------------------------------
#endif // _WINDOW_DATA_H_