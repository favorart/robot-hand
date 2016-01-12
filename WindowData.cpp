#include "StdAfx.h"
#include "WindowData.h"
#include "Draw.h"

// #include <thread>
// #include <atomic>
// #include <chrono>
#include <boost/thread.hpp>

using namespace std;
using namespace HandMoves;
//-------------------------------------------------------------------------------
MyWindowData:: MyWindowData () : target (35U, 30U, // (-0.39, 0.62, -0.01, -0.99);
                                                      -0.70, 0.90,  0.90, -0.99)
{
  std::srand ((ulong_t) std::clock ());

  testing = false;

  /* создаем ручки */
  hPen_grn  = CreatePen (PS_SOLID, 1, RGB (100, 180, 050));
  hPen_red  = CreatePen (PS_SOLID, 2, RGB (255, 000, 000));
  hPen_blue = CreatePen (PS_SOLID, 2, RGB (000, 000, 255));
  hPen_cian = CreatePen (PS_SOLID, 2, RGB (057, 168, 157));

  /* создаем кисти */
  hBrush_white = (HBRUSH) GetStockObject (WHITE_BRUSH);
  hBrush_null  = (HBRUSH) GetStockObject (NULL_BRUSH);

  // p_x = 0; p_y = 0;
  // fm = 0; fn = 0;
  //=======================
  hand.set (Hand::Clvcl | Hand::Shldr | Hand::Elbow, { 50., 50., 50. });
  HandMoves::storeLoad (store);
  //=======================
}
MyWindowData::~MyWindowData ()
{
  /* очищаем ручку */
  DeleteObject (hPen_grn);
  DeleteObject (hPen_red);
  DeleteObject (hPen_blue);
  DeleteObject (hPen_cian);

  DeleteObject (hBrush_white);
  DeleteObject (hBrush_null);
  //=======================
  HandMoves::storeSave (store);
  //=======================
}
//-------------------------------------------------------------------------------
void  OnPaintMyLogic (HDC hdc, MyWindowData &wd)
{
  // ----- Отрисовка фигуры ------------------------
  wd.hand.draw (hdc, wd.hPen_red, wd.hBrush_white);
  draw_trajectory (hdc, wd.trajectory_frames, wd.hPen_cian);

  // if ( testing_trajectories_show )
  if ( !wd.testing && !wd.testing_trajectories.empty () )
  {
    int i = 0;

    // for ( auto t : wd.testing_trajectories )
    auto it = wd.testing_trajectories.begin ();

    for ( auto i = 0U; i < wd.testing_trajectories.size (); ++i )
    {
      // if ( i == iter ) break;
      draw_trajectory (hdc, (*(*it)), wd.hPen_blue);

      auto fin = (*(*it)).back ();
      Ellipse (hdc, Tx (-0.01 + fin.x), Ty (0.01 + fin.y),
               Tx (0.01 + fin.x), Ty (-0.01 + fin.y));
      ++it;
    } // end for
  } // end if

  if ( wd.mouse_haved )
  {
    draw_adjacency (hdc, wd.mouse_aim, wd.radius, ellipse, wd.hPen_cian);
  }

  // --------------------------------------------------------------
  HPEN hPen_old = (HPEN) SelectObject (hdc, wd.hPen_cian);
  for ( auto p : wd.pointsDB )
  {
    Ellipse (hdc, Tx (-0.01 + p->aim.x), Ty ( 0.01 + p->aim.y),
                  Tx ( 0.01 + p->aim.x), Ty (-0.01 + p->aim.y));
    // SetPixel (hdc, Tx (p->aim.x), Ty (p->aim.y), RGB (000, 000, 255));
  }
  SelectObject (hdc, hPen_old);
  // --------------------------------------------------------------
}
//-------------------------------------------------------------------------------
void  OnWindowTimer (MyWindowData &wd)
{
  if ( !wd.testing )
  {
    if ( wd.trajectory_frames_show )
    {
      --wd.trajectory_frames_lasts;
      if ( !wd.trajectory_frames_lasts )
        wd.hand.step (wd.trajectory_frames_muscle);
    }
    // ============
    wd.hand.step ();
    // ============
    if ( wd.trajectory_frames_show )
      wd.trajectory_frames.push_back (wd.hand.position);
  }

  // if ( show )
  // {
  //   ++iter;
  // }
}
void  OnWindowMouse (MyWindowData &wd)
{
  wd.mouse_haved = true;
  wd.mouse_aim = logic_coord (&wd.mouse_coords);
  
  OnShowDBPoints (wd);

  // HandMoves::adjacencyRectPoints<decltype(found_points)>
  //  (store,  std::back_inserter (found_points), { 0.1, 0.1 }, { 0.4, 0.3 });

  //HandMoves::adjacencyPoints //<decltype(found_points)>
  //  (store, std::back_inserter (found_points), aim, radius, true);
}
//-------------------------------------------------------------------------------
void  OnRandomTest (MyWindowData &wd)
{
  // wd.hand.set (Hand::Clvcl | Hand::Shldr | Hand::Elbow, { 50., 50., 50. });

  // ??? Thread
  if ( !wd.testing )
  {
    wd.testing = true;

    boost::thread  WorkingThread (HandMoves::test_random,
                                  std::ref (wd.store),
                                  wd.hand,
                                  1000U);

    // test_random (wd.store, wd.hand, 1000U);

    WorkingThread.join ();
    wd.testing = false;
  }

  
}
void  OnCoverTest  (MyWindowData &wd)
{
  //iter = 1;
  if ( !wd.testing )
  {
    wd.testing = true;
    wd.testing_trajectories.clear ();

    // boost::thread  WorkingThread (HandMoves::test_cover,
    //                               std::ref (wd.store),
    //                               wd.hand,
    //                               std::ref (wd.testing_trajectories),
    //                               2);

    // hand.set (Hand::Clvcl | Hand::Shldr | Hand::Elbow, { 50., 50., 50. });
    // HandMoves::test_cover (wd.store, wd.hand, wd.testing_trajectories, 2);

    // if ( WorkingThread.try_join_for (boost::chrono::milliseconds (10)) )
    {
      wd.testing = false;
      wd.testing_trajectories_animation_num_iteration = 1;
      wd.testing_trajectories_animation_show = true;
    } // end if
  } // end if
}
//-------------------------------------------------------------------------------
void  OnShowTrajectory (MyWindowData &wd)
{
  wd.hand.set (Hand::Clvcl | Hand::Shldr | Hand::Elbow, { 50., 50., 50. });
  wd.trajectory_frames.clear ();

  wd.trajectory_frames_show = true;
  wd.trajectory_frames_muscle = selectHandMove ( random (HandMovesCount) );
  wd.trajectory_frames_lasts = random ( wd.hand.timeMuscleWorking (wd.trajectory_frames_muscle) );

  wd.hand.step (wd.trajectory_frames_muscle);
  wd.trajectory_frames.push_back (wd.hand.position);
}
//-------------------------------------------------------------------------------
void  draw_adjacency (HDC hdc, const Point &pt, double r, figure_t figure, HPEN hPen_cian)
{
  HPEN hPen_old = (HPEN) SelectObject (hdc, hPen_cian);
  switch ( figure )
  {
    case ellipse:     Ellipse (hdc, Tx(pt.x - r), Ty(pt.y + r),
                                    Tx(pt.x + r), Ty(pt.y - r)); break;
    case rectangle: Rectangle (hdc, Tx(pt.x - r), Ty(pt.y + r),
                                    Tx(pt.x + r), Ty(pt.y - r)); break;
  }
  SelectObject (hdc, hPen_old);
}

void  OnShowDBPoints (MyWindowData &wd)
{
  if ( !wd.testing )
  {
    // boost::thread workerThread (workerFunc);
    if ( wd.mouse_haved )
    {
      wd.pointsDB.clear ();
      HandMoves::adjacencyPoints (wd.store, wd.pointsDB,
                                  wd.mouse_aim, wd.radius);

      // ?? figure_t
      // HandMoves::adjacencyRectPoints (wd.store, std::back_inserter (wd.pointsDB),
      //                                 wd.mouse_aim, wd.radius, true);
    }
    else
    {
      for ( auto rec : wd.store )
      { wd.pointsDB.push_back (make_shared<Record> (rec)); }
    } // end else
  }
}
//-------------------------------------------------------------------------------
void  OnShowDBTrajectories (MyWindowData &wd)
{
  for ( auto rec : wd.pointsDB )
  { 
    wd.trajectoriesDB.push_back ( make_shared<trajectory_t> (rec->trajectory) );
  }
}
//-------------------------------------------------------------------------------
