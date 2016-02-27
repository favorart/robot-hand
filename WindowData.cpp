﻿#include "StdAfx.h"
#include "WindowData.h"
#include "Draw.h"
#include "littleTests.h"

using namespace std;
using namespace HandMoves;
//-------------------------------------------------------------------------------
MyWindowData:: MyWindowData (HWND hLabMAim, HWND hLabTest, HWND hLabStat) :
  pWorkerThread(nullptr), 
  hLabMAim (hLabMAim),
  hLabTest (hLabTest),
  hLabStat (hLabStat),
  target ( 32U, 32U, // (-0.39,  0.62, -0.01, -0.99);
                     //  -0.70,  0.90,  0.90, -0.99)
                      -0.41,  0.46, -0.05, -0.90
                     // -0.39, 0.41, -0.05, -0.85
                     // 200U, 200U, -1., 1., -1., 1.
          ),
  scaleLetters (target.Min (), target.Max ()),
  lt (nullptr)
{
  std::srand ((unsigned int) clock ());

  testing = false;
  reach   = false;

  working_space_show = false;
  allPointsDB_show = false;

  /* создаем ручки */
  hPen_grn  = CreatePen (PS_SOLID, 1, RGB (100, 180, 050));
  hPen_red  = CreatePen (PS_SOLID, 2, RGB (255, 000, 000));
  hPen_blue = CreatePen (PS_SOLID, 2, RGB (000, 000, 255));
  hPen_cian = CreatePen (PS_SOLID, 2, RGB (057, 168, 157));
  hPen_orng = CreatePen (PS_SOLID, 2, RGB (255, 128, 000));

  /* создаем кисти */
  hBrush_white = (HBRUSH) GetStockObject (WHITE_BRUSH);
  hBrush_null  = (HBRUSH) GetStockObject (NULL_BRUSH);
  hBrush_back  = CreateSolidBrush (RGB (235, 235, 255)
                                   /* background color */
                                   // RGB (255,204,238)
                                   );

  // p_x = 0; p_y = 0;
  // fm = 0; fn = 0;
  //=======================
  hand.SET_DEFAULT;
  WorkerThreadRunStoreTask (*this, _T ("  *** loading ***  "),
                            storeLoad, CurFileName);
  //=======================
}
MyWindowData::~MyWindowData ()
{
  if ( lt ) delete lt;
  /* очищаем ручку */
  DeleteObject (hPen_grn);
  DeleteObject (hPen_red);
  DeleteObject (hPen_blue);
  DeleteObject (hPen_cian);
  DeleteObject (hPen_orng);

  DeleteObject (hBrush_white);
  DeleteObject (hBrush_null);
  DeleteObject (hBrush_back);

  DeleteDC (hStaticDC);
  DeleteObject (hStaticBitmap);
    
  if ( pWorkerThread )
  {
    // pWorkerThread->interrupt ();
    delete pWorkerThread;
    pWorkerThread = nullptr;
  }
  //=======================
  // HandMoves::storeSave (store, CurFileName);
  //=======================
}
//-------------------------------------------------------------------------------
void  OnPaintStaticFigures (HDC hdc, MyWindowData &wd)
{
  const double CircleRadius = 0.01;
  // --------------------------------------------------------------
  DrawDecardsCoordinates (hdc);
  wd.target.draw (hdc, wd.hPen_grn);
  // --------------------------------------------------------------

  // WorkerThreadRunStoreTask (wd, _T (" *** loading ***  "),
  //                           [hdc](const HandMoves::Store &store) { /* ... */ });

  // ----- Отрисовка точек БД -------------------------------------
  if ( !wd.testing && wd.allPointsDB_show && !wd.store.empty () ) /* command  q */
  {
    HPEN hPen_old = (HPEN) SelectObject (hdc, wd.hPen_blue);

    std::unordered_map<Point, double, PointHasher>  map_points;
    for ( auto rec : wd.store )
    {
      /*  Если в одну точку попадают несколько движений -
       *  выбрать лучшее движение по элегантности */

      auto pRec = map_points.find (rec.aim);
      if ( pRec != map_points.end () )
      {
        double elegance = rec.eleganceMove ();
        if ( pRec->second < elegance )
          map_points.insert (std::make_pair(rec.aim, elegance));
      }
      else
      { map_points.insert (std::make_pair(rec.aim, rec.eleganceMove ())); }
    }

    for ( auto pt : map_points )
    { DrawCircle (hdc, pt.first, CircleRadius); }

    SelectObject (hdc, hPen_old);
  }
  // --------------------------------------------------------------
  if ( wd.working_space_show ) /* u */
    DrawTrajectory (hdc, wd.working_space, wd.hPen_orng);
  // --------------------------------------------------------------
}
void  OnPainDynamicFigures (HDC hdc, MyWindowData &wd)
{
  const double CircleRadius = 0.01;
  // --------------------------------------------------------------
  /* Target to achive */
  if ( wd.lt )
    wd.lt->draw (hdc, wd);

  // ----- Отрисовка фигуры ---------------------------------------
  wd.hand.draw (hdc, wd.hPen_red, wd.hBrush_white);
  DrawTrajectory (hdc, wd.trajectory_frames, wd.hPen_orng);
  // --------------------------------------------------------------

  if ( !wd.testing && !wd.testing_trajectories.empty () && 
        wd.testing_trajectories_animation_show )
  {
    // for ( auto t : wd.testing_trajectories )
    auto it = wd.testing_trajectories.begin ();
    for ( auto i = 0U; i < wd.testing_trajectories_animation_num_iteration; ++i )
    {
      auto itt = *it;
      DrawTrajectory (hdc, *itt.get (), wd.hPen_blue);

      DrawCircle(hdc, itt->back (), CircleRadius);
      ++it;
    } // end for
  } // end if
   
  // ----- Отрисовка точек БД -------------------------------------
  if ( !wd.adjPointsDB.empty () )
  {
    HPEN hPen_old = (HPEN) SelectObject (hdc, wd.hPen_cian);
    for ( auto p : wd.adjPointsDB )
    { DrawCircle (hdc, p->aim, CircleRadius); }
    SelectObject (hdc, hPen_old);
  }
  // --------------------------------------------------------------
  if ( wd.mouse_haved )
   DrawAdjacency (hdc, wd.mouse_aim, wd.radius, ellipse, wd.hPen_cian);
  // --------------------------------------------------------------
  if ( wd.scaleLetters.show )
   wd.scaleLetters.draw (hdc, wd.hand.jointsPositions (), &wd.hand.position);
  // --------------------------------------------------------------
}
//-------------------------------------------------------------------------------
void  WorkerThreadTryJoin (MyWindowData &wd)
{
  if ( wd.pWorkerThread && wd.pWorkerThread->try_join_for (boost::chrono::milliseconds (10)) )
  { /* joined */
    delete wd.pWorkerThread;
    wd.pWorkerThread = nullptr;

    wd.testing = false;
    wd.testing_trajectories_animation_num_iteration = 1;
    wd.testing_trajectories_animation_show = true;

    /* Set text of label 'Stat'  */
    SendMessage (wd.hLabTest, WM_SETTEXT, NULL,
                 reinterpret_cast<LPARAM> (_T (" ")) );
  } // end if
}

void  OnWindowTimer (MyWindowData &wd)
{
  if ( !wd.testing && !wd.reach )
  {
    /* Auto-drawing trajectory animation */

    /* !!! WORKING ONLY FOR NEW_HAND !!! */

    // if ( wd.trajectory_frames_show )
    // { --wd.trajectory_frames_lasts;
    //   if ( !wd.trajectory_frames_lasts )
    //   {
    //     // std::wcout << L"end " << tstring (wd.hand.position) << std::endl;
    //     wd.hand.step (wd.trajectory_frames_muscle);
    //   }
    // }

    // ============
    wd.hand.step ();
    // ============
    if ( wd.trajectory_frames_show )
    { 
      
      /* !!! WORKING ONLY FOR NEW_HAND !!! */
      if ( wd.hand.moveEnd )
      {
        auto aim = wd.hand.position;
        wd.trajectory_frames.push_back (aim);

        if ( !wd.testing )
        {
          Point  base_pos = wd.trajectory_frames.front ();
          wd.trajectory_frames.pop_front ();

          auto rec = Record (aim, base_pos, aim,
                             { wd.trajectory_frames_muscle },
                             { 0U }, { wd.trajectory_frames_lasts },
                             1U, wd.trajectory_frames);
          wd.store.insert (rec);
        }
        wd.trajectory_frames_lasts = 0U;
        wd.trajectory_frames_muscle = Hand::EmptyMov;
      }
      else if ( wd.trajectory_frames_lasts )
      { wd.trajectory_frames.push_back (wd.hand.position); }

    }

    /* Trajectoriaes animation */
    if ( wd.testing_trajectories_animation_show && 
         wd.testing_trajectories_animation_num_iteration >= 
         wd.testing_trajectories.size () )
    { ++wd.testing_trajectories_animation_num_iteration; }
  }

  else
  { WorkerThreadTryJoin (wd); }
}
void  OnWindowMouse (MyWindowData &wd)
{
  wd.mouse_haved = true;
  wd.mouse_aim = logic_coord (&wd.mouse_coords);

  /* Setting the Label's text */
  SendMessage (wd.hLabMAim,      /* Label   */
               WM_SETTEXT,       /* Message */
               (WPARAM) NULL,    /* Unused  */
               (LPARAM) tstring (wd.mouse_aim).c_str () );
  
  OnShowDBPoints (wd);
}
//-------------------------------------------------------------------------------
/* inline */ void  OnRandomTest (MyWindowData &wd)
{
  const size_t repeats = 1000U;
  WorkerThreadRunStoreTask (wd, _T ("\n *** random test ***  "),
                            HandMoves::test_random,
                            wd.hand, repeats);
  WorkerThreadTryJoin (wd);
}
/* inline */ void  OnCoverTest  (MyWindowData &wd)
{ 
  const size_t nested = 2U;
  WorkerThreadRunStoreTask (wd, _T ("\n *** cover test ***  "),
                            HandMoves::test_cover,
                            wd.hand, nested);
  WorkerThreadTryJoin (wd);
}
//-------------------------------------------------------------------------------
void  OnShowTrajectoryFrames (MyWindowData &wd)
{
  wd.hand.SET_DEFAULT;
  wd.trajectory_frames.clear ();
  wd.trajectory_frames_show = true;

  // --- 2 actions -------------------------------------
  auto muscle1 = wd.hand.selectControl ();
  auto lasts1  = random ( wd.hand.maxMuscleLast (muscle1) );
  Hand::Control  control1 (muscle1, 0, lasts1);

  auto muscle2 = wd.hand.selectControl (muscle1);
  auto lasts2  = random ( wd.hand.maxMuscleLast (muscle2) );
  Hand::Control  control2 (muscle2, random (1U, lasts1), lasts2);

  wd.trajectory_frames_lasts = max (lasts1, lasts2);
  wd.trajectory_frames_muscle = muscle1 | muscle2;

  wd.trajectory_frames.push_back (wd.hand.position);
  wd.hand.step (wd.trajectory_frames_muscle);
  wd.trajectory_frames.push_back (wd.hand.position);
}
//-------------------------------------------------------------------------------
void  MakeHandMove   (MyWindowData &wd)
{
  HandMoves::adjacencyPoints (wd.store, wd.adjPointsDB,
                              wd.mouse_aim, wd.radius);

  ClosestPredicate pred (wd.mouse_aim);
  auto it_min = std::min_element (wd.adjPointsDB.begin (),
                                  wd.adjPointsDB.end (),
                                  pred);

  if ( it_min != wd.adjPointsDB.end () )
  {
    const shared_ptr<Record> &pRec = (*it_min);
    pRec->repeatMove (wd.hand);
    wd.trajectory_frames = pRec->trajectory;
  }

  // ================================================
  // EXACT VARIANTS
  decltype(wd.adjPointsDB) range;
  HandMoves::adjacencyPoints (wd.store, range,
                              wd.mouse_aim, 0.01);
  if ( range.size () > 1U )
    for ( auto rec : range )
    {
      wd.testing_trajectories.push_back (std::make_shared<trajectory_t> (rec->trajectory));
    }

}
void  OnShowDBPoints (MyWindowData &wd)
{
  if ( !wd.testing && wd.mouse_haved )
  {
    wd.adjPointsDB.clear ();
    wd.testing_trajectories.clear ();
    wd.trajectory_frames.clear ();

    // ?? figure_t ???

    wd.reach = true;
    MakeHandMove (wd);

    // wd.testing = false;
    // typedef void  (*func_type) (MyWindowData &wd);


    // wd.pWorkerThread = new boost::thread (MakeHandMove,
    //                                       std::ref (wd));

    // typedef size_t  (*func_type) (Store&,std::list<std::shared_ptr<Record>>&,const Point&,double);
    // wd.pWorkerThread = new boost::thread ( (func_type) (&HandMoves::adjacencyPoints),
    //                                        std::ref (wd.store),
    //                                        std::ref (wd.pointsDB),
    //                                        std::ref (wd.mouse_aim),
    //                                        wd.radius );

    // HandMoves::adjacencyPoints (wd.store, std::back_inserter (wd.pointsDB),
    //                             wd.mouse_aim, wd.radius);

    // HandMoves::adjacencyRectPoints (wd.store, std::back_inserter (wd.pointsDB),
    //                                 wd.mouse_aim, wd.radius);

    // TryJoinWorkerThread (wd);

  } // end if
}
//-------------------------------------------------------------------------------
void  OnShowDBTrajectories (MyWindowData &wd)
{
  for ( auto rec : wd.adjPointsDB )
  { 
    wd.trajectoriesDB.push_back ( make_shared<trajectory_t> (rec->trajectory) );
  }
}
//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
