﻿#include "StdAfx.h"
#include "WindowData.h"
#include "Draw.h"

extern bool zoom;

using namespace std;
using namespace HandMoves;
//-------------------------------------------------------------------------------
MyWindowData:: MyWindowData () :
  pWorkerThread (NULL),
  // lt (NULL),
  target ( // 200U, 200U,
           18U, 18U,
           // 32U, 32U, // (-0.39,  0.62, -0.01, -0.99);
                        //  -0.70,  0.90,  0.90, -0.99)
                         -0.41,  0.46, -0.05, -0.90
                        // -0.39, 0.41, -0.05, -0.85
          ),
  scaleLetters ((target.min) (), (target.max) ()),
  pd (hand)
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
  hPen_cian = CreatePen (PS_SOLID, 1, RGB (057, 168, 157));
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
}
MyWindowData::~MyWindowData ()
{
  // if ( lt ) delete lt;
  //=======================
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

  //=======================    
  if ( pWorkerThread )
  {
    pWorkerThread->interrupt ();

    delete pWorkerThread;
    pWorkerThread = NULL;
  }
  //=======================
  // store.save (CurFileName);
  //=======================
}
//-------------------------------------------------------------------------------
void  OnPaintStaticBckGrnd (HDC hdc, MyWindowData &wd)
{
  DrawDecardsCoordinates (hdc);
  wd.target.draw (hdc, wd.hPen_grn,
                  /* false, */ true,
                  // false, false);
                  true, false);
}
void  OnPaintStaticFigures (HDC hdc, MyWindowData &wd)
{
  // --------------------------------------------------------------
  if ( wd.working_space_show ) /* u */
    DrawTrajectory (hdc, wd.working_space, wd.hPen_orng);
  // ----- Отрисовка точек БД -------------------------------------
  if ( !wd.testing && wd.allPointsDB_show && !wd.store.empty () )
  { /* command  <q>  */

    // size_t colorGradations = 15U;
    // color_interval_t colors = // make_pair(RGB(0,0,130), RGB(255,0,0)); // 128
    //   make_pair (RGB (150, 10, 245), RGB (245, 10, 150));
    // // make_pair(RGB(130,0,0), RGB(255,155,155));
    // 
    // gradient_t  gradient;
    // MakeGradient (colors, colorGradations, gradient);
    // // wd.store.draw (hdc, gradient);

    gradient_t  gradient ({ RGB (25, 255, 25),
                            RGB (25, 25, 255),
                            // RGB (0, 0, 255),
                            RGB (255, 25, 25) // ,
                            // RGB (255, 0, 0)
                          });

    WorkerThreadRunStoreTask ( wd, _T (" *** drawing ***  "),
                               [hdc](HandMoves::Store &store,
                                     gradient_t gradient, double r,
                                     HandMoves::trajectory_t uncoveredPoints,
                                     HPEN hPen)
                               { 
                                 store.draw (hdc, gradient, r);

                                 for ( auto &pt : uncoveredPoints )
                                   if ( zoom )
                                   { DrawCircle (hdc, pt, 0.005, hPen); }
                                   else
                                   { SetPixel (hdc, Tx (pt.x), Ty (pt.y), RGB (255, 0, 0)); }
                               },
                              gradient, /* (zoom) ? 0.0005 : */ 0.,
                              ( wd.uncovered_show ) ?  wd.uncoveredPoints : HandMoves::trajectory_t(),
                              wd.hPen_red);
  }
  // --------------------------------------------------------------
}
void  OnPainDynamicFigures (HDC hdc, MyWindowData &wd)
{
  const double CircleRadius = 0.01;
  // --------------------------------------------------------------
  /* Target to achive */
  // if ( wd.lt )  wd.lt->draw (hdc, wd);

  // ----- Отрисовка фигуры ---------------------------------------
  wd.hand.draw (hdc, wd.hPen_red, wd.hBrush_white);
  DrawTrajectory (hdc, wd.trajectory_frames, wd.hPen_orng);
  // --------------------------------------------------------------
  if ( !wd.testing && !wd.testing_trajectories.empty () && 
        wd.testing_trajectories_animation_show )
  {
    for ( auto &t : wd.testing_trajectories )
    { DrawTrajectory (hdc, t, wd.hPen_blue); }

    // auto it = wd.testing_trajectories.begin ();
    // for ( auto i = 0U; i < wd.testing_trajectories_animation_num_iteration; ++i )
    // {
    //   DrawTrajectory (hdc, *it, wd.hPen_blue);
    // 
    //   DrawCircle(hdc, it->back (), CircleRadius);
    //   ++it;
    // } // end for
  } // end if

  if ( wd.trajectory_frames_show )
  { DrawTrajectory (hdc, wd.trajectory_frames, wd.hPen_orng); }
  // // ----- Отрисовка точек БД -------------------------------------
  // if ( !wd.adjPointsDB.empty () )
  // {
  //   HPEN hPen_old = (HPEN) SelectObject (hdc, wd.hPen_cian);
  //   for ( auto &p : wd.adjPointsDB )
  //   { DrawCircle (hdc, p->hit, CircleRadius); }
  //   SelectObject (hdc, hPen_old);
  // }

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
                 reinterpret_cast<LPARAM> (_T (" Done  ")) );
  } // end if
}

tostream&  operator << (tostream &out, const HandMoves::controling_t  &controls)
{
  out << std::endl << _T ("Controls:\n");
  for ( auto &c : controls )
  { out << '-' << c << std::endl; }
  return out;
}
//-------------------------------------------------------------------------------
bool  RepeatMove   (MyWindowData &wd)
{
  const Point &aim = wd.mouse_aim;
  // -------------------------------------------------
  if ( !wd.store.adjacencyPoints (wd.adjPointsDB, aim, 0.005) )
  { return false; }
  // -------------------------------------------------
  ClosestPredicate  pred (aim);
  auto it_min = std::min_element (wd.adjPointsDB.begin (),
                                  wd.adjPointsDB.end (), pred);
  if ( it_min == wd.adjPointsDB.end () )
  { return false; }
  // -------------------------------------------------
  else
  {
    const Record &rec = (**it_min);
    // rec.
    rec.repeatMove (wd.hand);
    wd.trajectory_frames = rec.trajectory;
    // -------------------------------------------------
    tstring text = GetTextToString (wd.hLabMAim);

    tstringstream ss;
    ss << text << _T ("\r");
    for ( const Hand::Control &c : rec.controls )
    { ss << c << _T ("  "); } // \r

    // -------------------------------------------------
    SendMessage (wd.hLabMAim, WM_SETTEXT, NULL, (LPARAM) ss.str ().c_str ());
    // -------------------------------------------------
    return true;
  }
}
void  MakeHandMove (MyWindowData &wd)
{
  if ( !RepeatMove (wd) )
  {
    const Point &aim = wd.mouse_aim;
    // -------------------------------------------------
    wd.testing_trajectories.clear ();
    wd.trajectory_frames.clear ();

    if ( 0 )
    {
      // adjacency_refs_t  range;
      std::pair<Record, Record> x_pair, y_pair;
      auto count = wd.store.adjacencyByPBorders (aim, 0.06, x_pair, y_pair); // range, aim, min, max);

      tcout << count << std::endl;

      wd.testing_trajectories.push_back (x_pair.first.trajectory);
      wd.testing_trajectories.push_back (y_pair.first.trajectory);
      wd.testing_trajectories.push_back (x_pair.second.trajectory);
      wd.testing_trajectories.push_back (y_pair.second.trajectory);
      return;
    }
    // -------------------------------------------------
    if ( 1 /* gradient || rundown */ )
    {
      // WorkerThreadRunStoreTask (wd, _T (" *** stage 1 test ***  "),
      //                             [](Store &store, Hand &hand, const Point &aim,
      //                                trajectory_t *t, trajectories_t *ts)
      //                             {
      //                               Positions::LearnMovements  lm;
      //                               lm.Mean (store, hand, aim, t, ts);
      //                             }
      //                             , wd.hand, aim,
      //                               NULL, // &wd.trajectory_frames,
      //                              &wd.testing_trajectories);
      // WorkerThreadTryJoin (wd);
      // -------------------------------------------------
      Positions::LearnMovements lm;
      lm.gradientMethod (wd.store, wd.hand, aim);
      // lm.Close ( wd.store, wd.hand, aim, 0.1,
      //            NULL, // &wd.trajectory_frames,
      //           &wd.testing_trajectories);
    }
    else if ( 0 /* LinearOperator && SimplexMethod */ )
    {
      HandMoves::controling_t controls;
      Positions::LinearOperator  lp (wd.store, wd.mouse_aim,
                                     0.07, controls, true);
      // lp.predict (wd.mouse_aim, controls);

      wd.hand.SET_DEFAULT;
      wd.hand.move (controls.begin (), controls.end (), &wd.trajectory_frames);
    }
    // -------------------------------------------------
    RepeatMove (wd);
  }

// #ifdef   _ANIMATION_
//   wd.trajectory_frames_show = true;
//   wd.hand.step (wd.trajectory_frames_muscle,
//                 wd.trajectory_frames_lasts);
// #endif // _ANIMATION_
}

void  OnWindowTimer (MyWindowData &wd)
{
  if ( !wd.testing && !wd.reach ) // ????????
  {
    // ============
    for ( size_t i = 0U; i < wd.skip_show_steps; ++i )
    {
      wd.hand.step ();
      // ============
#ifdef    _ANIMATION_
      /* Auto-drawing trajectory animation */
      if ( wd.trajectory_frames_show )
      {
        /* !!! WORKING ONLY FOR NEW_HAND !!! */
        wd.hand.step ();

        if ( wd.hand.moveEnd )
        {
          auto hand_pos = wd.hand.position;
          wd.trajectory_frames.push_back (hand_pos);

          if ( !wd.testing )
          {
            Point  base_pos = wd.trajectory_frames.front ();
            // wd.trajectory_frames.pop_front ();

            auto rec = Record (hand_pos, base_pos, hand_pos,
            { wd.trajectory_frames_muscle },
            { 0U }, { wd.trajectory_frames_lasts },
                               1U, wd.trajectory_frames);
            wd.store.insert (rec);
          }

          // wd.trajectory_frames_show = false;
          // wd.trajectory_frames.clear ();
          wd.trajectory_frames_muscle = Hand::EmptyMov;
          wd.trajectory_frames_lasts = 0;
        }
        else
        { wd.trajectory_frames.push_back (wd.hand.position); }
      }
#endif // _ANIMATION_

      /* Trajectoriaes animation */
      if ( wd.testing_trajectories_animation_show &&
          wd.testing_trajectories_animation_num_iteration >=
          wd.testing_trajectories.size () )
      { ++wd.testing_trajectories_animation_num_iteration; }
    } // end for
  } // end if
}
void  OnWindowMouse (MyWindowData &wd)
{
  wd.mouse_haved = true;
  wd.mouse_aim = logic_coord (&wd.mouse_coords);

  /* Setting the Label's text */
  tstring  message = tstring (wd.mouse_aim) + _T ("  ");
  SendMessage (wd.hLabMAim, WM_SETTEXT, NULL, (WPARAM) message.c_str ());
 
  if ( !wd.testing && wd.mouse_haved )
  {
    wd.adjPointsDB.clear ();
    wd.trajectory_frames.clear ();

    wd.reach = true;
    MakeHandMove (wd);
  }
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
void  OnShowDBTrajes (MyWindowData &wd)
{
  for ( auto &rec : wd.adjPointsDB )
  { 
    wd.trajectoriesDB.push_back ( make_shared<trajectory_t> (rec->trajectory) );
  }
}
//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------