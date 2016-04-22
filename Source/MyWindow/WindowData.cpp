#include "StdAfx.h"
#include "WindowData.h"
#include "Draw.h"
#include "Position.h"

extern bool zoom;

using namespace std;
using namespace HandMoves;

bool    RepeatMove (MyWindowData &wd);
bool  MakeHandMove (MyWindowData &wd);
//-------------------------------------------------------------------------------
void  MyWindowData::TrajectoryFrames::step  (Store &store, Hand &hand, const boost::optional<controling_t> controls)
{
  if ( controls.is_initialized () )
  {
    show_ = true;

    if ( animation )
    {
      controls_ = controling_t{ *controls };
      time_     = Hand::frames_t{ 0U };

      for ( iter_   = controls_->begin ();
          (*iter_) != controls_->end () && 
         (**iter_).start == time_;
        ++(*iter_) )
      { hand.step ((**iter_).muscle, (**iter_).last); }

      ++(*time_);
    }
    else
    {
      hand.SET_DEFAULT;
      Point hand_base = hand.position;

      hand.move (controls->begin (), controls->end (), &trajectory_);
      // Point  hand_base = trajectory_.front ();
      // wd.trajectory_frames.pop_front ();
      store.insert (HandMoves::Record (hand.position, hand_base, hand.position,
                                       *controls, trajectory_));
      // trajectory_.clear ();
    }
  }
  // ============
  /* Auto-drawing trajectory animation */
  if ( show_ && animation )
  {
    if ( time_.is_initialized () )
    {
      for ( size_t i = 0U; i < skip_show_steps; ++i )
      {
        // ============
        // hand.step ();
        // ============
        if ( ((*iter_) != (*controls_).end ()) && ((**iter_).start == time_) )
        {
          while ( ((*iter_) != (*controls_).end ()) && ((**iter_).start == time_) )
          {
            hand.step ((**iter_).muscle, (**iter_).last);
            ++(*iter_);
          }
        }
        else
        { hand.step (); /* !!! WORKING ONLY FOR NEW_HAND !!! */ }
        // ============
        if ( hand.moveEnd )
        {
          const Point &hand_pos = hand.position;
          trajectory_.push_back (hand_pos);
          // -------------------------------------------
          Point  base_pos = trajectory_.front ();
          // wd.trajectory_.pop_front ();
          store.insert (HandMoves::Record (hand_pos, base_pos, hand_pos,
                                           *controls_, trajectory_));
          // -------------------------------------------
          controls_ = boost::none;
          // -------------------------------------------
          iter_ = boost::none;
          time_ = boost::none;
          // -------------------------------------------
          break;
        }
        else if ( !(*time_ % hand.visitedSaveEach) )
        { trajectory_.push_back (hand.position); }
        // ============
        (*time_) += 1U;
        // ============
      } // end for
    } // end if
  } // end if
}
void  MyWindowData::TrajectoryFrames::draw  (HDC hdc, HPEN hPen) const
{ if ( show_ ) { DrawTrajectory (hdc, trajectory_, hPen); } }
void  MyWindowData::TrajectoryFrames::clear ()
{
  show_ = false;
  trajectory_.clear ();
  // -------------------------------------------
  controls_ = boost::none;
  // -------------------------------------------
  time_ = boost::none;
  iter_ = boost::none;
}
//-------------------------------------------------------------------------------
MyWindowData:: MyWindowData () :
  pWorkerThread (nullptr),
  // lt (NULL),
  target ( // 200U, 200U,
           18U, 18U,
           // 32U, 32U, // (-0.39,  0.62, -0.01, -0.99);
                        //  -0.70,  0.90,  0.90, -0.99)
                         -0.41,  0.46, -0.05, -0.90
                        // -0.39, 0.41, -0.05, -0.85
          ),
  scaleLetters (target.min (), target.max ())
{
  std::srand ((unsigned int) clock ());

  testing = false;

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
  { 

#ifdef _DRAW_STORE_GRADIENT_
    size_t colorGradations = 15U;
    color_interval_t colors = // make_pair (RGB(0,0,130), RGB(255,0,0)); // 128
                                 make_pair (RGB(150,10,245), RGB(245,10,150));
                              // make_pair (RGB(130,0,0), RGB(255,155,155));
    
    gradient_t  gradient;
    MakeGradient (colors, colorGradations, gradient);
#else
    gradient_t  gradient ({ RGB (25, 255, 25),
                            RGB (25, 25, 255),
                            RGB (255, 25, 25) });
#endif
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
  const double  CircleRadius = 0.01;
  // --------------------------------------------------------------
  /* Target to achive */
  // if ( wd.lt )  wd.lt->draw (hdc, wd);

  // ----- Отрисовка фигуры ---------------------------------------
  wd.hand.draw (hdc, wd.hPen_red, wd.hBrush_white);
  // --------------------------------------------------------------
  wd.trajectory_frames.draw (hdc, wd.hPen_orng);
  // --------------------------------------------------------------
  if ( !wd.testing && wd.testing_trajectories_show &&
       !wd.testing_trajectories.empty () )
  {

#ifdef    _TESTING_TRAJECTORIES_ANIMATION_

    auto  it = wd.testing_trajectories.begin ();
    for ( size_t i = 0U; i < wd.testing_trajectories_animation_num_iteration; ++i, ++it )
    {
      DrawTrajectory (hdc, *it, wd.hPen_blue);
      DrawCircle     (hdc,  it->back (), CircleRadius);
    }

#else // !_TESTING_TRAJECTORIES_ANIMATION_ !not! 

    for ( auto &t : wd.testing_trajectories )
    { DrawTrajectory (hdc, t, wd.hPen_blue); }

#endif // _TESTING_TRAJECTORIES_ANIMATION_

  } // end if
  
  // ----- Отрисовка точек БД -------------------------------------
  if ( !wd.adjPointsDB.empty () )
  {
    HPEN hPen_old = (HPEN) SelectObject (hdc, wd.hPen_cian);
    for ( auto &p : wd.adjPointsDB )
    { DrawCircle (hdc, p->hit, CircleRadius); }
    SelectObject (hdc, hPen_old);
  }
  // --------------------------------------------------------------
  if ( wd.mouse_haved )
  { DrawAdjacency (hdc, wd.mouse_aim, wd.radius, ellipse, wd.hPen_cian); }
  // --------------------------------------------------------------
  if ( wd.scaleLetters.show )
  { wd.scaleLetters.draw (hdc, wd.hand.jointsPositions (), &wd.hand.position); }
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

#ifdef    _TESTING_TRAJECTORIES_ANIMATION_
    wd.testing_trajectories_animation_num_iteration = 1;
    wd.testing_trajectories_animation_show = true;
#endif // _TESTING_TRAJECTORIES_ANIMATION_

    /* Set text of label 'Stat'  */
    SendMessage (wd.hLabTest, WM_SETTEXT, NULL,
                 reinterpret_cast<LPARAM> (_T (" Done  ")) );
  } // end if
}
//-------------------------------------------------------------------------------
void  OnWindowTimer (MyWindowData &wd)
{
  if ( !wd.testing )
  {
    wd.trajectory_frames.step (wd.store, wd.hand);

#ifdef    _TESTING_TRAJECTORIES_ANIMATION_

      /* Trajectoriaes animation */
      if ( wd.testing_trajectories_animation_show &&
           wd.testing_trajectories_animation_num_iteration < wd.testing_trajectories.size () )
      { ++wd.testing_trajectories_animation_num_iteration; }

#endif  // _TESTING_TRAJECTORIES_ANIMATION_

  } // end if
}
void  OnWindowMouse (MyWindowData &wd)
{
  wd.mouse_haved = true;
  wd.mouse_aim = logic_coord (&wd.mouse_coords);
  /* Setting the Label's text */
  tstring  message = tstring (wd.mouse_aim) + _T ("  ");
  SendMessage (wd.hLabMAim, WM_SETTEXT, NULL, (WPARAM) message.c_str ());
  // -------------------------------------------------
  if ( !wd.testing ) { MakeHandMove (wd); }
}
//-------------------------------------------------------------------------------
void  OnShowDBPoints (MyWindowData &wd)
{
  if ( !wd.testing )
  {
    wd.adjPointsDB.clear ();
    wd.testing_trajectories.clear ();
    wd.store.adjacencyPoints (wd.adjPointsDB, wd.mouse_aim, wd.radius);
  }
}
void  OnShowDBTrajes (MyWindowData &wd)
{
  wd.trajectory_frames.clear ();
  for ( auto &rec : wd.adjPointsDB )
  { wd.trajectoriesDB.push_back ( make_shared<trajectory_t> (rec->trajectory) ); }
}
//-------------------------------------------------------------------------------
bool  RepeatMove   (MyWindowData &wd)
{
  const Point &aim = wd.mouse_aim;
  // -------------------------------------------------
  HandMoves::adjacency_refs_t  range;
  if ( !wd.store.adjacencyPoints (range, aim, wd.target.precision () + wd.side) )
  { return false; }
  // -------------------------------------------------
  ClosestPredicate  pred (aim);
  auto it_min = boost::range::min_element (range, pred);
  // -------------------------------------------------
  if ( it_min != range.end () )
  {
    /* Repeat Hand Movement */
    const Record &rec = (**it_min);

    wd.hand.SET_DEFAULT;
    wd.trajectory_frames.step (wd.store, wd.hand, boost::optional<controling_t>{rec.controls});
    // -------------------------------------------------
    if ( !wd.trajectory_frames.animation )
    {
      if ( !boost::equal (wd.trajectory_frames.trajectory, rec.trajectory) )
      { throw std::exception ("Incorrect Repeat Hand Move"); }
    }
    // -------------------------------------------------
    tstring text = GetTextToString (wd.hLabMAim);

    tstringstream ss;
    ss << text << _T ("\r");
    for ( const Hand::Control &c : rec.controls )
    { ss << c << _T ("  "); } // \r

    SendMessage (wd.hLabMAim, WM_SETTEXT, NULL, (LPARAM) ss.str ().c_str ());
    // -------------------------------------------------
    return true;
  }
  return false;
}
bool  MakeHandMove (MyWindowData &wd)
{
  wd.adjPointsDB.clear ();
  wd.trajectory_frames.clear ();

  Positions::LearnMovements lm;
  lm.gradientMethod_admixture (wd.store, wd.hand, wd.mouse_aim);

  if ( 0 && !RepeatMove (wd) )
  {
    const Point &aim = wd.mouse_aim;
    // -------------------------------------------------
    wd.testing_trajectories.clear ();
    wd.trajectory_frames.clear ();

    if ( 0 )
    {
      // adjacency_refs_t  range;
      // std::pair<Record, Record>  x_pair, y_pair;
      // auto count = wd.store.adjacencyByPBorders (aim, 0.06, x_pair, y_pair); // range, aim, min, max);
      // 
      // tcout << count << std::endl;
      // 
      // wd.testing_trajectories.push_back (x_pair.first.trajectory);
      // wd.testing_trajectories.push_back (y_pair.first.trajectory);
      // wd.testing_trajectories.push_back (x_pair.second.trajectory);
      // wd.testing_trajectories.push_back (y_pair.second.trajectory);
      // return false;
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
      // HandMoves::controling_t controls;
      // Positions::LinearOperator  lp (wd.store, wd.mouse_aim,
      //                                0.07, controls, true);
      // // lp.predict (wd.mouse_aim, controls);
      // 
      // wd.hand.SET_DEFAULT;
      // wd.hand.move (controls.begin (), controls.end (), &wd.trajectory_frames.trajectory);
    }
    // -------------------------------------------------
    return  RepeatMove (wd);
  }
  return !RepeatMove (wd); // true;
}
//-------------------------------------------------------------------------------
