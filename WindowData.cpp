#include "StdAfx.h"
#include "WindowData.h"
#include "Draw.h"

using namespace std;
using namespace HandMoves;
//-------------------------------------------------------------------------------
MyWindowData:: MyWindowData (HWND hLabMAim, HWND hLabTest, HWND hLabStat) :
  pWorkerThread(nullptr), 
  hLabMAim (hLabMAim),
  hLabTest (hLabTest),
  hLabStat (hLabStat),
  target (32U, 32U, // (-0.39,  0.62, -0.01, -0.99);
                    //  -0.70,  0.90,  0.90, -0.99)
                        -0.39,  0.41, -0.05, -0.85)
{
  std::srand ((unsigned int) clock ());

  testing = false;
  reach   = false;

  /* создаем ручки */
  hPen_grn  = CreatePen (PS_SOLID, 1, RGB (100, 180, 050));
  hPen_red  = CreatePen (PS_SOLID, 2, RGB (255, 000, 000));
  hPen_blue = CreatePen (PS_SOLID, 2, RGB (000, 000, 255));
  hPen_cian = CreatePen (PS_SOLID, 2, RGB (057, 168, 157));
  hPen_orng = CreatePen (PS_SOLID, 2, RGB (255, 128, 000));

  /* создаем кисти */
  hBrush_white = (HBRUSH) GetStockObject (WHITE_BRUSH);
  hBrush_null  = (HBRUSH) GetStockObject (NULL_BRUSH);

  // p_x = 0; p_y = 0;
  // fm = 0; fn = 0;
  //=======================
  hand.SET_DEFAULT;
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
  DeleteObject (hPen_orng);

  DeleteObject (hBrush_white);
  DeleteObject (hBrush_null);
    
  if ( pWorkerThread )
  {
    // pWorkerThread->interrupt ();
    delete pWorkerThread;
    pWorkerThread = nullptr;
  }
  //=======================
  HandMoves::storeSave (store);
  //=======================
}
//-------------------------------------------------------------------------------
void  OnPaintMyLogic (HDC hdc, MyWindowData &wd)
{
  // ----- Отрисовка фигуры ------------------------
  wd.hand.draw (hdc, wd.hPen_red, wd.hBrush_white);
  draw_trajectory (hdc, wd.trajectory_frames, wd.hPen_orng);

  if ( !wd.testing && !wd.testing_trajectories.empty () && 
        wd.testing_trajectories_animation_show )
  {
    // for ( auto t : wd.testing_trajectories )
    auto it = wd.testing_trajectories.begin ();
    for ( auto i = 0U; i < wd.testing_trajectories_animation_num_iteration; ++i )
    {
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
  if ( !wd.testing && !wd.reach )
  {
    /* Auto-drawing trajectory animation */
    if ( wd.trajectory_frames_show )
    { --wd.trajectory_frames_lasts;
      if ( !wd.trajectory_frames_lasts )
      {
        std::wcout << L"end " << std::wstring (wd.hand.position) << std::endl;
        wd.hand.step (wd.trajectory_frames_muscle);
      }
    }
    // ============
    wd.hand.step ();
    // ============
    if ( wd.trajectory_frames_show )
    {
       // ???????
      // std::wcout << std::wstring (wd.hand.position) << std::endl;

      // wd.trajectory_frames.push_back (wd.hand.position);

      if ( wd.hand.isMoveEnd () )
      {
        // auto aim = wd.hand.position;
        // auto rec = Record (aim, aim, { wd.trajectory_frames_muscle },
        //                    { 0U }, { wd.trajectory_frames.size() /* !!!!!! */ },
        //                    1U, wd.trajectory_frames);
        // wd.store.insert (rec);
      }

    }

    /* Trajectoriaes animation */
    if ( wd.testing_trajectories_animation_show && 
         wd.testing_trajectories_animation_num_iteration >= 
         wd.testing_trajectories.size () )
    { ++wd.testing_trajectories_animation_num_iteration; }
  }
  else  if ( wd.pWorkerThread && wd.pWorkerThread->try_join_for (boost::chrono::milliseconds (10)) )
  { /* joined */
    delete wd.pWorkerThread;
    wd.pWorkerThread = nullptr;

    wd.testing = false;

    /* Setting the Label's text */
    SendMessage (wd.hLabTest,         /* Label   */
                 WM_SETTEXT,       /* Message */
                 (WPARAM) NULL,    /* Unused  */
                 (LPARAM) _T (" "));
  }
}
void  OnWindowMouse (MyWindowData &wd)
{
  wd.mouse_haved = true;
  wd.mouse_aim = logic_coord (&wd.mouse_coords);

  /* Setting the Label's text */
  SendMessage (wd.hLabMAim,      /* Label   */
               WM_SETTEXT,       /* Message */
               (WPARAM) NULL,    /* Unused  */
               (LPARAM) wstring (wd.mouse_aim).c_str () );
  
  OnShowDBPoints (wd);

  // HandMoves::adjacencyRectPoints<decltype(found_points)>
  //  (store,  std::back_inserter (found_points), { 0.1, 0.1 }, { 0.4, 0.3 });

  //HandMoves::adjacencyPoints //<decltype(found_points)>
  //  (store, std::back_inserter (found_points), aim, radius, true);
}
//-------------------------------------------------------------------------------
void  OnRandomTest (MyWindowData &wd)
{
  if ( !wd.testing )
  {
    // ?wd.testing_trajectories ???.clear ();

    wd.testing = true;
    /* Setting the Label's text */
    SendMessage (wd.hLabTest,         /* Label   */
                 WM_SETTEXT,       /* Message */
                 (WPARAM) NULL,    /* Unused  */
                 (LPARAM) _T ("\n *** testing ***  "));

    const size_t repeats = 1000U;
    wd.pWorkerThread = new boost::thread (HandMoves::test_random,
                                         std::ref (wd.store),
                                         wd.hand, /* copy */
                                         repeats);

    // wd.hand.SET_DEFAULT;
    // test_random (wd.store, wd.hand, 1000U);

    if ( wd.pWorkerThread->try_join_for (boost::chrono::milliseconds (10)) )
    { /* joined */
      delete wd.pWorkerThread;
      wd.pWorkerThread = nullptr;

      wd.testing = false;
      /* Setting the Label's text */
      SendMessage (wd.hLabTest,      /* Label   */
                   WM_SETTEXT,       /* Message */
                   (WPARAM) NULL,    /* Unused  */
                   (LPARAM) _T (" "));
    } // end if
  }

  
}
void  OnCoverTest  (MyWindowData &wd)
{ 
  if ( !wd.testing )
  {
    // wd.testing = true;
    wd.testing_trajectories.clear ();

    /* Setting the Label's text */
    SendMessage (wd.hLabTest,      /* Label   */
                 WM_SETTEXT,       /* Message */
                 (WPARAM) NULL,    /* Unused  */
                 (LPARAM) _T("\n *** testing ***  "));

    const size_t nested = 1U;
    // wd.pWorkerThread = new boost::thread (HandMoves::test_cover,
    //                                       std::ref (wd.store),
    //                                       wd.hand,  /* copy */
    //                                       // std::ref (wd.testing_trajectories),
    //                                       nested);



    // hand.SET_DEFAULT;
    HandMoves::test_cover (wd.store, wd.hand, nested);

    if ( wd.pWorkerThread && wd.pWorkerThread->try_join_for (boost::chrono::milliseconds (10)) )
    { /* joined */
      delete wd.pWorkerThread;
      wd.pWorkerThread = nullptr;

      wd.testing = false;
      wd.testing_trajectories_animation_num_iteration = 1;
      wd.testing_trajectories_animation_show = true;

      /* Setting the Label's text */
      SendMessage (wd.hLabTest,      /* Label   */
                   WM_SETTEXT,       /* Message */
                   (WPARAM) NULL,    /* Unused  */
                   (LPARAM) _T (" "));
    } // end if
  } // end if
}
//-------------------------------------------------------------------------------
void  OnShowTrajectory (MyWindowData &wd)
{
  wd.hand.SET_DEFAULT;
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


void  MakeHandMove (MyWindowData &wd);
void  OnShowDBPoints (MyWindowData &wd)
{
  if ( !wd.testing )
  { 
    if ( wd.mouse_haved )
    {
      wd.pointsDB.clear ();


     

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

      // if ( wd.pWorkerThread->try_join_for (boost::chrono::milliseconds (1)) )
      // { /* joined */
      //   delete wd.pWorkerThread;
      //   wd.pWorkerThread = nullptr;
      // 
      //   wd.testing = false;
      // } // end if
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
void  UncoveredTargetPoints (Store &store, const RecTarget &t, std::list<Point> &uncovered)
{
  std::list<std::shared_ptr<Record>> range;
  adjacencyRectPoints (store, range, t.Min (), t.Max ());

  for ( auto p : range )
  {
    // if ( p not in t )
    // { uncovered.push_back (p); }
  }
}


void  MakeHandMove (MyWindowData &wd)
{

  HandMoves::adjacencyPoints (wd.store, wd.pointsDB,
                              wd.mouse_aim, wd.radius);

  boost_point2_t m_aim = wd.mouse_aim;
  wd.pointsDB.sort ([m_aim](const shared_ptr<Record> &a,
                            const shared_ptr<Record> &b)
                            { boost_point2_t a_p = a->aim;
                              boost_point2_t b_p = b->aim;
                              double  d1 = boost::geometry::distance (m_aim, a_p);
                              double  d2 = boost::geometry::distance (m_aim, b_p);
                              return (d1 > d2);
                            });

  wd.testing_trajectories.clear ();

  std::wstringstream buffer;
  for ( auto pRec : wd.pointsDB )
  {
    buffer << pRec << std::endl;

    wd.testing_trajectories.push_back (make_shared<HandMoves::trajectory_t> (pRec->trajectory));
  }
  /* Setting the Label's text */
  SendMessage (wd.hLabStat,      /* Label   */
               WM_SETTEXT,       /* Message */
               (WPARAM) NULL,    /* Unused  */
               (LPARAM) buffer.str ().c_str ()
               );

  if ( !wd.pointsDB.empty () )
  {
    Record &Rec = (*(*wd.pointsDB.begin ()));
    Rec.makeHandMove (wd);
  }
}

void  Record::makeHandMove (MyWindowData &wd)
{
  wd.hand.reset ();
  wd.hand.SET_DEFAULT;
  std::list<Point> visited;
  for ( auto mp : moves_ ) // !!! TIME ORDER ???
  {
    wd.hand.move (mp.muscle, mp.last, visited);
    
  }

  // for ( auto p : visited )
  //   wcout << wstring (p) << endl;
  // wcout << visited.size () << endl << endl;
  // 
  // for ( auto p : trajectory )
  //   wcout << wstring (p) << endl;
  // wcout << trajectory.size () << endl << endl;

  std::copy (trajectory.begin (), trajectory.end (), std::back_inserter(wd.trajectory_frames));
}

