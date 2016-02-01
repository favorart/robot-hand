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
  target ( 32U, 32U, // (-0.39,  0.62, -0.01, -0.99);
                     //  -0.70,  0.90,  0.90, -0.99)
                      -0.41,  0.46, -0.05, -0.90
                     // -0.39, 0.41, -0.05, -0.85
                     // 200U, 200U, -1., 1., -1., 1.
          ),
  scaleLetters (target.Min (), target.Max ())
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

  // p_x = 0; p_y = 0;
  // fm = 0; fn = 0;
  //=======================
  // hand.set (Hand::Shldr | Hand::Elbow, {0.,50.});
  hand.SET_DEFAULT;

  // HandMoves::storeLoad (store);
  WorkerThreadRunStore (*this, _T ("  *** loading ***  "),
                        storeLoad, CurFileName);
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
  HandMoves::storeSave (store, CurFileName);
  //=======================
}
//-------------------------------------------------------------------------------
void  OnPaintMyLogic (HDC hdc, MyWindowData &wd)
{
  const double CircleRadius = 0.01;
  // --------------------------------------------------------------
  /* Target to achive */

  // auto fin = wd.target.coords ()[45];
  // DrawCircle (hdc, fin, REllipse);

  if ( wd.working_space_show )
    DrawTrajectory (hdc, wd.working_space, wd.hPen_orng);

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
      DrawTrajectory (hdc, (**it), wd.hPen_blue);

      DrawCircle(hdc, (**it).back (), CircleRadius);
      ++it;
    } // end for
  } // end if

  // --------------------------------------------------------------
  if ( wd.allPointsDB_show && !wd.store.empty () )
  {
    HPEN hPen_old = (HPEN) SelectObject (hdc, wd.hPen_blue);
    for ( auto rec : wd.store )
      DrawCircle (hdc, rec.aim, CircleRadius);
    SelectObject (hdc, hPen_old);
  }

  // ----- Отрисовка точек БД -------------------------------------
  if ( !wd.adjPointsDB.empty () )
  {
    HPEN hPen_old = (HPEN) SelectObject (hdc, wd.hPen_cian);
    for ( auto p : wd.adjPointsDB )
    {
      DrawCircle (hdc, p->aim, CircleRadius);
      // SetPixel (hdc, Tx (p->aim.x), Ty (p->aim.y), RGB (000, 000, 255));
    }
    SelectObject (hdc, hPen_old);
  }
  // --------------------------------------------------------------
  if ( wd.mouse_haved )
   DrawAdjacency (hdc, wd.mouse_aim, wd.radius, ellipse, wd.hPen_cian);
  // --------------------------------------------------------------
  if ( wd.scaleLetters.show )
   wd.scaleLetters.draw (hdc, wd.hand.points (), &wd.hand.position);
  // --------------------------------------------------------------
}
//-------------------------------------------------------------------------------
//template<typename T, typename... Args>
//void  WorkerThreadStart   (MyWindowData &wd, tstring &message, T func, Args... args)
//{
//  wd.testing = true;
//  /* Setting the Label's text */
//  SendMessage (wd.hLabTest,      /* Label   */
//               WM_SETTEXT,       /* Message */
//               (WPARAM) NULL,    /* Unused  */
//               (LPARAM) message.c_str ());
//
//  wd.pWorkerThread = new boost::thread (func, args...);
//}
void  WorkerThreadTryJoin (MyWindowData &wd)
{
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
          auto rec = Record (aim, aim, { wd.trajectory_frames_muscle },
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
void  OnRandomTest (MyWindowData &wd)
{
  if ( !wd.testing )
  {
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

    WorkerThreadTryJoin (wd);
  }
}
void  OnCoverTest  (MyWindowData &wd)
{ 
  if ( !wd.testing )
  {
    wd.testing = true;
    /* Setting the Label's text */
    SendMessage (wd.hLabTest,      /* Label   */
                 WM_SETTEXT,       /* Message */
                 (WPARAM) NULL,    /* Unused  */
                 (LPARAM) _T("\n *** testing ***  "));

    const size_t nested = 2U;
    wd.pWorkerThread = new boost::thread (HandMoves::test_cover,
                                          std::ref (wd.store),
                                          wd.hand,  /* copy */
                                          nested);
    // hand.SET_DEFAULT;
    // HandMoves::test_cover (wd.store, wd.hand, nested);

    WorkerThreadTryJoin (wd);
  } // end if
}
//-------------------------------------------------------------------------------
void  OnShowTrajectory (MyWindowData &wd)
{
  wd.hand.SET_DEFAULT;
  wd.trajectory_frames.clear ();

  wd.trajectory_frames_show = true;
  // wd.trajectory_frames_muscle = selectHandMove ( random (HandMovesCount) );
  wd.trajectory_frames_muscle = wd.hand.selectControl ();
  wd.trajectory_frames_lasts = random ( wd.hand.maxMuscleLast (wd.trajectory_frames_muscle) );
  
  wd.hand.step (wd.trajectory_frames_muscle);
  wd.trajectory_frames.push_back (wd.hand.position);
}
//-------------------------------------------------------------------------------
void  MakeHandMove (MyWindowData &wd);
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
void  UncoveredTargetPoints (Store &store, const RecTarget &t, std::list<Point> &uncovered)
{
  std::list<std::shared_ptr<Record>> range;
  adjacencyRectPoints (store, range, t.Min (), t.Max ());

  typedef Store::index<Record::ByP>::type::const_iterator StorePcIter;
  Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();

  for ( auto p : t.coords () ) //.cbegin (); it != cend (); )
  {
    if ( index.find (boost::tuple<double, double> (p)) == index.end () )
    { uncovered.push_back (p); }
  }
}


void  MakeHandMove (MyWindowData &wd)
{

  HandMoves::adjacencyPoints (wd.store, wd.adjPointsDB,
                              wd.mouse_aim, wd.radius);

  boost_point2_t m_aim = wd.mouse_aim;
  wd.adjPointsDB.sort ([m_aim](const shared_ptr<Record> &a,
                               const shared_ptr<Record> &b)
                               { boost_point2_t a_p = a->aim;
                                 boost_point2_t b_p = b->aim;
                                 double  d1 = boost::geometry::distance (m_aim, a_p);
                                 double  d2 = boost::geometry::distance (m_aim, b_p);
                                 return (d1 > d2);
                               });

  // wd.testing_trajectories.clear ();

  // tstringstream buffer;
  // for ( auto pRec : wd.pointsDB )
  // {
  //   buffer << pRec << std::endl;
  // 
  //   wd.testing_trajectories.push_back (make_shared<HandMoves::trajectory_t> (pRec->trajectory));
  // }
  // /* Setting the Label's text */
  // SendMessage (wd.hLabStat,      /* Label   */
  //              WM_SETTEXT,       /* Message */
  //              (WPARAM) NULL,    /* Unused  */
  //              (LPARAM) buffer.str ().c_str ()
  //              );

  if ( !wd.adjPointsDB.empty () )
  {
    Record &Rec = (*(*wd.adjPointsDB.begin ()));
    Rec.makeHandMove (wd);
  }
}

void  Record::makeHandMove (MyWindowData &wd)
{
  wd.hand.reset ();
  wd.hand.SET_DEFAULT;
  std::list<Point> visited;
  for ( auto mp : moves_ )
  {
    wd.hand.move (mp.muscle, mp.last); // , visited);
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

