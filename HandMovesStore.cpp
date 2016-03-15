#include "StdAfx.h"
#include "HandMovesStore.h"

using namespace std;
using namespace HandMoves;

// HandMoves::ElegancePredicate pred;
// range.sort (pred);
//------------------------------------------------------------------------------
/* прямоугольная окрестность точки */
size_t  HandMoves::adjacencyRectPoints (Store &store, std::list<Record> &range,
                                        const Point &left_down, const Point &right_up)
{
  typedef Store::index<Record::ByP>::type::const_iterator StorePcIter;
  Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();
  /* Range searching, i.e.the lookup of all elements in a given interval */
  StorePcIter itFirstLower = index.lower_bound (boost::tuple<double, double> (left_down));
  StorePcIter itFirstUpper = index.upper_bound (boost::tuple<double, double> (right_up));

  size_t count = 0U;
  for ( auto it = itFirstLower; it != itFirstUpper; ++it )
  { range.push_back (*it);
    ++count;
  }
  return count;
}
size_t  HandMoves::adjacencyRectPoints (Store &store, std::list<std::shared_ptr<Record>> &range,
                                        const Point &left_down, const Point &right_up)
{
  typedef Store::index<Record::ByP>::type::const_iterator StorePcIter;
  Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();

  StorePcIter itFirstLower = index.lower_bound (boost::tuple<double,double> (left_down));
  StorePcIter itFirstUpper = index.upper_bound (boost::tuple<double,double> (right_up));

  size_t count = 0U;
  for ( auto it = itFirstLower; it != itFirstUpper; ++it )
  { range.push_back ( make_shared<Record> (*it) );
    ++count;
  }
  return count;
}
//------------------------------------------------------------------------------
/* круглая окрестность точки */
size_t  HandMoves::adjacencyPoints (Store &store, std::list<Record> &range,
                                    const Point &center, double radius)
{
  typedef Store::index<Record::ByP>::type::const_iterator StorePcIter;
  Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();

  StorePcIter itFirstLower = index.lower_bound (boost::make_tuple (center.x - radius,
                                                                   center.y - radius));
  StorePcIter itFirstUpper = index.upper_bound (boost::make_tuple (center.x + radius,
                                                                   center.y + radius));
  
  // auto range_it = std::back_inserter (range);
  size_t count = 0U;
  for ( auto it = itFirstLower; it != itFirstUpper; ++it )
  {
    if ( boost::geometry::distance (boost_point2_t (center),
                                    boost_point2_t (it->hit)) <= radius )
    {
      range.push_back (*it);
      // *range_it = (pointer_type) ? (make_shared<Record> (*it)) : (*it); ++range_it;
      ++count;
    }
  }
  return count;
}
size_t  HandMoves::adjacencyPoints (Store &store, std::list<std::shared_ptr<HandMoves::Record>> &range,
                                    const Point &center, double radius)
{
  typedef Store::index<Record::ByP>::type::const_iterator StorePcIter;
  Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();

  StorePcIter itFirstLower = index.lower_bound (boost::make_tuple (center.x - radius,
                                                                   center.y - radius));
  StorePcIter itFirstUpper = index.upper_bound (boost::make_tuple (center.x + radius,
                                                                   center.y + radius));

  size_t count = 0U;
  for ( auto it = itFirstLower; it != itFirstUpper; ++it )
  {
    if ( boost::geometry::distance (boost_point2_t (center), 
                                    boost_point2_t (it->hit)) <= radius )
    { range.push_back (make_shared<Record> (*it));
      ++count;
    }
  }
  return count;
}
//------------------------------------------------------------------------------
size_t  HandMoves::similDistances (Store &store, std::list<std::shared_ptr<HandMoves::Record>> &range,
                                   double distance, double over)
{
  //typedef Store::index<Record::ByD>::type::const_iterator StoreDcIter;
  //Store::index<Record::ByD>::type& index = store.get<Record::ByD> ();

  //StoreDcIter itFirstLower = index.lower_bound (distance - over);
  //StoreDcIter itFirstUpper = index.upper_bound (distance + over);

  size_t count = 0U;
  //for ( auto it = itFirstLower; it != itFirstUpper; ++it )
  //{
  //  range.push_back (make_shared<Record> (*it));
  //  ++count;
  //}
  return count;
}
size_t  HandMoves::similControls (Store &store, std::list<std::shared_ptr<HandMoves::Record>> &range,
                                  Hand::MusclesEnum control, bool contains_not_exact)
{
  //typedef Store::index<Record::ByD>::type::const_iterator StoreDcIter;
  //Store::index<Record::ByD>::type& index = store.get<Record::ByD> ();

  //auto equal_range = index.equal_range (control,
  //                                      [](Hand::MusclesEnum a,
  //                                         Hand::MusclesEnum b)
  //                                      { return a & b; });
  //// StoreDcIter itFirstLower = index.lower_bound (distance - over);
  //// StoreDcIter itFirstUpper = index.upper_bound (distance + over);
  //// 
  size_t count = 0U;
  //for ( auto it = equal_range.first; it != equal_range.second; ++it )
  //{
  //  range.push_back (make_shared<Record> (*it));
  //  ++count;
  //}
  return count;
}
//------------------------------------------------------------------------------
// Все точки с данным x 
void  HandMoves::adjacencyYsByXPoints (Store &store, std::list<Record> &range,
                                       double x, double up, double down)
{
  //typedef Store::index<Record::ByX>::type::iterator StoreXiter;
  //Store::index<Record::ByX>::type& index = store.get<Record::ByX> ();
  //
  //StoreXiter itFirstLower = index.lower_bound (up);
  //StoreXiter itFirstUpper = index.upper_bound (down);
  //
  //// Store::index<Record::ByX>::type::const_iterator ity = lsy.find (x);
  //// if ( ity != lsy.end () ) {}
  //std::copy (itFirstLower, itFirstUpper, std::back_inserter (range));
}
// Все точки с данным y 
void  HandMoves::adjacencyXsByYPoints (Store &store, std::list<Record> &range,
                                       double y, double left, double right)
{
  //typedef Store::index<Record::ByY>::type::iterator StoreYiter;
  //Store::index<Record::ByY>::type& index = store.get<Record::ByY> ();
  //
  //StoreYiter itFirstLower = index.lower_bound (left);
  //StoreYiter itFirstUpper = index.upper_bound (right);
  //
  //// Store::index<Record::ByY>::type::const_iterator ity = lsy.find (fy);
  //// if ( ity != lsy.end () ) {}
  //std::copy (itFirstLower, itFirstUpper, std::back_inserter (range));
}
//------------------------------------------------------------------------------
// enum { Pixels, Ellipses };
void  HandMoves::storeDraw (HDC hdc, const Store &store, double CircleRadius, HPEN hPen)
{
  HPEN hPen_old = (HPEN) SelectObject (hdc, hPen);
  // --------------------------------------------------------------
  std::unordered_map<Point, double, PointHasher>  map_points;
  for ( auto &rec : store )
  {
    /*  Если в одну точку попадают несколько движений -
    *  выбрать лучшее движение по элегантности */

    auto pRec = map_points.find (rec.hit);
    if ( pRec != map_points.end () )
    {
      double elegance = rec.eleganceMove ();
      if ( pRec->second < elegance )
        map_points.insert (std::make_pair (rec.hit, elegance));
    }
    else
    { map_points.insert (std::make_pair (rec.hit, rec.eleganceMove ())); }
  }

  for ( auto &pt : map_points )
  { DrawCircle (hdc, pt.first, CircleRadius); }
  // --------------------------------------------------------------
  SelectObject (hdc, hPen_old);
}
void  HandMoves::storeDraw (HDC hdc, const Store &store, color_interval_t colors)
{
  gradient_t  gradient;
  MakeGradient (colors, 190, gradient);

  // int i = 0;
  // for ( auto c : gradient )
  // {
  //   std::wcout << '(' << GetRValue (c) << ' ' << GetGValue (c) << ' ' << GetBValue (c) << ' ' << ')' << ' '; //std::endl;
  //   HPEN Pen = CreatePen (PS_SOLID, 1, c);
  //   DrawCircle (hdc,Point(0.01 * i -0.99, 0.9), 0.01, Pen);
  //   DeleteObject (Pen);
  //   ++i;
  // }

  try
  {
    // int i = 0U;
    // --------------------------------------------------------------
    for ( auto &rec : store )
    {
      double elegance = rec.eleganceMove ();

      // std::wcout << elegance << std::endl;
      int index = static_cast<int> (elegance * (gradient.size () - 1));

      // COLORREF col = GetPixel (hdc, Tx (rec.hit.x), Ty (rec.hit.y));
      // if ( col >= RGB(255,0,0) && col <= RGB(0,0,255) && gradient[index] < col )

      SetPixel (hdc, Tx (rec.hit.x), Ty (rec.hit.y), gradient[index]);

      // HPEN Pen = CreatePen (PS_SOLID, 1, gradient[index]);
      // DrawCircle (hdc, rec.hit, 0.01, Pen);
      // DeleteObject (Pen);


      // ++i;
      // if ( !(i % 1000) )
      // { 
      //   tstringstream ss; ss << i << _T ("  ");
      //   SendMessage (wd.hLabMAim, WM_SETTEXT, NULL,
      //                reinterpret_cast<LPARAM> (ss.str().c_str()) );
      // }

      boost::this_thread::interruption_point ();
    }
    // --------------------------------------------------------------
  }
  catch ( boost::thread_interrupted& )
  { /* std::cout << "WorkingThread interrupted" << std::endl; */ }
}
void  HandMoves::storeSave (const Store &store, tstring filename)
{
  boost::this_thread::disable_interruption  no_interruption;
  try
  {
    ofstream  ofs (filename, std::ios_base::binary | std::ios_base::out);
    // boost::archive::text_oarchive  oa (ofs);
    boost::archive::binary_oarchive   oa (ofs);
    oa << store;
  }
  catch ( ... )
  {
    tstring last_error = GetLastErrorToString ();
    MessageBoxW (NULL, last_error.c_str (), _T ("Error"), MB_OK);
  }
}
void  HandMoves::storeLoad (      Store &store, tstring filename)
{
  // boost::this_thread::disable_interruption  no_interruption;
  if ( isFileExists (filename.c_str ()) )
  {
    ifstream  ifs (filename,  std::ios_base::binary | std::ios_base::in);
    std::stringstream buffer (std::ios_base::binary | std::ios_base::in | std::ios_base::out);
    buffer << ifs.rdbuf ();
    ifs.close ();

    // ifstream  ifs (filename);
    // boost::archive::text_iarchive    ia (ifs);
    // boost::archive::binary_iarchive  ia (ifs);
    boost::archive::binary_iarchive     ia (buffer);
    ia >> store;
  }
}
//------------------------------------------------------------------------------
void  HandMoves::storeInsert (    Store &store, const Record &rec)
{

  // if (rec.aim in store)
  // {
  //   /* Вверх класть более удачные, отн. кол-ва движение и точность попадания */
  //  
  //   /* ??? Несколько вариантов одного и того же движения ??? */
  //   // массив траекторий для каждой точки
  //
  //   ?? store.insert (rec);
  // }
  // else
  try
  {
    store.insert (rec);
  }
  catch ( ... )
  {
    MessageBox (NULL, GetLastErrorToString ().c_str (), _T (" Error "), MB_OK);
  }
}
//------------------------------------------------------------------------------
