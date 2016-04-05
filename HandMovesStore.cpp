#include "StdAfx.h"
#include "HandMovesStore.h"

using namespace std;
using namespace HandMoves;
//------------------------------------------------------------------------------
// Все точки с данным x 
void  HandMoves::Store::adjacencyYsByXPoints (std::list<Record> &range,
                                              double x, double up, double down) const
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
void  HandMoves::Store::adjacencyXsByYPoints (std::list<Record> &range,
                                              double y, double left, double right) const
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
void  HandMoves::Store::draw (HDC hdc, double circleRadius, HPEN hPen) const
{
  HPEN hPen_old = (HPEN) SelectObject (hdc, hPen);
  // --------------------------------------------------------------
  std::unordered_map<Point, double, PointHasher>  map_points;
  {
    boost::lock_guard<boost::mutex>  lock (store_mutex_);
    for ( auto &rec : store_ )
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
  }
  for ( auto &pt : map_points )
  { DrawCircle (hdc, pt.first, circleRadius); }
  // --------------------------------------------------------------
  SelectObject (hdc, hPen_old);
}
void  HandMoves::Store::draw (HDC hdc, gradient_t gradient, double circleRadius) const
{
  vector<HPEN> hPens (gradient.size());
  for ( int i = 0; i < gradient.size (); ++i )
  {
    // std::wcout << '(' << GetRValue (c) << ' '
    //                   << GetGValue (c) << ' '
    //                   << GetBValue (c) << ' '
    //            << ')' << ' '; // std::endl;
  
    HPEN hPen = CreatePen (PS_SOLID, 1, gradient[i]);
    DrawCircle (hdc, Point (0.01 * i - 0.99, 0.9), 0.01, hPen);
    hPens[i] = hPen;
  }

  boost::lock_guard<boost::mutex>  lock (store_mutex_);

  try
  {
    int i = 0U;
    // --------------------------------------------------------------
    for ( auto &rec : store_ )
    {
      // double elegance = rec.eleganceMove ();
      // std::wcout << elegance << std::endl;
      // size_t index = static_cast<size_t> (elegance * (gradient.size () - 1));
      // // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      // index = index >= gradient.size () ? gradient.size () - 1 : index;

      // double longs = static_cast<double> (rec.longestMusclesControl () - minTimeLong);
      size_t longs = rec.longestMusclesControl ();

      size_t index;
      if ( longs > 500U )
        index = 2U;
      else if ( longs > 200U )
        index = 1U;
      else
        index = 0U;

      // double step  = static_cast<double> (maxTimeLong - minTimeLong) / gradient.size ();
      // size_t index = static_cast<size_t> (longs / step);


      // size_t index = static_cast<size_t>
      //               (((longs       - minTimeLong) /
      //                 (maxTimeLong - minTimeLong)) * (gradient.size (); - 1));

      // COLORREF col = GetPixel (hdc, Tx (rec.hit.x), Ty (rec.hit.y));
      // if ( col >= RGB(255,0,0) && col <= RGB(0,0,255) && gradient[index] < col )

      if ( !circleRadius )
      { SetPixel (hdc, Tx (rec.hit.x), Ty (rec.hit.y), gradient[index]); }
      else
      { DrawCircle (hdc, rec.hit, 0.01, hPens[index]); }

      // ++i;
      // if ( !(i % 100) )
      // { 
      //   // tstringstream ss; ss << i << _T ("  ");
      //   // SendMessage (wd.hLabMAim, WM_SETTEXT, NULL,
      //   //              reinterpret_cast<LPARAM> (ss.str().c_str()) );
      //   std::wcout << '(' << longs << ' ' << index << ')' << ' '; // std::endl;
      // 
      // }

      boost::this_thread::interruption_point ();
    }
  }
  catch ( boost::thread_interrupted& )
  { /* std::cout << "WorkingThread interrupted" << std::endl; */ }
  // --------------------------------------------------------------
  for ( auto hPen : hPens )
  { DeleteObject (hPen); }
}
void  HandMoves::Store::save (tstring filename) const
{
  boost::this_thread::disable_interruption  no_interruption;
  boost::lock_guard<boost::mutex>  lock (store_mutex_);
  try
  {
    ofstream  ofs (filename, std::ios_base::binary | std::ios_base::out);
    // boost::archive::text_oarchive  toa (ofs);
    boost::archive::binary_oarchive   boa (ofs);
    boa & minTimeLong & maxTimeLong & store_;
  }
  catch ( ... )
  {
    tstring last_error = GetLastErrorToString ();
    MessageBoxW (NULL, last_error.c_str (), _T ("Error"), MB_OK);
  }
}
void  HandMoves::Store::load (tstring filename)
{
  if ( isFileExists (filename.c_str ()) )
  {
    boost::this_thread::disable_interruption  no_interruption;
    boost::lock_guard<boost::mutex>  lock (store_mutex_);

    ifstream  ifs (filename,  std::ios_base::binary | std::ios_base::in);
    std::stringstream buffer (std::ios_base::binary | std::ios_base::in | std::ios_base::out);
    buffer << ifs.rdbuf ();
    ifs.close ();

    // ifstream  ifs (filename);         
    // boost::archive::text_iarchive     ia (ifs);
    // boost::archive::binary_iarchive   ia (ifs);
    boost::archive::binary_iarchive     bia (buffer);
    bia & minTimeLong & maxTimeLong & store_;
    //-------------------------------------------
  }
}
//------------------------------------------------------------------------------
void  HandMoves::Store::insert (const Record &rec)
{
  // if (rec.aim in store)
  // {
  //   /* Вверх класть более удачные, отн. кол-ва движение и точность попадания */
  //  
  //   /* ??? Несколько вариантов одного и того же движения ??? */
  //   // массив траекторий для каждой точки
  //
  //   ?? store.insert (rec);
  // } else
  try
  {
    boost::this_thread::disable_interruption  no_interruption;
    boost::lock_guard<boost::mutex>  lock (store_mutex_);

    auto result = store_.insert (rec);
    if ( result.second )
    {
      Hand::frames_t longs = rec.longestMusclesControl ();
      if ( !maxTimeLong && !minTimeLong )
      {
        maxTimeLong = longs;
        minTimeLong = longs;
      }
      else if ( longs > maxTimeLong )
      { maxTimeLong = longs; }
      else if ( longs < minTimeLong )
      { minTimeLong = longs; }
    }
  }
  catch ( ... )
  { MessageBox (NULL, GetLastErrorToString ().c_str (), _T (" Error "), MB_OK); }
}
//------------------------------------------------------------------------------
