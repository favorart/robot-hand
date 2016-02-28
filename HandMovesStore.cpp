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
void  HandMoves::storeSave (const Store& store, tstring filename)
{
  ofstream  ofs (filename, std::ios_base::binary | std::ios_base::out);
  // boost::archive::text_oarchive  oa (ofs);
  boost::archive::binary_oarchive   oa (ofs);
  oa << store;
}
void  HandMoves::storeLoad (      Store& store, tstring filename)
{
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

  store.insert (rec);
}
//------------------------------------------------------------------------------
