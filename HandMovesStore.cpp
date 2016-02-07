#include "StdAfx.h"
#include "HandMovesStore.h"

using namespace std;
using namespace HandMoves;
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
                                    boost_point2_t (it->aim)) <= radius )
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
                                    boost_point2_t (it->aim)) <= radius )
    { range.push_back (make_shared<Record> (*it));
      ++count;
    }
  }
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
  ofstream  ofs (filename);
  boost::archive::text_oarchive oa (ofs);
  oa << store;
}
void  HandMoves::storeLoad (      Store& store, tstring filename)
{
  if ( isFileExists (filename.c_str ()) )
  {
    ifstream  ifs (filename);
    boost::archive::text_iarchive ia (ifs);
    ia >> store;
  }
}
//------------------------------------------------------------------------------
