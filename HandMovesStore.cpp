#include "StdAfx.h"
#include "HandMovesStore.h"


using namespace std;
using namespace HandMoves;
//---------------------------------------------------------
Record::Record (const Point         &aim,
                const Point         &hand,
                const muscles_array &muscles,
                const times_array   &times_start,
                const times_array   &times_stop,
                size_t               moves_count,
                const trajectory_t  &visited) :
  aim_ (aim), hand_ (hand), muscles_(Hand::EmptyMov),
  moves_count_ (moves_count), visited_(visited)
{
  auto emp = !times_.empty ();

  if ( moves_count_ > maxMovesCount )
    throw new exception ("Incorrect number of muscles in constructor Record"); // _T( ??

  for ( auto i = 0U; i < moves_count; ++i )
  {
    muscles_ = muscles_ | muscles[i];
    times_[muscles[i]] = std::make_pair (static_cast<time_t>(times_start[i] - times_start[0]),
                                         static_cast<time_t>(times_stop [i] - times_start[0]));
  }

  if ( !validateMusclesTimes () )
    throw new exception ("Invalid muscles constructor Record parameter"); // _T( ??

  elegance_ = Elegance ();
  distance_ = boost::geometry::distance (boost_point2_t (hand_),
                                         boost_point2_t (aim_));
}

double  Record::Elegance ()
{ return 0.; }

void  Record::makeHandMove (Hand &hand, const Point &aim)
{
  for ( size_t i = 0; i < moves_count_; ++i )
  {

  }
}

bool  Record::validateMusclesTimes ()
{
  if ( times_.size () > 1U )
  {
    /* Каждый с каждым - n^2 !!! TODO !!!  */
    for ( muscle_times_t::iterator iti = times_.begin (); iti != times_.end (); ++iti )
      for ( muscle_times_t::iterator itj = std::next (iti); itj != times_.end (); ++itj )
        /* Если есть перекрытие по времени */
        if ( ((iti->second.first <= itj->second.first) && (iti->second.second >= itj->second.first))
          || ((itj->second.first <= iti->second.first) && (itj->second.second >= iti->second.first)) )
        {
          for ( auto j : Hand::joints )
          {
            Hand::MusclesEnum  Opn = muscleByJoint (j, true);
            Hand::MusclesEnum  Сls = muscleByJoint (j, false);
            /* Одновременно работающие противоположные мышцы */
            if ( (Opn & iti->first) && (Сls & itj->first) )
            { return false; } // end if
          } // end for
        } // end if
  } //end if
    
  // if ( (Opn & muscles_) && (Сls & muscles_) )
  // {
  //   if ( ((times_[Opn].first <= times_[Сls].first) && (times_[Opn].second >= times_[Сls].first))
  //     || ((times_[Сls].first <= times_[Opn].first) && (times_[Сls].second >= times_[Opn].first)) )
  //     return false;
  // }
  
  return true;
}
//---------------------------------------------------------
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
//---------------------------------------------------------
/* круглая окрестность точки */
//size_t  HandMoves::adjacencyPoints (Store &store, std::list<Record> &range,
//                                    const Point &center, double radius)
//{
//  typedef Store::index<Record::ByP>::type::const_iterator StorePcIter;
//  Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();
//
//  StorePcIter itFirstLower = index.lower_bound (boost::make_tuple (center.x - radius,
//                                                                   center.y - radius));
//  StorePcIter itFirstUpper = index.upper_bound (boost::make_tuple (center.x + radius,
//                                                                   center.y + radius));
//  
//  // auto range_it = std::back_inserter (range);
//  size_t count = 0U;
//  for ( auto it = itFirstLower; it != itFirstUpper; ++it )
//  {
//    if ( boost::geometry::distance (boost_point2_t (center),
//                                    boost_point2_t (it->aim)) <= radius )
//    {
//      range.push_back (*it);
//      // *range_it = (pointer_type) ? (make_shared<Record> (*it)) : (*it); ++range_it;
//      ++count;
//    }
//  }
//  return count;
//}
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
//---------------------------------------------------------
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

//---------------------------------------------------------
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

//---------------------------------------------------------
void  store_test ()
{
  //double fx = 0.1; // find x
  //double fy = 0.2; // find y
  //
  //auto XRange = adjacencyHorizontalPoints (store, fy);
  //for ( auto it = XRange.begin (); it != XRange.end (); ++it)
  //  cout << (*it).x << (*it).y << (*it).comment << endl;
  //
  //auto YRange = adjacencyHorizontalPoints (store, fx);
  //for ( auto it = YRange.begin (); it != YRange.end (); ++it )
  //  cout << (*it).x << (*it).y << (*it).comment << endl;
  //
  //Store::index<Record::ByX>::type::const_iterator itx = store.get<Record::ByX> ().find (fx);
  //if ( itx != lsx.end () )
  //{ cout << (*itx).comment << endl; }
  //
  //// CHANGING
  //auto &np = store.get<Record::ByP> ();
  //auto &nx = store.get<Record::ByX> ();
  //
  //auto nit = nx.find (fx);
  //if ( nit != nx.end () )
  //{
  //  auto pit = store.project<Record::ByP> (nit);
  //  // convert iterator
  //  Point p (0.7, 0.8);
  //  np.modify (pit, Record::ChangePoint (p));
  //}
  //
  //{
  //  Point *result = new Point ();
  //  Store::index<Record::ByP>::type::key_from_value::result_type find_id = //Store::index<Record::ByP>::type::key_from_value ()(rec);
  //  // tuple<string, string> find_tuple = make_tuple(rec.x, rec.y);
  //  // создаёт объект ключа, извлекая параметры из переданной записи
  //  // tuple сравним с key_from_value::result_type
  //  if ( store.get<Record::ByP> ().find (find_id) == store.get<Record::ByP> ().end () )
  //    store.insert (rec);
  //}
  //
  //{
  //  typedef Store::index<Record::ByP>::type StoreByPType;
  //  StoreByPType& index = store.get<Record::ByP> ();
  //  StoreByPType::iterator it = index.find (boost::make_tuple(0.,1.));
  //  index.modify (it, Record::ChangePoint ({0.,0.}));
  //}

}

//------------------------------------------------------------------------------
void  HandMoves::storeSave (const Store& store, const TCHAR *filename)
{
  ofstream  ofs (filename);
  boost::archive::text_oarchive oa (ofs);
  oa << store;
}
void  HandMoves::storeLoad (      Store& store, const TCHAR *filename)
{
  if ( isFileExists (filename) )
  {
    ifstream  ifs (filename);
    boost::archive::text_iarchive ia (ifs);
    ia >> store;
  }
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//hs*   HandMoves::Get (const Point *aim)
//{
//  hm_map::iterator  hm_it = hmoves.find (aim);
//  return (hm_it != hmoves.cend () && hm_it->second.size ()) ?
//    &(*hm_it->second.begin ()) : NULL;
//}
//
//
//void  HandMoves::Set (const hs     &Hs)
//{ hm_map::iterator hm_it = hmoves.find (&Hs.aim_);
//  if ( hm_it == hmoves.end () )
//    return;
//
//  hm_list &list = hm_it->second;
//  if ( !list.size () )
//    list.push_back (Hs);
//  else
//    for ( hm_list::iterator it = list.begin ();
//  it != list.end (); ++it )
//      if ( it->exactHit_ > Hs.exactHit_ )
//        list.insert (it, Hs);
//}
//------------------------------------------------------------------------------
