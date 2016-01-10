#include "StdAfx.h"
#include "HandMovesStore.h"

using namespace std;
using namespace HandMoves;


bool  muscle_time_validate (const Hand::MusclesEnum muscle,
                            const uint_t           *times_start,
                            const uint_t           *times_stop)
{
  for ( auto j : Hand::joints )
  {
    Hand::MusclesEnum  Opn = Hand::muscleByJoint (j, true);
    Hand::MusclesEnum  Сls = Hand::muscleByJoint (j, false);
    /* Одновременно работающие мышцы */
    if ( (Opn & muscle) && (Сls & muscle) )
    {
      size_t  ic = Hand::muscleIndex (Сls);
      size_t  io = Hand::muscleIndex (Opn);
      if ( ((times_start[io] <= times_start[ic]) && (times_stop[io] >= times_start[ic]))
        || ((times_start[ic] <= times_start[io]) && (times_stop[ic] >= times_start[io])) )
        return false;
    }
  }
  return true;
}
//------------------------------------------------------------------------------
Record::Record (const Point        &aim,
                const Point        &hand,
                Hand::MusclesEnum   muscles,
                const uint_t        *times_start,
                const uint_t        *times_stop) :
  aim_ (aim), hand_ (hand), muscles_ (muscles), moves_count_ (0)
{
  if ( !muscle_time_validate (muscles, times_start, times_stop) )
    throw new exception ("Invalid muscles constructor Record parameter");

  std::memcpy (times_start_, 0, maxMovesCount * sizeof (*times_start_));
  std::memcpy (times_stop_,  0, maxMovesCount * sizeof (*times_stop_));

  for ( auto m : Hand::muscles )
  {
    if ( muscles & m )
      ++moves_count_;
  }
  if ( moves_count_  > maxMovesCount )
    throw new exception ("Incorrect number of muscles in constructor Record");

  std::memcpy (times_start_, times_start, moves_count_ * sizeof (*times_start_));
  std::memcpy (times_stop_,  times_stop,  moves_count_ * sizeof (*times_stop_));

  distance_ = boost::geometry::distance (boost_point2_t (hand_), 
                                         boost_point2_t (aim_));
  elegance_ = Elegance ();
}

//------------------------------------------------------------------------------
// bool  Control (uint_t no_mov, uint_t hyd) const
// { return control_[hyd * no_mov]; }
// //------------------------------------------
// friend std::istream&  operator>> (std::istream &in, HandStatus &hs);
// friend std::ostream&  operator<< (std::ostream &out, const HandStatus &hs);

//std::istream&  operator>> (std::istream &in, Record &rec)
//{ in >> Hs.movesCount_ >> Hs.exactHit_ >> Hs.aim_;
//  for ( uint_t i = 0U; i < Hs.movesCount_; ++i )
//    in >> Hs.initTime_[i];
//  in >> Hs.control_;
//  return in;
//}
//std::ostream&  operator<< (std::ostream &out, const hs &Hs)
//{ out << std::endl << Hs.movesCount_ << ' ' << Hs.exactHit_ << Hs.aim_;
//  for ( uint_t i = 0U; i < Hs.movesCount_; ++i )
//    out << Hs.initTime_[i] << " , ";
//  out << Hs.control_ << std::endl;
//  return out;
//}
////------------------------------------------------------------------------------
//HandMoves::HandMoves (Target *T) : tg_ (T)
//{ Target::set_t::iterator it;
//  for ( it = tg_->coords_.begin (); it != tg_->coords_.end (); ++it )
//  { hm_list list;
//    hmoves.insert (hm_map::value_type (&(*it), list));
//  }
//}
//
////------------------------------------------------------------------------------
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



//void  updatePoint (Store &store, const Point &p)
//{
//  auto pit = store.project<Record::ByP> (nit);
//  // convert iterator
//  np.modify (pit, Record::ChangePoint (p));
//}
//---------------------------------------------------------
//Point*  getKeyPoint (Store &store, const Record &rec)
//{
//  Point *result = new Point ();
//
//  Store::index<Record::ByP>::type::key_from_value::result_type find_id = Store::index<Record::ByP>::type::key_from_value ()(rec);
//  // tuple<string, string> find_tuple = make_tuple(rec.x, rec.y);
//  // создаёт объект ключа, извлекая параметры из переданной записи
//  // tuple сравним с key_from_value::result_type
//  if ( store.get<Record::ByP> ().find (find_id) == store.get<Record::ByP> ().end () )
//    store.insert (rec);
//
//  return result;
//}

// typedef employee_set::index<name>::type employee_set_by_name;
// employee_set_by_name& name_index = es.get<name> ();
// employee_set_by_name::iterator it = name_index.find ("Anna Jones");
// name_index.modify (it, change_name ("Anna Smith"));


//---------------------------------------------------------
// прямоугольная окрестность точки
template<typename T>
uint_t  adjacencyRectPoints (Store &store,
                             std::back_insert_iterator<T>
                             /* std::list<std::shared_ptr<Point>>::iterator */ &range_it,
                             const boost::tuple<double, double> &left_down,
                             const boost::tuple<double, double> &right_up,
                             bool  pointer_type = false)
{
  typedef Store::index<Record::ByP>::type::const_iterator StorePcIter;
  Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();
  // Range searching, i.e.the lookup of all elements in a given interval
  StorePcIter itFirstLower = index.lower_bound (left_down);
  StorePcIter itFirstUpper = index.upper_bound (right_up);
  // return make_pair (itFirstLower, itFirstUpper);

  // for ( auto it = itFirstLower; it != itFirstUpper; ++it )
  //   std::cout << it->x << ' ' << it->y << ' ' << it->comment << std::endl;

  uint_t count = 0U;
  // std::copy (itFirstLower, itFirstUpper, range_it);
  for ( auto it = itFirstLower; it != itFirstUpper; ++it )
  {
     *range_it = (pointer_type) ? (make_shared<Record> (*it)) : (*it);
    ++range_it;
    ++count;
  }
  return count;
}
//---------------------------------------------------------
// круглая окрестность точки
template<typename T>
uint_t  adjacencyPoints (Store &store,
                         std::back_insert_iterator<T>
                         /* std::list<std::shared_ptr<Point>>::iterator */ &range_it,
                         const boost_point2_t &center,
                         double radius,
                         bool   pointer_type=false)
{
  typedef Store::index<Record::ByP>::type::const_iterator StorePcIter;
  Store::index<Record::ByP>::type& index = store.get<Record::ByP> ();

  StorePcIter itFirstLower = index.lower_bound (boost::make_tuple (center.x () - radius, center.y () - radius));
  StorePcIter itFirstUpper = index.upper_bound (boost::make_tuple (center.x () + radius, center.y () + radius));

  for ( auto it = itFirstLower; it != itFirstUpper; ++it )
    std::cout << it->x << ' ' << it->y << ' ' << it->comment << std::endl;

  uint_t count = 0U;
  // std::copy (itFirstLower, itFirstUpper, range_it);
  for ( auto it = itFirstLower; it != itFirstUpper; ++it )
  {
    if ( boost::geometry::distance (center, boost_point2_t (*it)) <= radius )
    {
       *range_it = (pointer_type) ? (std::make_shared<Record> (*it)) : (*it);
      ++range_it;
      ++count;
    }
  }
  return count;

}
//---------------------------------------------------------
//// Все точки с данным x 
//void  adjacencyYsByXPoints (Store &store, std::list<Point> &range,
//                            double x, double up = 0., double down = 0.)
//{
//  typedef Store::index<Record::ByX>::type::iterator StoreXiter;
//  Store::index<Record::ByX>::type& index = store.get<Record::ByX> ();
//
//  StoreXiter itFirstLower = index.lower_bound (up);
//  StoreXiter itFirstUpper = index.upper_bound (down);
//
//  // Store::index<Record::ByX>::type::const_iterator ity = lsy.find (x);
//  // if ( ity != lsy.end () ) {}
//  std::copy (itFirstLower, itFirstUpper, std::back_inserter (range));
//}
//
////---------------------------------------------------------
//// Все точки с данным y 
//void  adjacencyXsByYPoints (Store &store, std::list<Point> &range,
//                            double y, double left = 0., double right = 0.)
//{
//  typedef Store::index<Record::ByY>::type::iterator StoreYiter;
//  Store::index<Record::ByY>::type& index = store.get<Record::ByY> ();
//
//  StoreYiter itFirstLower = index.lower_bound (left);
//  StoreYiter itFirstUpper = index.upper_bound (right);
//
//  // Store::index<Record::ByY>::type::const_iterator ity = lsy.find (fy);
//  // if ( ity != lsy.end () ) {}
//  std::copy (itFirstLower, itFirstUpper, std::back_inserter (range));
//}
//
////---------------------------------------------------------
//
//void  HandMoves::store_test ()
//{
//  Store store;
//
//  int n = 20;
//  while ( n-- )
//  {
//    Record h1 (0.1 * n, 0.2, std::string ("comment") + std::to_string (n));
//    store.insert (h1);
//    // std::cout << "ok  " << store.insert (h1).second << endl;
//  }
//
//  for ( auto rec : store )
//  { cout << std::string (rec) << endl; }
//
//  std::list<std::shared_ptr<Record>> range;
//  adjacencyRectPoints<decltype(range)> (store, std::back_inserter (range), { 0.1, 0.1 }, { 0.4, 0.3 });
//
//  for ( auto p : range )
//  {
//    cout << *p << endl;
//  }
//
//  std::list<Point> range1;
//  adjacencyPoints<decltype(range1)> (store, std::back_inserter (range1), { 0.1, 0.1 }, 0.1);
//
//  for ( auto pt : range1 )
//    cout << pt << " ";
//  cout << endl;
//
//  double fx = 0.1; // find x
//  double fy = 0.2; // find y
//                   //
//                   //  auto XRange = adjacencyHorizontalPoints (store, fy);
//                   //  for ( auto it = XRange.begin (); it != XRange.end (); ++it)
//                   //    cout << (*it).x << (*it).y << (*it).comment << endl;
//                   //
//                   //  //auto YRange = adjacencyHorizontalPoints (store, fx);
//                   //  //for ( auto it = YRange.begin (); it != YRange.end (); ++it )
//                   //  //  cout << (*it).x << (*it).y << (*it).comment << endl;
//                   //
//                   //  //// Store::index<Record::ByX>::type::const_iterator itx = store.get<Record::ByX> ().find (fx);
//                   //  //// if ( itx != lsx.end () )
//                   //  //// { cout << (*itx).comment << endl; }
//                   //
//                   //
//                   //  //// CHANGING
//                   //  //auto &np = store.get<Record::ByP> ();
//                   //  //auto &nx = store.get<Record::ByX> ();
//                   //
//                   //  //auto nit = nx.find (fx);
//                   //  //if ( nit != nx.end () )
//                   //  //{
//                   //  //  auto pit = store.project<Record::ByP> (nit);
//                   //  //  // convert iterator
//                   //  //  Point p (0.7, 0.8);
//                   //  //  np.modify (pit, Record::ChangePoint (p));
//                   //  //}
//                   //
//  return;
//}

//------------------------------------------------------------------------------
void  save (const Store& store)
{
  std::ofstream  ofs ("moves.bin");
  boost::archive::text_oarchive oa (ofs);
  oa << store;
}

void  load (Store& store)
{
  std::ifstream  ifs ("moves.bin");
  boost::archive::text_iarchive ia (ifs);
  ia >> store;
}

// employee_set es;
// ... // fill it with data
// save (es);

// employee_set restored_es;
// load (restored_es);