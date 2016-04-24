#include "StdAfx.h"

#ifndef  _HAND_MOVES_H_
#define  _HAND_MOVES_H_
//------------------------------------------------------------------------------
#include "Record.h"
#include "WindowHeader.h"
//------------------------------------------------------------------------------
namespace HandMoves
{
  struct ByP {};
  struct ByR {};
  struct ByA {};
  struct ByX {};
  struct ByY {};
  struct ByD {};
  struct ByC {};

  typedef std::list<Record>                   adjacency_t;
  typedef std::list<std::shared_ptr<Record>>  adjacency_refs_t;
  //------------------------------------------------------------------------------
  class  ClosestPredicate
  {
    boost_point2_t aim;
  public:
    ClosestPredicate (const Point &aim) : aim (aim) {}
    double  operator() (const std::shared_ptr<Record> &a,
                        const std::shared_ptr<Record> &b)
    {
      return this->operator() (boost_point2_t (a->hit),
                               boost_point2_t (b->hit));
    }
    double  operator() (const Record &a,
                        const Record &b)
    {
      return this->operator() (boost_point2_t (a.hit),
                               boost_point2_t (b.hit));
    }
    double  operator() (const boost_point2_t &a,
                        const boost_point2_t &b)
    {
      double  da = boost::geometry::distance (aim, a);
      double  db = boost::geometry::distance (aim, b);
      return (da < db);
    }
  };
  //------------------------------------------------------------------------------
  class  ControlingHasher : std::unary_function<const controling_t&, size_t>
  {
  public:
    std::size_t operator()(const controling_t &controls) const
    {
      std::size_t seed = 0U;
      for ( Hand::Control c : controls )
      {
        boost::hash_combine (seed, boost::hash_value (c.muscle));
        boost::hash_combine (seed, boost::hash_value (c.start));
        boost::hash_combine (seed, boost::hash_value (c.last));
      }
      return seed;
    }
  };
  //------------------------------------------------------------------------------
  class  RangeInserter
  {
  public:
    void operator () (OUT adjacency_t &range, IN const Record &rec)
    { range.push_back (rec); }
    void operator () (OUT adjacency_refs_t &range, IN const Record &rec)
    { range.push_back (std::make_shared<Record> (rec)); }
  };
  //------------------------------------------------------------------------------
  using namespace boost::multi_index;
  class Store // DataBase
  {
    //------------------------------------------------------------------------------
    typedef boost::multi_index_container
    < Record,
      indexed_by < hashed_unique      < tag<ByC>,
                                        const_mem_fun < Record, const controling_t&, &Record::_get_controls>,
                                        ControlingHasher
                                      >,
                   ordered_non_unique < tag<ByP>,
                                        composite_key < Record,
                                                        const_mem_fun<Record, double, &Record::hit_x >,
                                                        const_mem_fun<Record, double, &Record::hit_y >
                                                      >
                                      >,
                   ordered_non_unique < tag<ByA>,
                                        composite_key < Record,
                                                        const_mem_fun<Record, double, &Record::aim_x >,
                                                        const_mem_fun<Record, double, &Record::aim_y >
                                                      >
                                      >,
                   ordered_non_unique < tag<ByX>, const_mem_fun<Record, double, &Record::hit_x > >,
                   ordered_non_unique < tag<ByY>, const_mem_fun<Record, double, &Record::hit_y > >,
                   ordered_non_unique < tag<ByD>, const_mem_fun<Record, double, &Record::distanceCovered > > // ,
                   // random_access      < > // доступ, как у вектору
                 >
    > MultiIndexMoves;
    //==============================================================================
    MultiIndexMoves store_;

    gradient_t  gradient;

    mutable boost::mutex    store_mutex_;
    Hand::frames_t          minTimeLong, maxTimeLong;
    //==============================================================================
  public:
    Store () : minTimeLong (0U), maxTimeLong (0U) {}
    //------------------------------------------------------------------------------
    const Record&   ClothestPoint (IN const Point &aim, IN double side)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      // -----------------------------------------------
      MultiIndexMoves::index<ByP>::type  &index = store_.get<ByP> ();
      // -----------------------------------------------
      auto iterLower = index.lower_bound (boost::tuple<double, double> (aim.x - side, aim.y - side));
      auto iterUpper = index.upper_bound (boost::tuple<double, double> (aim.x + side, aim.y + side));
      // -----------------------------------------------
      ClosestPredicate  cp (aim);
      return *std::min_element (iterLower, iterUpper, cp);
    }
    //------------------------------------------------------------------------------
    /* прямоугольная окрестность точки */
    template <class range_t, class index_t>
    size_t  adjacencyRectPoints  (OUT range_t &range, IN const Point &min, IN const Point &max)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      // -----------------------------------------------
      static_assert ( boost::is_same<range_t, adjacency_t>::value
                   || boost::is_same<range_t, adjacency_refs_t>::value,
                      "Incorrect type to template function." );
      // -----------------------------------------------
      typedef MultiIndexMoves::index<index_t>::type::const_iterator Index_cIter;
      MultiIndexMoves::index<index_t>::type  &index = store_.get<index_t> ();   
      // -----------------------------------------------
      /* Range searching, i.e.the lookup of all elements in a given interval */
      Index_cIter itFirstLower = index.lower_bound (boost::tuple<double, double> (min));
      Index_cIter itFirstUpper = index.upper_bound (boost::tuple<double, double> (max));
      // -----------------------------------------------
      RangeInserter rangeInserter;
      // -----------------------------------------------
      for ( auto it = itFirstLower; it != itFirstUpper; ++it )
      {
        if ( (min.x <= it->hit.x && min.y <= it->hit.y)
          && (max.x >= it->hit.x && max.y >= it->hit.y) )
        { rangeInserter (range, *it); }
      }
      // -----------------------------------------------
      return range.size ();
    }
    /* круглая окрестность точки */
    template <class range_t>
    size_t  adjacencyPoints      (OUT range_t &range, IN const Point &aim, IN double radius)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      // -----------------------------------------------
      static_assert ( boost::is_same<range_t, adjacency_t>::value
                   || boost::is_same<range_t, adjacency_refs_t>::value,
                      "Incorrect type to template function." );
      // -----------------------------------------------
      typedef MultiIndexMoves::index<ByP>::type::const_iterator IndexPcIter;
      MultiIndexMoves::index<ByP>::type  &index = store_.get<ByP> ();
      // -----------------------------------------------
      IndexPcIter itFirstLower = index.lower_bound (boost::make_tuple (aim.x - radius, aim.y - radius));
      IndexPcIter itFirstUpper = index.upper_bound (boost::make_tuple (aim.x + radius, aim.y + radius));
      // -----------------------------------------------
      RangeInserter  rangeInserter;
      // -----------------------------------------------
      for ( auto it = itFirstLower; it != itFirstUpper; ++it )
      { if ( boost_distance (aim, it->hit) <= radius )
        { rangeInserter (range, *it); }
      }
      // -----------------------------------------------
      return range.size ();
    }
    //------------------------------------------------------------------------------
    template <class range_t>
    size_t  similDistances       (OUT range_t &range, IN double min_distance, IN double max_distance)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      // -----------------------------------------------
      static_assert ( boost::is_same<range_t, adjacency_t>::value
                   || boost::is_same<range_t, adjacency_refs_t>::value,
                      "Incorrect type to template function." );
      // -----------------------------------------------
      typedef MultiIndexMoves::index<ByD>::type::const_iterator IndexDcIter;
      MultiIndexMoves::index<ByD>::type  &index = store_.get<ByD> ();
      // -----------------------------------------------
      IndexDcIter itFirstLower = index.lower_bound (min_distance);
      IndexDcIter itFirstUpper = index.upper_bound (max_distance);
      // -----------------------------------------------
      RangeInserter  rangeInserter;
      // -----------------------------------------------
      for ( auto it = itFirstLower; it != itFirstUpper; ++it )
      { rangeInserter (range, *it); }
      // -----------------------------------------------
      return range.size ();
    }
    //------------------------------------------------------------------------------
    template <class range_t>
    size_t  adjacencyByPBorders (OUT range_t &range, IN const Point &aim, IN double side)
    {
      adjacencyRectPoints<range_t, ByP> (range, Point (aim.x - side, aim.y - side),
                                                Point (aim.x + side, aim.y + side));
      // -----------------------------------------------
      ClosestPredicate cp (aim);
      range.sort (cp);
      // -----------------------------------------------
      int i = 0, j = 0, k = 0, l = 0;
      // -----------------------------------------------
      auto it = range.begin ();
      while ( it != range.end () )
      {
             if ( ((**it).hit.x < aim.x && (**it).hit.y < aim.y && !i) )
        { ++i; ++it;}
        else if ( ((**it).hit.x < aim.x && (**it).hit.y > aim.y && !j) )
        { ++j; ++it;}
        else if ( ((**it).hit.x > aim.x && (**it).hit.y < aim.y && !k) )
        { ++k; ++it;}
        else if ( ((**it).hit.x > aim.x && (**it).hit.y > aim.y && !l) )
        { ++l; ++it;}
        else
        { it = range.erase (it); }
      }
      // -----------------------------------------------
      return range.size ();
    }
    //------------------------------------------------------------------------------
    size_t  adjacencyByXYBorders ( IN const Point &aim, IN double side,
                                  OUT std::pair<Record, Record> &x_pair,
                                  OUT std::pair<Record, Record> &y_pair);
    //------------------------------------------------------------------------------
    const Record*  ExactRecordByControl (IN const controling_t &controls)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      // -----------------------------------------------
      MultiIndexMoves::index<ByC>::type  &index = store_.get<ByC> ();
      // -----------------------------------------------
      auto equal = index.find (controls);
      return  (equal != index.end ()) ? (&(*equal)) : (nullptr);
    }
    //------------------------------------------------------------------------------
    template <class range_t>
    size_t  FindEndPoint (OUT range_t &range, IN const Point &aim)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      // -----------------------------------------------      
      static_assert < boost::is_same<range_t, adjacency_t>::value
                   || boost::is_same<range_t, adjacency_refs_t>::value,
                      "Incorrect type to template function.">;
      // -----------------------------------------------
      MultiIndexMoves::index<ByP>::type  &index = store_.get<ByP> ();
      auto result = index.equal_range (boost::tuple<double, double> (aim));
      // -----------------------------------------------
      size_t count = 0U;
      RangeInserter rangeInserter;
      for ( auto it = result.first; it != result.second (); ++it )
      {
        rangeInserter (range, *it);
        ++count;
      }
      // -----------------------------------------------
      return  count;
    }
    //------------------------------------------------------------------------------
    void  draw (HDC hdc, gradient_t gradient, double circleRadius=0.) const;
    void  draw (HDC hdc, HPEN hPen, double circleRadius=0.) const;
    //------------------------------------------------------------------------------
    /* сериализация */
    void  save (tstring filename) const;
    void  load (tstring filename);
    //------------------------------------------------------------------------------
    void  insert (const Record &rec);
    void  insert (Hand &hand, const controling_t &controls);
    //------------------------------------------------------------------------------
    auto  begin ()       -> decltype(store_.begin ()) { return  store_.begin (); }
    auto  begin () const -> decltype(store_.begin ()) { return store_.begin (); }

    auto  end ()       -> decltype(store_.end ()) { return store_.end (); }
    auto  end () const -> decltype(store_.end ()) { return store_.end (); }
    //------------------------------------------------------------------------------
    void  clear ()
    { 
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      store_.clear ();

      minTimeLong = 0U;
      maxTimeLong = 0U;
    }
    bool  empty () const
    { 
      // boost::lock_guard<boost::mutex>  lock (store_mutex_);
      return store_.empty ();
    }
    size_t size () const
    { 
      // boost::lock_guard<boost::mutex>  lock (store_mutex_);
      return store_.size ();
    }
    //------------------------------------------------------------------------------
  };
  //------------------------------------------------------------------------------
  /* тестовые движения рукой */
  void  testRandom (Store &store, Hand &hand, size_t tries);
  void  testCover  (Store &store, Hand &hand, size_t nesting /* = 1,2,3 */);
  //------------------------------------------------------------------------------
}
BOOST_CLASS_VERSION (HandMoves::Store, 2)
//------------------------------------------------------------------------------
// #define  _DEBUG_PRINT
//------------------------------------------------------------------------------
#endif // _HAND_MOVES_H_
