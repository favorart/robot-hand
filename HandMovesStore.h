#include "StdAfx.h"

#ifndef  _HAND_MOVES_H_
#define  _HAND_MOVES_H_
//------------------------------------------------------------------------------
#include "Record.h"
#include "MyWindow.h"
//------------------------------------------------------------------------------
namespace HandMoves
{
  struct ByP {};
  struct ByA {};
  struct ByX {};
  struct ByY {};
  struct ByD {};
  struct ByC {};

  using namespace ::boost::multi_index;

  class Store // DataBase
  {
    struct ControlingHasher : std::unary_function<const controling_t&, size_t>
    {
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
    typedef boost::multi_index_container
    < Record,
      indexed_by < hashed_unique      < tag<ByC>,
                                        const_mem_fun<Record, const controling_t&, &Record::controls>,
                                        ControlingHasher
                                      >,
                   ordered_non_unique < tag<ByP>,
                                        composite_key < Record,
                                                        const_mem_fun<Record, double, &Record::hit_x>,
                                                        const_mem_fun<Record, double, &Record::hit_y>
                                                      >
                                      >,
                   ordered_non_unique < tag<ByA>,
                                        composite_key < Record,
                                                        const_mem_fun<Record, double, &Record::aim_x>,
                                                        const_mem_fun<Record, double, &Record::aim_y>
                                                      >
                                      >,
                   // ordered_non_unique < tag<ByX>, const_mem_fun<Record, double, &Record::hit_x> >,
                   // ordered_non_unique < tag<ByY>, const_mem_fun<Record, double, &Record::hit_y> >,
                   ordered_non_unique < tag<ByD>, const_mem_fun<Record, double, &Record::distanceCovered> > // ,
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
    class RangeInserter
    {
    public:
      void operator () (std::list<Record> &range, const Record &rec)
      { range.push_back (rec); }
      void operator () (std::list<std::shared_ptr<Record>> &range, const Record &rec)
      { range.push_back (std::make_shared<Record> (rec)); }
    };
    //------------------------------------------------------------------------------
    /* прямоугольная окрестность точки */
    template <class T>
    size_t  adjacencyRectPoints (std::list<T> &range, const Point &left_down, const Point &right_up)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      
      typedef MultiIndexMoves::index<ByP>::type::const_iterator IndexPcIter;
      MultiIndexMoves::index<ByP>::type  &index = store_.get<ByP> ();
      /* Range searching, i.e.the lookup of all elements in a given interval */
      IndexPcIter itFirstLower = index.lower_bound (boost::tuple<double, double> (left_down));
      IndexPcIter itFirstUpper = index.upper_bound (boost::tuple<double, double> (right_up));
    
      size_t count = 0U;
      RangeInserter rangeInserter;
      for ( auto it = itFirstLower; it != itFirstUpper; ++it )
      {
        auto &rec = *it;
        rangeInserter (range, rec);
        // range.push_back (*it); // range.push_back ( std::make_shared<Record> (*it) );
        ++count;
      }
      return count;
    }
    /* круглая окрестность точки */
    template <class T>
    size_t  adjacencyPoints (std::list<T> &range, const Point &center, double radius)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      
      typedef MultiIndexMoves::index<ByP>::type::const_iterator IndexPcIter;
      MultiIndexMoves::index<ByP>::type  &index = store_.get<ByP> ();
    
      IndexPcIter itFirstLower = index.lower_bound (boost::make_tuple (center.x - radius,
                                                                       center.y - radius));
      IndexPcIter itFirstUpper = index.upper_bound (boost::make_tuple (center.x + radius,
                                                                       center.y + radius));
      size_t count = 0U;
      RangeInserter  rangeInserter;
      for ( auto it = itFirstLower; it != itFirstUpper; ++it )
      {
        auto &rec = *it;
        if ( boost::geometry::distance (boost_point2_t (center),
                                        boost_point2_t (rec.hit)) <= radius )
        {
          rangeInserter (range, rec);
          // range.push_back (*it); // range.push_back ( make_shared<Record> (*it) );
          ++count;
        }
      }
      return count;
    }
    //------------------------------------------------------------------------------
    template <class T>
    size_t  similDistances  (std::list<T> &range, double min_distance, double max_distance)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      
      typedef MultiIndexMoves::index<ByD>::type::const_iterator IndexDcIter;
      MultiIndexMoves::index<ByD>::type  &index = store_.get<ByD> ();
    
      IndexDcIter itFirstLower = index.lower_bound (min_distance);
      IndexDcIter itFirstUpper = index.upper_bound (max_distance);
    
      size_t count = 0U;
      RangeInserter  rangeInserter;
      for ( auto it = itFirstLower; it != itFirstUpper; ++it )
      {
        auto &rec = *it;
        rangeInserter (rec);
        ++count;
      }
      return count;
    }
    //------------------------------------------------------------------------------
    /* Все точки с данным x | y */
    void  adjacencyYsByXPoints (std::list<Record> &range, double x,
                                double up = 0., double down =0.) const;
    void  adjacencyXsByYPoints (std::list<Record> &range, double y,
                                double left=0., double right=0.) const;
    //------------------------------------------------------------------------------
    const Record*  ExactRecordByControl (controling_t controls)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      
      MultiIndexMoves::index<ByC>::type  &index = store_.get<ByC> ();
      auto equal = index.find (controls);
      return  (equal != index.end ()) ? (&(*equal)) : (nullptr);
    }
    //------------------------------------------------------------------------------
    template <class T>
    size_t  FindEndPoint (std::list<T> &range, const Point &point)
    {
      boost::lock_guard<boost::mutex>  lock (store_mutex_);

      MultiIndexMoves::index<ByP>::type  &index = store_.get<ByP> ();
      auto result = index.equal_range (boost::tuple<double, double> (point));

      size_t count = 0U;
      RangeInserter rangeInserter;
      for ( auto it = result.first; it != result.second (); ++it )
      {
        rangeInserter (range, *it);
        ++count;
      }
      return  count;
    }
    //------------------------------------------------------------------------------
    void  draw (HDC hdc, gradient_t gradient) const;
    void  draw (HDC hdc, double circleRadius, HPEN hPen) const;
    //------------------------------------------------------------------------------
    /* сериализация */
    void  save (tstring filename) const;
    void  load (tstring filename);
    //------------------------------------------------------------------------------
    void  insert (const Record &rec);
    //------------------------------------------------------------------------------
    void  clear ()
    { 
      boost::lock_guard<boost::mutex>  lock (store_mutex_);
      store_.clear ();

      minTimeLong = 0U;
      maxTimeLong = 0U;
    }
    bool  empty ()
    { 
      // boost::lock_guard<boost::mutex>  lock (store_mutex_);
      return store_.empty ();
    }
    size_t size ()
    { 
      // boost::lock_guard<boost::mutex>  lock (store_mutex_);
      return store_.size ();
    }
    //------------------------------------------------------------------------------
    // void  uncoveredTargetPoints (IN const RecTarget &target, OUT std::list<Point> &uncovered)
    // {
    //   boost::lock_guard<boost::mutex>  lock (store_mutex_);
    // 
    //   // std::list<std::shared_ptr<Record>> range;
    //   // adjacencyPoints (range, target.Min (), target.Max ());
    //  
    //   // MultiIndexMoves::index<ByP>::type  &index = store_.get<ByP> ();
    //   for ( auto &pt : target.coords () )
    //   {
    //     std::list<std::shared_ptr<Record>> exact;
    //     adjacencyPoints (exact, pt, target.precision ());
    // 
    //     if ( exact.empty )
    //     // if ( index.find (boost::tuple<double, double> (pt)) == index.end () )
    //     { uncovered.push_back (pt); }
    //   }
    // }
  };
  //------------------------------------------------------------------------------
  class ClosestPredicate
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
  /* тестовые движения рукой */
  void  test_random (Store &store, Hand &hand, size_t tries);
  void  test_cover  (Store &store, Hand &hand, size_t nesting /* = 1,2,3 */);
  //------------------------------------------------------------------------------
  // void  testCover (Store &store, Hand &hand, Hand::MusclesEnum muscles, \
                       std::list<std::shared_ptr<std::list<Point>>> &trajectories);
  
  // void  getTargetCenter (Hand &hand, Point &center);
  // void  testCoverTarget (Store &store, Hand &hand, RecTarget &target);
  //------------------------------------------------------------------------------
}
BOOST_CLASS_VERSION (HandMoves::Store, 2)

#include "Position.h"

#endif // _HAND_MOVES_H_
