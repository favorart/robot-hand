#include "StdAfx.h"

#include <functional>

#ifndef  _ROBO_MOVES_STORE_H_
#define  _ROBO_MOVES_STORE_H_
//------------------------------------------------------------------------------
#include "RoboMovesRecord.h"
#include "WindowHeader.h"
#include "WindowDraw.h"

#include "nanoflann.hpp"
//------------------------------------------------------------------------------
namespace RoboMoves
{
  struct ByP {};
  struct ByR {};
  struct ByA {};
  struct ByX {};
  struct ByY {};
  struct ByD {};
  struct ByC {};

  typedef std::list<Record>                   adjacency_t;
  typedef std::list<const Record*>            adjacency_ptrs_t;
  typedef std::list<std::shared_ptr<Record>>  adjacency_sh_ptrs_t;
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
    double  operator() (const Record *a,
                        const Record *b)
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
  class  ControlHasher : std::unary_function<const Robo::Control&, size_t>
  {
  public:
    std::size_t operator()(const Robo::Control &controls) const
    {
      std::size_t seed = 0U;
      for (auto& c : controls)
      {
        boost::hash_combine(seed, boost::hash_value (c.muscle));
        boost::hash_combine(seed, boost::hash_value (c.start));
        boost::hash_combine(seed, boost::hash_value (c.last));
      }
      return seed;
    }
  };
  //------------------------------------------------------------------------------
  class  RangeInserter
  {
  public:
    void operator()(OUT adjacency_t &range, IN const Record &rec)
    { range.push_back(rec); }
    void operator()(OUT adjacency_ptrs_t &range, IN const Record &rec)
    { range.push_back(/* (Record*) */(&rec)); }
    void operator()(OUT adjacency_sh_ptrs_t &range, IN const Record &rec)
    { range.push_back(std::make_shared<Record>(rec)); }
  };
  //------------------------------------------------------------------------------
  using namespace boost::multi_index;
  /// Robot Moves DataBase
  class Store
  {
    //------------------------------------------------------------------------------
    using MultiIndexMoves = boost::multi_index_container
    < Record,
      indexed_by < hashed_unique      < tag<ByC>,
                                        const_mem_fun < Record, const Robo::Control&, &Record::getControls>,
                                        ControlHasher
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
                   ordered_non_unique < tag<ByD>, const_mem_fun<Record, double, &Record::distanceCovered > >
                   // , random_access      < > // доступ, как у вектору
                 >
    >;

    //using InverseIndex = std::unordered_multimap < std::reference_wrapper<const Point>,
    //                                               std::reference_wrapper<const Record>,
    //                                               PointHasher,
    //                                               std::equal_to<Point> > ;

    //==============================================================================
    
    using InverseIndex = std::vector<std::pair<const Point*, const Record*>>;
    
    MultiIndexMoves _store{};
    mutable boost::mutex _store_mutex{};
    InverseIndex _inverse{};

    //==============================================================================
    //inline size_t kdtree_get_point_count() const { return _inverse.size(); }
    //
    //// Returns the dim'th component of the idx'th point in the class:
    //// Since this is inlined and the "dim" argument is typically an immediate value, the
    ////  "if/else's" are actually solved at compile time.
    //inline double kdtree_get_pt(const int idx, int dim) const
    //{ return (dim) ? _inverse[idx].first.get().y : _inverse[idx].first.get().x; }
    //    
    //// Optional bounding-box computation: return false to default to a standard bbox computation loop.
    ////   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    ////   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    //template <class BBOX>
    //bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
    //
    //// construct a kd-tree index:
    
    using KDTree = nanoflann::KDTreeSingleIndexDynamicAdaptor< nanoflann::L2_Simple_Adaptor<double, Store>, Store, 2 /* dim */ >;
    KDTree _inverse_kdtree{ 2, _store, nanoflann::KDTreeSingleIndexAdaptorParams(10) };
    
    //friend class KDTree;
    //friend class KDTreeSingleIndexDynamicAdaptor_;
    //==============================================================================
    friend class boost::serialization::access;
    //BOOST_SERIALIZATION_SPLIT_MEMBER()
    template <class Archive>
    void serialize(Archive &ar, unsigned version)
    { ar & _store; }

  public:
    Store() = default;
    Store(Store&&) = delete;
    Store(const Store&) = delete;
    Store(const tstring &database) : Store() { pick_up(database); }
    //------------------------------------------------------------------------------
    const Record& ClothestPoint(IN const Point &aim, IN double side) const
    {
      boost::lock_guard<boost::mutex> lock(_store_mutex);
      // -----------------------------------------------
      const MultiIndexMoves::index<ByP>::type &index = _store.get<ByP>();
      // -----------------------------------------------
      auto iterLower = index.lower_bound(boost::tuple<double, double> (aim.x - side, aim.y - side));
      auto iterUpper = index.upper_bound(boost::tuple<double, double> (aim.x + side, aim.y + side));
      // -----------------------------------------------
      ClosestPredicate cp(aim);
      return *std::min_element(iterLower, iterUpper, cp);
    }
    //------------------------------------------------------------------------------
    /* прямоугольная окрестность точки */
    template <class range_t, class index_t>
    size_t  adjacencyRectPoints (OUT range_t &range, IN const Point &min, IN const Point &max) const
    {
      boost::lock_guard<boost::mutex> lock(_store_mutex);
      // -----------------------------------------------
      static_assert ( boost::is_same<range_t, adjacency_t>::value
                   || boost::is_same<range_t, adjacency_ptrs_t>::value
                   || boost::is_same<range_t, adjacency_sh_ptrs_t>::value,
                      "Incorrect type to template function." );
      // -----------------------------------------------
      typedef MultiIndexMoves::index<index_t>::type::const_iterator Index_cIter;
      const MultiIndexMoves::index<index_t>::type &index = _store.get<index_t> ();   
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
    size_t adjacencyPoints(OUT range_t &range, IN const Point &aim, IN double radius) const
    {
        boost::lock_guard<boost::mutex> lock(_store_mutex);
        // -----------------------------------------------
        static_assert (boost::is_same<range_t, adjacency_t>::value
                       || boost::is_same<range_t, adjacency_ptrs_t>::value
                       || boost::is_same<range_t, adjacency_sh_ptrs_t>::value,
                       "Incorrect type to template function.");
        // -----------------------------------------------
        typedef MultiIndexMoves::index<ByP>::type::const_iterator IndexPcIter;
        const MultiIndexMoves::index<ByP>::type &index = _store.get<ByP>();
        // -----------------------------------------------
        IndexPcIter itFirstLower = index.lower_bound(boost::make_tuple(aim.x - radius, aim.y - radius));
        IndexPcIter itFirstUpper = index.upper_bound(boost::make_tuple(aim.x + radius, aim.y + radius));
        // -----------------------------------------------
        RangeInserter rangeInserter;
        // -----------------------------------------------
        for (auto it = itFirstLower; it != itFirstUpper; ++it)
            if (boost_distance(aim, it->hit) <= radius)
            { rangeInserter(range, *it); }
        // -----------------------------------------------
        return range.size();
    }
    //------------------------------------------------------------------------------
    template <class range_t>
    size_t similDistances(OUT range_t &range, IN double min_distance, IN double max_distance) const
    {
        boost::lock_guard<boost::mutex> lock(_store_mutex);
        // -----------------------------------------------
        static_assert (boost::is_same<range_t, adjacency_t>::value
                       || boost::is_same<range_t, adjacency_ptrs_t>::value
                       || boost::is_same<range_t, adjacency_sh_ptrs_t>::value,
                       "Incorrect type to template function.");
        // -----------------------------------------------
        typedef MultiIndexMoves::index<ByD>::type::const_iterator IndexDcIter;
        MultiIndexMoves::index<ByD>::type  &index = _store.get<ByD>();
        // -----------------------------------------------
        IndexDcIter itFirstLower = index.lower_bound(min_distance);
        IndexDcIter itFirstUpper = index.upper_bound(max_distance);
        // -----------------------------------------------
        RangeInserter  rangeInserter;
        // -----------------------------------------------
        for (auto it = itFirstLower; it != itFirstUpper; ++it)
        { rangeInserter(range, *it); }
        // -----------------------------------------------
        return range.size();
    }
    //------------------------------------------------------------------------------
    template <class range_t>
    size_t adjacencyByPBorders(OUT range_t &range, IN const Point &aim, IN double side) const
    {
        adjacencyRectPoints<range_t, ByP>(range, Point(aim.x - side, aim.y - side),
                                                 Point(aim.x + side, aim.y + side));
        // -----------------------------------------------
        ClosestPredicate cp(aim);
        range.sort(cp);
        // -----------------------------------------------
        int i = 0, j = 0, k = 0, l = 0;
        // -----------------------------------------------
        auto it = range.begin();
        while (it != range.end())
        {
            if (((**it).hit.x < aim.x && (**it).hit.y < aim.y && !i))
            { ++i; ++it; }
            else if (((**it).hit.x < aim.x && (**it).hit.y > aim.y && !j))
            { ++j; ++it; }
            else if (((**it).hit.x > aim.x && (**it).hit.y < aim.y && !k))
            { ++k; ++it; }
            else if (((**it).hit.x > aim.x && (**it).hit.y > aim.y && !l))
            { ++l; ++it; }
            else
            { it = range.erase(it); }
        }
        // -----------------------------------------------
        return range.size();
    }
    //------------------------------------------------------------------------------
    size_t adjacencyByXYBorders(IN  const Point &aim, IN double side,
                                OUT std::pair<Record, Record> &x_pair,
                                OUT std::pair<Record, Record> &y_pair) const;
    //------------------------------------------------------------------------------
    const Record* exactRecordByControl(IN const Robo::Control &controls) const
    {
        boost::lock_guard<boost::mutex>  lock(_store_mutex);
        // -----------------------------------------------
        const MultiIndexMoves::index<ByC>::type  &index = _store.get<ByC>();
        // -----------------------------------------------
        auto equal = index.find(controls);
        return  (equal != index.end()) ? (&(*equal)) : (nullptr);
    }
    //------------------------------------------------------------------------------
    template <class range_t>
    size_t findEndPoint(OUT range_t &range, IN const Point &aim) const
    {
        boost::lock_guard<boost::mutex>  lock(_store_mutex);
        // -----------------------------------------------
        static_assert (boost::is_same<range_t, adjacency_t>::value
                       || boost::is_same<range_t, adjacency_ptrs_t>::value
                       || boost::is_same<range_t, adjacency_sh_ptrs_t>::value,
                       "Incorrect type to template function.");
        // -----------------------------------------------
        MultiIndexMoves::index<ByP>::type  &index = _store.get<ByP>();
        auto result = index.equal_range(boost::tuple<double, double>(aim));
        // -----------------------------------------------
        size_t count = 0U;
        RangeInserter rangeInserter;
        for (auto it = result.first; it != result.second(); ++it)
        {
            rangeInserter(range, *it);
            ++count;
        }
        // -----------------------------------------------
        return  count;
    }
    //------------------------------------------------------------------------------
    void draw(HDC hdc, double radius) const
    { draw(hdc, radius, [](size_t) { return (HPEN)GetStockObject(BLACK_PEN); }); }
    void draw(HDC hdc, double radius, HPEN hPen) const
    { draw(hdc, radius, [hPen](size_t) { return hPen; }); }
    void draw(HDC hdc, double radius, std::function<HPEN(size_t)> getPen) const;
    //------------------------------------------------------------------------------
    void dump_off(const tstring &filename, bool text_else_bin = true) const;
    void pick_up(const tstring &filename, bool text_else_bin = true);

    //------------------------------------------------------------------------------
    // using InverseIndexPassing = std::pair<InverseIndex::const_iterator, InverseIndex::const_iterator>;
    // 
    // /// for (auto it=ret.first; it!=ret.second; ++it) {}
    // InverseIndexPassing moves_passed_point(const Point &p, double radius) const
    
    //using InverseIndexPassing = std::pair<InverseIndex::const_iterator, InverseIndex::const_iterator>;
    template <class range_t>
    size_t near_passed_points(OUT range_t &range, IN const Point &p, IN double radius) const
    {
        boost::lock_guard<boost::mutex>  lock(_store_mutex);
        // -----------------------------------------------
        static_assert (boost::is_same<range_t, adjacency_t>::value
                    || boost::is_same<range_t, adjacency_ptrs_t>::value
                    || boost::is_same<range_t, adjacency_sh_ptrs_t>::value,
                       "Incorrect type to template function.");
        // -----------------------------------------------
        _inverse_kdtree.addPoints(_inverse_kdtree.getAllIndices().size(), _inverse.size());
        // -----------------------------------------------
        double query_point[] = { p.x, p.y };
        nanoflann::SearchParams searchParams{ 10 /* !IGNORED */, EPS, true /* sorted by distance to aim */ };
    
        std::vector<std::pair<size_t, double>> found_points_indices;
        size_t n_found = _inverse_kdtree.radiusSearch(query_point, radius, result, searchParams);
        // -----------------------------------------------
        RangeInserter rangeInserter;
        for (auto &p : found_points_indices)
            rangeInserter(range, _inverse[p.first].second);
        // -----------------------------------------------
        return n_found;
    }
    
    /// 
    bool near_passed_build_index()
    {
        try
        {
            boost::lock_guard<boost::mutex> lock(_store_mutex);
            _inverse_kdtree.addPoints(_inverse_kdtree.getAllIndices().size(), _inverse.size());
        }
        catch (const std::exception &e)
        {
            CERROR(e.what());
        }
        return true;
    }
    //------------------------------------------------------------------------------
    using MultiIndexMovesIxPcIter = MultiIndexMoves::index<ByP>::type::const_iterator;
    using MultiIndexMovesPassing = std::pair<MultiIndexMovesIxPcIter, MultiIndexMovesIxPcIter>;

    /// for (auto it=ret.first; it!=ret.second; ++it) {}
    MultiIndexMovesPassing aim_adjacency(IN const Point &aim, IN double radius) const
    {
        const auto &indexP = _store.get<ByP>();
        // -----------------------------------------------
        auto itFirstLower = indexP.lower_bound(boost::make_tuple(aim.x - radius, aim.y - radius));
        auto itFirstUpper = indexP.upper_bound(boost::make_tuple(aim.x + radius, aim.y + radius));
        // -----------------------------------------------
        return std::make_pair(itFirstLower, itFirstUpper);
    }
    //------------------------------------------------------------------------------
    void  insert(const Record &rec);
    //------------------------------------------------------------------------------
    auto  begin()       -> decltype(_store.begin()) { return _store.begin (); }
    auto  begin() const -> decltype(_store.begin()) { return _store.begin (); }

    auto  end()       -> decltype(_store.end()) { return _store.end (); }
    auto  end() const -> decltype(_store.end()) { return _store.end (); }
    //------------------------------------------------------------------------------
    void  clear()
    {
        boost::lock_guard<boost::mutex> lock(_store_mutex);
        _store.clear();
        _inverse.clear();
        for (size_t i = 0; i < _inverse_kdtree.getAllIndices().size(); ++i)
            _inverse_kdtree.removePoint(i);
    }
    bool  empty() const
    {
        // boost::lock_guard<boost::mutex>  lock(_store_mutex);
        return _store.empty();
    }
    size_t size() const
    {
        // boost::lock_guard<boost::mutex>  lock(_store_mutex);
        return _store.size();
    }
    //------------------------------------------------------------------------------


    //==============================================================================
    inline size_t kdtree_get_point_count() const { return _inverse.size(); }
    inline double kdtree_get_pt(const int idx, int dim) const
    { return (dim) ? _inverse[idx].first->y : _inverse[idx].first->x; }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
    //==============================================================================
  };
}
//------------------------------------------------------------------------------
BOOST_CLASS_VERSION(RoboMoves::Store, 2)
//------------------------------------------------------------------------------
#endif // _ROBO_MOVES_STORE_H_
