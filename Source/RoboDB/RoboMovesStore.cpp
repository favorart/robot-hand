#ifndef NO_INVERSE_INDEX
#include "nanoflann.hpp" // https://github.com/jlblancoc/nanoflann
#endif
#ifndef NO_MTREE
#include "mtree.h" // https://github.com/erdavila/M-Tree
#endif

#include "RoboMovesStore.h"
#include "RoboPosApprox.h"


//------------------------------------------------------------------------------
namespace RoboMoves {
class ANoFilter : public RoboMoves::ApproxFilter
{
    const Store &store;
    Store::MultiIndexIterator it;
    const Store::MultiIndexIterator it_end;
public:
    ANoFilter(const Store &store) : store(store), it(store.begin()), it_end(store.end()) {}
    ANoFilter(ANoFilter&&) = default;
    ANoFilter(const ANoFilter&) = default;
    const Record* operator()() override { return ((it != it_end) ? &(*(it++)) : nullptr); }
    void reset() override { it = store.begin(); }
    size_t expect_size() const override { return store.size(); }
};
}




#ifndef NO_MTREE
namespace RoboMoves {
//------------------------------------------------------------------------------
struct MTreeDistance
{
    Robo::distance_t operator()(const Point &aim, const Record *p) const { return bg::distance(aim, p->hit); }
    Robo::distance_t operator()(const Record *aim, const Record *p) const { return bg::distance(aim->hit, p->hit); }
};
}
#endif //NO_MTREE



#ifndef NO_INVERSE_INDEX
//------------------------------------------------------------------------------
class RoboMoves::Store::InverseIndex final
{
public:
    using Index = size_t;
    using Distance = Robo::distance_t;
    using IndexPair = std::pair<const Point*, const Record*>;
    using IndexContainer = std::vector<IndexPair>;
    using KDTreeDataset = InverseIndex;
    using KDTMetric = nanoflann::L2_Simple_Adaptor<Robo::distance_t, KDTreeDataset>;
    using KDTree = nanoflann::KDTreeSingleIndexDynamicAdaptor<KDTMetric, KDTreeDataset, Point::ndimensions>;
    //using RSearchReply  = std::vector<std::pair<Index, Robo::distance_t>>;
    class ResultCallBack;

    //KDTreeDataset(const Store &store) : owner(store) {}
    inline Index kdtree_get_point_count() const { return inverse.size(); } //!< \returns number of points in kdtree
    inline Distance kdtree_get_pt(Index i, Index dim) const { return (*inverse[int(i)].first)[int(dim)]; } //!< get the dim-th component of the i-th point
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&/*bb*/) const { return false; } //!< optional: false - default, true - if bounding-box was already computed (avoid redoing) and return in bb

    inline IndexPair operator[](Index i) const { return inverse[int(i)]; }
    inline Index size() const { return inverse.size(); }
    inline void append(const IndexPair &v) { inverse.push_back(v); }
    inline void append(const Point &p, const Record &rec) { inverse.emplace_back(&p, &rec); }
    void clear();

    size_t radius_search(const Point &aim, Distance radius, SearchCallBack callback) const;
    size_t knn_search(const Point &aim, Index k, SearchCallBack callback) const;
    void build_index();  //!< build kd-tree index of all passed points
    bool operator==(const InverseIndex &index) const { return (inverse == index.inverse); }
private:
    //const Store &owner;                //!< owner base class 
    IndexContainer inverse{};            //!< container, keeps data for kd-tree
    Index last_pass_point_indexed{ 0 };  //!< is need to recalc kd-tree?
    KDTree kdtree{ *this };              //!< kd-tree index
};

//------------------------------------------------------------------------------
void RoboMoves::Store::InverseIndex::clear()
{
    inverse.clear();
    last_pass_point_indexed = 0;
    //for (size_t i = 0; i < kdtree.getAllIndices().size(); ++i)
    //    kdtree.removePoint(i);
    kdtree.clear();
}

//------------------------------------------------------------------------------
void RoboMoves::Store::InverseIndex::build_index()
{
    if (last_pass_point_indexed != inverse.size())
        return;
    //boost::lock_guard<boost::mutex> lock(_store_mutex);
    kdtree.addPoints(last_pass_point_indexed, inverse.size() - 1);
    CINFO("built " << inverse.size() - last_pass_point_indexed);
    last_pass_point_indexed = inverse.size();
}

//------------------------------------------------------------------------------
class RoboMoves::Store::InverseIndex::ResultCallBack final
{
    const InverseIndex::IndexContainer &indices;
    const InverseIndex::Distance radius{};
    const InverseIndex::Index num_points = 0;
    InverseIndex::Index count = 0;
    Store::SearchCallBack callback{};
    std::vector<std::pair<InverseIndex::Distance, InverseIndex::Index>> space{};
public:
    ResultCallBack(const IndexContainer &indices, InverseIndex::Distance radius, SearchCallBack callback)
        : indices(indices), radius(radius), callback(callback)
    {}
    ResultCallBack(const IndexContainer &indices, InverseIndex::Index num_points, SearchCallBack callback)
        : indices(indices), num_points(num_points), callback(callback), space(num_points)
    {}
    size_t size() const { return count; }
    bool full() const { return (num_points == 0 || count < num_points); }
    InverseIndex::Distance worstDist() const { return (num_points > 0) ? space[num_points - 1].first : radius; }
    //! return true if the search should be continued, false if the results are sufficient
    bool addPoint(InverseIndex::Distance dist, InverseIndex::Index index);
    void calling()
    {
        for (const auto &p : space)
            callback(indices[p.second].second);
    }
};

//------------------------------------------------------------------------------
bool RoboMoves::Store::InverseIndex::ResultCallBack::addPoint(InverseIndex::Distance dist, InverseIndex::Index index)
{
    if (num_points > 0) // knn
    {
        InverseIndex::Index i;
        for (i = count; i > 0; --i) // sorted input
            if (space[i - 1].first > dist)
            {
                if (i < num_points)
                    space[i] = space[i - 1];
            }
            else break;
        if (i < num_points)
            space[i] = { dist, index };
        if (count < num_points)
            count++;
        else
        {
            //end??
            //callback(indices[space[i].second/*index*/].second);
        }
    }
    else if (dist < radius) // radius search
        callback(indices[index].second);
    return true; // tell caller that the search shall continue
}

//------------------------------------------------------------------------------
size_t RoboMoves::Store::InverseIndex::radius_search(const Point &aim, Robo::distance_t radius, SearchCallBack callback) const
{
    const nanoflann::SearchParams search_params{ 0 /* ignored */, Utils::EPSILONT, true /* sorted by distance to aim */ };
    ResultCallBack res(inverse, radius, callback);
    const double query[] = { aim.x, aim.y };
    kdtree.findNeighbors(res, query, search_params);
    return res.size();
}

//------------------------------------------------------------------------------
size_t RoboMoves::Store::InverseIndex::knn_search(const Point &aim, size_t k, SearchCallBack callback) const
{
    const nanoflann::SearchParams search_params{ 0 /* ignored */, Utils::EPSILONT, true /* sorted by distance to aim */ };
    ResultCallBack res(inverse, k, callback);
    const double query[] = { aim.x, aim.y };
    kdtree.findNeighbors(res, query, search_params);
    res.calling(); // execute all callbacks
    return res.size();
}
#endif //NO_INVERSE_INDEX

//------------------------------------------------------------------------------
size_t RoboMoves::Store::nearPassPoints(const Point &aim, Robo::distance_t radius, SearchCallBack callback) const
{
    size_t n_found = 0;

#ifndef NO_INVERSE_INDEX
    {
        boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);
        _inverse->build_index();
        n_found = _inverse->radius_search(aim, radius, callback);
    }
#endif //!NO_INVERSE_INDEX

    CDEBUG(" near to " << aim << " passed " << n_found << " moves");
    return n_found;
}



//------------------------------------------------------------------------------
RoboMoves::Store::Store() :
#ifndef NO_MTREE
    _mtree(std::make_shared<RoboMoves::MTree>()),
#endif
#ifndef NO_INVERSE_INDEX
    _inverse(std::make_shared<RoboMoves::Store::InverseIndex>()),
#endif
    _trajectories_enumerate(0)
{}

//------------------------------------------------------------------------------
void RoboMoves::Store::replace(IN const Robo::Control &controls, IN const RoboMoves::Store::Mod &mod)
{
    boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);
    // -----------------------------------------------
    auto it = _store.get<ByC>().find(controls);
    _store.get<ByC>().modify(it, mod);
    // -----------------------------------------------
#ifndef NO_MTREE
    // ???
#endif
    // -----------------------------------------------
#ifndef INVERSE_INDEX
    // ???
#endif
}

//------------------------------------------------------------------------------
void RoboMoves::Store::clear()
{
    boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);
    _store.clear();
    if (_approx)
        _approx->clear();
    _trajectories_enumerate = 0;

#ifndef NO_MTREE
    if (_mtree) _mtree->clear();
#endif
#ifndef NO_INVERSE_INDEX
    if (_inverse) _inverse->clear();
#endif
    CINFO(" store clear");
}

//------------------------------------------------------------------------------
std::pair<bool, RoboMoves::Record> RoboMoves::Store::getClosestPoint(IN const Point &aim, IN double side) const
{
    boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);
    // -----------------------------------------------
    const auto &indexP = _store.get<ByP>();
    // -----------------------------------------------
    auto itPL = indexP.lower_bound(boost::tuple<double, double>(aim.x - side, aim.y - side));
    auto itPU = indexP.upper_bound(boost::tuple<double, double>(aim.x + side, aim.y + side));
    // -----------------------------------------------
    const bool exist = (itPU != indexP.end() || itPL != indexP.end());
    const Record &rec = (exist) ? *std::min_element(itPL, itPU, ClosestPredicate(aim)) : Record{};
    if (exist)
        CINFO(" aim=" << aim << " hit=" << rec.hit << " d=" << bg::distance(rec.hit, aim));
    return std::make_pair(exist, rec);
}

//------------------------------------------------------------------------------
RoboMoves::Record RoboMoves::Store::closestEndPoint(const Point &aim) const
{
    boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);
    // -----------------------------------------------
#if defined(NO_MTREE)
    const auto &indexP = _store.get<ByP>();
    auto itPL = indexP.lower_bound(boost::tuple<double, double>(aim));
    auto itPU = indexP.upper_bound(boost::tuple<double, double>(aim));
    // -----------------------------------------------
    if (itPU == indexP.end() && itPL == indexP.end())
    {
        CERROR(" No closest point exists");
    }
    else if (itPU != indexP.end() && itPL != indexP.end())
    {
        const Record &rec = (ClosestPredicate(aim)(*itPU, *itPL)) ? *itPU : *itPL;
        CDEBUG(" aim=" << aim << " PU=" << itPU->hit << " PL=" << itPL->hit << " d=" << bg::distance(rec.hit, aim));
        return rec;
    }
    else if (itPU != indexP.end())
    {
        const Record &rec = *itPU;
        CDEBUG(" aim=" << aim << " PU=" << rec.hit << " d=" << bg::distance(rec.hit, aim));
        return rec;
    }
    //else if (itPL != indexP.end())
    {
        const Record &rec = *itPL;
        CDEBUG(" aim=" << aim << " PU=" << rec.hit << " d=" << bg::distance(rec.hit, aim));
        return rec;
    }
#else  //MTREE
    auto query = _mtree->get_nearest(aim);
    if (query.begin() != query.end())
    {
        const Record *rec = query.begin()->data;
        CDEBUG(" aim=" << aim << " PU=" << rec->hit << " d=" << bg::distance(rec->hit, aim));
        return *rec;
    }
    return Record{};
#endif //MTREE
}

//------------------------------------------------------------------------------
size_t RoboMoves::Store::knn_search(const Point &aim, size_t k, Store::SearchCallBack callback, Store::VisitedHashes *visited) const
{
    boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);
    // -----------------------------------------------
    size_t matched = 0;
    RoboMoves::ControlHasher ch{};

#if defined(NO_MTREE)
    const Robo::distance_t side = 0.11;
    RoboMoves::adjacency_ptrs_t range{};
    adjacencyPoints(range, aim, side);
    range.sort(ClosestPredicate(aim));
    for (const Record *rec : range)
    {
        if (matched == k)
            break;
#else  //MTREE
    auto query = _mtree->get_nearest_by_limit(aim, k);
    for (const auto &match : query)
    {
        const Record *rec = match->data;
#endif //MTREE
        if (rec)
        {
            auto hash = ch(rec->controls);
            if (visited && visited->count(hash) != 0)
                continue;
            visited->insert(hash);
            callback(rec);
            ++matched;
        }
    }
    return matched;
}

//------------------------------------------------------------------------------
size_t RoboMoves::Store::equalEndPoint(const Point &aim, SearchCallBack callback) const
{
    boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);
    // -----------------------------------------------
    const MultiIndexMoves::index<ByP>::type &indexP = _store.get<ByP>();
    const auto result = indexP.equal_range(boost::tuple<double, double>(aim));
    // -----------------------------------------------
    size_t count = 0;
    for (auto it = result.first; it != result.second; ++it, ++count)
        callback(&(*it));
    // -----------------------------------------------
    CDEBUG(" aim " << aim << " count " << count);
    // -----------------------------------------------
    return count;
}

//------------------------------------------------------------------------------
const RoboMoves::Record* RoboMoves::Store::exactRecordByControl(IN const Robo::Control &controls) const
{
    boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);
    // -----------------------------------------------
    const auto &index = _store.get<ByC>();
    // -----------------------------------------------
    auto equal = index.find(controls);
    bool exist = (equal != index.end());
    // -----------------------------------------------
    CDEBUG(" exact c=" << controls << " p=" << tstring{ (exist) ? tstring(equal->hit) : tstring(_T("-")) });
    // -----------------------------------------------
    return (exist) ? (&(*equal)) : (nullptr);
}

//------------------------------------------------------------------------------
size_t RoboMoves::Store::adjacencyByXYBorders(IN  const Point &aim, IN double side,
                                              OUT std::pair<Record, Record> &x_pair,
                                              OUT std::pair<Record, Record> &y_pair) const
{
    boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);
    //-------------------------------------------------------
    size_t count = 0U;
    //-------------------------------------------------------
    {
        const MultiIndexMoves::index<ByX>::type &X_index = _store.get<ByX>();
        auto pair_x = X_index.range((aim.x - side) <= boost::lambda::_1, boost::lambda::_1 <= (aim.x + side));

        decltype(pair_x.first)  it_max_x = X_index.end();
        decltype(pair_x.first)  it_min_x = X_index.end();
        for (auto it = pair_x.first; it != pair_x.second; ++it)
        {
            if ((it->hit.x < aim.x) && (it_max_x == X_index.end() || it_max_x->hit.x < it->hit.x))
            { it_max_x = it; }

            if ((it->hit.x > aim.x) && (it_min_x == X_index.end() || it_min_x->hit.x > it->hit.x))
            { it_min_x = it; }
        }

        if (it_max_x != X_index.end() && it_min_x != X_index.end())
        { x_pair = std::make_pair(*it_max_x, *it_min_x); count += 2U; }
    }
    //-------------------------------------------------------
    {
        const MultiIndexMoves::index<ByY>::type  &Y_index = _store.get<ByY>();
        auto pair_y = Y_index.range((aim.y - side) <= boost::lambda::_1, boost::lambda::_1 <= (aim.y + side));

        decltype(pair_y.first)  it_max_y = Y_index.end();
        decltype(pair_y.first)  it_min_y = Y_index.end();
        for (auto it = pair_y.first; it != pair_y.second; ++it)
        {
            if ((it->hit.y < aim.y) && (it_max_y == Y_index.end() || it_max_y->hit.y < it->hit.y))
            { it_max_y = it; }

            if ((it->hit.y > aim.y) && (it_min_y == Y_index.end() || it_min_y->hit.y > it->hit.y))
            { it_min_y = it; }
        }

        if (it_max_y != Y_index.end() && it_min_y != Y_index.end())
        { y_pair = std::make_pair(*it_max_y, *it_min_y); count += 2U; }
    }
    //-------------------------------------------------------
    CDEBUG(" aim=" << aim << " r=" << side << " count=" << count);
    //-------------------------------------------------------
    return count;
}

// --------------------------------------------------------------
void RoboMoves::Store::draw(HDC hdc, double radius, const GetHPen &getPen) const
{
    //boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);
#ifdef MY_WINDOW
    try
    {
        unsigned i = 0;
        for (const auto &rec : _store)
        {
            drawCircle(hdc, rec.hit, radius, getPen(rec));
            if (++i == 500)
            {
                boost::this_thread::interruption_point();
                i = 1;
            }
        }
    }
    catch (boost::thread_interrupted&)
    { CINFO("WorkingThread interrupted"); }
#endif // MY_WINDOW
}

//------------------------------------------------------------------------------
void RoboMoves::Store::mtree_insert(const Record &rec)
{
#ifndef NO_MTREE
    // mtree: no exact_find and no replace !!!
    auto pRecNearest = _mtree->get_nearest(rec.hit).begin()->data;
    if (!pRecNearest || pRecNearest->hit != rec.hit)
    {
        _mtree->add(&rec);
    }
    else if (pRecNearest->eleganceMove() > rec.eleganceMove())
    {
        _mtree->remove(pRecNearest);
        _mtree->add(&rec);
    }
#endif
}

//------------------------------------------------------------------------------
void RoboMoves::Store::inverse_insert(const Record &rec)
{
#ifndef NO_INVERSE_INDEX
    for (const auto &p : rec.trajectory)
        _inverse->append(p.spec(), rec);
#endif
}

//------------------------------------------------------------------------------
void RoboMoves::Store::insert(const Record &rec)
{
    //try {
        /* Массив траекторий для каждой точки.
        *  Несколько вариантов одного и того же движения.
        *  Вверх класть более удачные движения, отн. кол-ва действий и точности попадания.
        */
        boost::this_thread::disable_interruption no_interruption;
        boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);

        std::pair<MultiIndexIterator, bool> status = _store.insert(rec);
        if (status.second)
        {
            status.first->updateTimeTraj(_trajectories_enumerate);
            // ==============================
            mtree_insert(*status.first);
            inverse_insert(*status.first);
        }
        //CDEBUG(" aim=" << rec->aim);
    //} catch (const std::exception &e) { SHOW_CERROR(e.what()); }
}

//------------------------------------------------------------------------------
void RoboMoves::Store::dump_off(const tstring &filename, const Robo::RoboI &robo, Format format) const
{
    //try {
        boost::this_thread::disable_interruption no_interruption;
        boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);

        tptree db_header;
        robo.save(db_header);
        // --------------
        if (format == Format::TXT)
        {
            std::ofstream ofs(filename, std::ios_base::out);
            boost::archive::text_oarchive toa(ofs);
            toa << db_header;
            toa << *this;
        }
        else if (format == Format::BIN)
        {
            std::ofstream ofs(filename, std::ios_base::out | std::ios_base::binary);
            boost::archive::binary_oarchive boa(ofs);
            boa << db_header;
            boa << *this;
        }
        else CERROR("Invalid store format");

        size_t inverse_sz = 0;
#ifndef NO_INVERSE_INDEX
        inverse_sz = _inverse->size();
#endif
        CINFO("saved '" << filename << "' store " << size() << " inverse " << inverse_sz);
    //} catch (const std::exception &e) { SHOW_CERROR(e.what()); }
}

//------------------------------------------------------------------------------
void RoboMoves::Store::pick_up(const tstring &filename, Robo::pRoboI &pRobo, Format format, pApproxFilter filter)
{
    if (!bfs::exists(filename))
        CERROR(_T("File '") << filename << _T("' does not exists."));

    clear();

    //try
    {
        boost::this_thread::disable_interruption no_interruption;
        boost::lock_guard<boost::recursive_mutex> lock(_store_mutex);

        tptree db_header;
        if (format == Format::TXT)
        {
            std::ifstream ifs(filename, std::ios_base::in);
            boost::archive::text_iarchive tia(ifs);
            tia >> db_header;
            tia >> *this;
            //printTree(db_header, tcerr);
        }
        else if (format == Format::BIN)
        {
            std::ifstream ifs(filename, std::ios_base::in | std::ios_base::binary);
            boost::archive::binary_iarchive bia(ifs);
            bia >> db_header;
            bia >> *this;
        }
        else CERROR("Invalid store format");

#if (!defined(NO_INVERSE_INDEX) || !defined(NO_MTREE))
        for (const Record &rec : _store)
        {
            mtree_insert(rec);
            inverse_insert(rec);
        }
#endif //INVERSE_INDEX||MTREE

        size_t inverse_sz = 0;
#ifndef NO_INVERSE_INDEX
        inverse_sz = _inverse->size();
#endif
        CINFO("loaded '" << filename << "' store " << size() << " inverse " << inverse_sz);

        // --- header ---
        Factory<Robo::RoboI> frobo;
        auto pNewRobo = frobo.create(db_header);
        if (*pNewRobo != *pRobo)
        {
            CINFO("change robo from " << pRobo->getName() << " to " << pNewRobo->getName());
            pRobo = pNewRobo;
        }

        constructApprox(RoboPos::Approx::max_n_controls, std::move(filter));
    }
    //catch (const std::exception &e) { SHOW_CERROR(e.what()); }
}

//------------------------------------------------------------------------------
RoboPos::Approx* RoboMoves::Store::getApprox() { return _approx.get(); }

//------------------------------------------------------------------------------
RoboMoves::pApproxFilter RoboMoves::Store::getApproxNoFilterAllRecords() const { return std::make_unique<ANoFilter>(*this); }

//------------------------------------------------------------------------------
void RoboMoves::Store::constructApprox(size_t max_n_controls, RoboMoves::pApproxFilter filter)
{
    boost::lock_guard<boost::recursive_mutex> lock(_store_mutex); // !!! double lock
    //boost::this_thread::disable_interruption no_interruption;
    if (!_approx)
    {
        _approx = std::make_unique<RoboPos::Approx>(
            (filter) ? filter->expect_size() : this->size(), // !!!
            max_n_controls,
            /*noize*/[](size_t) { return 0.00000000001; },
            /*sizing*/[]() { return 1.01; });
    }

    if (!_approx->constructed())
    {
        CINFO("Construct Approx...");
        getApprox()->constructXY((filter) ? *filter : *getApproxNoFilterAllRecords());
    }
}
