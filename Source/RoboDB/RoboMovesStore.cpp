#ifndef NO_INVERSE_INDEX
#include "nanoflann.hpp" // https://github.com/jlblancoc/nanoflann
#endif
#ifndef NO_MTREE
#include "mtree.h" // https://github.com/erdavila/M-Tree
#endif

#include "RoboMovesStore.h"
#include "RoboPosApprox.h"


//------------------------------------------------------------------------------
namespace RoboMoves
{
class ANoFilter : public RoboMoves::ApproxFilter
{
    Store::MultiIndexIterator it;
    const Store::MultiIndexIterator it_end;
public:
    ANoFilter(const Store &store) : it(store.begin()), it_end(store.end()) {}
    ANoFilter(ANoFilter&&) = default;
    ANoFilter(const ANoFilter&) = default;
    const Record* operator() () { return ((it != it_end) ? &(*(it++)) : nullptr); }
};
}




#ifndef NO_MTREE
namespace RoboMoves {
//------------------------------------------------------------------------------
struct MTreeDistance
{
    Robo::distance_t operator()(const std::shared_ptr<Record> &aim,
                                const std::shared_ptr<Record> &p)   const { return bg::distance(aim->hit, p->hit); }
    Robo::distance_t operator()(const Record *aim, const Record *p) const { return bg::distance(aim->hit, p->hit); }
    Robo::distance_t operator()(const Record *aim, const Point  *p) const { return bg::distance(aim->hit, *p); }
    Robo::distance_t operator()(const Point  *aim, const Record *p) const { return bg::distance(*aim, p->hit); }
    Robo::distance_t operator()(const Point  *aim, const Point  *p) const { return bg::distance(*aim, *p); }
    Robo::distance_t operator()(const Record &aim, const Record &p) const { return bg::distance(aim.hit, p.hit); }
    Robo::distance_t operator()(const Record &aim, const Point  &p) const { return bg::distance(aim.hit, p); }
    Robo::distance_t operator()(const Point  &aim, const Record &p) const { return bg::distance(aim, p.hit); }
    Robo::distance_t operator()(const Point  &aim, const Point  &p) const { return bg::distance(aim, p); }
};
//------------------------------------------------------------------------------
//using MTree = mt::mtree<Record, MTreeDistance>;
//typedef mt::mtree<Record, MTreeDistance> MTree;
}
#endif //NO_MTREE



#ifndef NO_INVERSE_INDEX
//------------------------------------------------------------------------------
class RoboMoves::Store::InverseIndex
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
class RoboMoves::Store::InverseIndex::ResultCallBack
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
        boost::lock_guard<boost::mutex> lock(_store_mutex);
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
    boost::lock_guard<boost::mutex> lock(_store_mutex);
    // -----------------------------------------------
    auto it = _store.get<ByC>().find(controls);
    _store.get<ByC>().modify(it, mod);
}

//------------------------------------------------------------------------------
void RoboMoves::Store::clear()
{
    boost::lock_guard<boost::mutex> lock(_store_mutex);
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
    boost::lock_guard<boost::mutex> lock(_store_mutex);
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
    boost::lock_guard<boost::mutex> lock(_store_mutex);
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
        CINFO(" aim=" << aim << " PU=" << itPU->hit << " PL=" << itPL->hit << " d=" << bg::distance(rec.hit, aim));
        return rec;
    }
    else if (itPU != indexP.end())
    {
        const Record &rec = *itPU;
        CINFO(" aim=" << aim << " PU=" << rec.hit << " d=" << bg::distance(rec.hit, aim));
        return rec;
    }
    //else if (itPL != indexP.end())
    {
        const Record &rec = *itPL;
        CINFO(" aim=" << aim << " PU=" << rec.hit << " d=" << bg::distance(rec.hit, aim));
        return rec;
    }
#else  //MTREE
    auto query = _mtree->get_nearest(aim);
    return query.begin()->data;
#endif //MTREE
}

//------------------------------------------------------------------------------
size_t RoboMoves::Store::equalEndPoint(const Point &aim, SearchCallBack callback) const
{
    boost::lock_guard<boost::mutex> lock(_store_mutex);
    // -----------------------------------------------
    const MultiIndexMoves::index<ByP>::type &indexP = _store.get<ByP>();
    const auto result = indexP.equal_range(boost::tuple<double, double>(aim));
    // -----------------------------------------------
    size_t count = 0;
    for (auto it = result.first; it != result.second; ++it, ++count)
        callback(&(*it));
    CINFO(" aim " << aim << " count " << count);
    // -----------------------------------------------
    return count;
}

//------------------------------------------------------------------------------
const RoboMoves::Record* RoboMoves::Store::exactRecordByControl(IN const Robo::Control &controls) const
{
    boost::lock_guard<boost::mutex> lock(_store_mutex);
    // -----------------------------------------------
    const auto &index = _store.get<ByC>();
    // -----------------------------------------------
    auto equal = index.find(controls);
    bool exist = (equal != index.end());
    // -----------------------------------------------
    CINFO(" exact c=" << controls << " p=" << tstring{ (exist) ? tstring(equal->hit) : tstring(_T("-")) });
    // -----------------------------------------------
    return (exist) ? (&(*equal)) : (nullptr);
}

//------------------------------------------------------------------------------
size_t RoboMoves::Store::adjacencyByXYBorders(IN  const Point &aim, IN double side,
                                              OUT std::pair<Record, Record> &x_pair,
                                              OUT std::pair<Record, Record> &y_pair) const
{
    boost::lock_guard<boost::mutex> lock(_store_mutex);
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
    return count;
}

// --------------------------------------------------------------
void RoboMoves::Store::draw(HDC hdc, double radius, const GetHPen &getPen) const
{
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
void RoboMoves::Store::insert(const Record &rec)
{
    //try {
        /* Массив траекторий для каждой точки.
        *  Несколько вариантов одного и того же движения.
        *  Вверх класть более удачные движения, отн. кол-ва действий и точности попадания.
        */
        boost::this_thread::disable_interruption no_interruption;
        boost::lock_guard<boost::mutex> lock(_store_mutex);
        // ==============================
        auto status = _store.insert(rec);
#ifndef NO_MTREE
        _mtree->add(rec);
#endif
        status.first->updateTimeTraj(_trajectories_enumerate);
        // ==============================
#ifndef NO_INVERSE_INDEX
        if (status.second)
            for (const auto &p : rec.trajectory)
                _inverse->append(p.spec(), *status.first);
#endif
    //} catch (const std::exception &e) { SHOW_CERROR(e.what()); }
}

//------------------------------------------------------------------------------
void RoboMoves::Store::dump_off(const tstring &filename, const Robo::RoboI &robo, Format format) const
{
    //try {
        boost::this_thread::disable_interruption no_interruption;
        boost::lock_guard<boost::mutex> lock(_store_mutex);

        // --- header ---
        tptree root;
        robo.save(root);
        // --------------
        if (format == Format::TXT)
        {
            std::ofstream ofs(filename, std::ios_base::out);
            boost::archive::text_oarchive toa(ofs);
            toa << root << *this;
        }
        else if (format == Format::BIN)
        {
            std::ofstream ofs(filename, std::ios_base::out | std::ios_base::binary);
            boost::archive::binary_oarchive boa(ofs);
            boa << root << *this;
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
void RoboMoves::Store::pick_up(const tstring &filename, Robo::pRoboI &pRobo, Format format, const ApproxFilter *filter)
{
    if (!bfs::exists(filename))
        CERROR(_T("File '") << filename << _T("' does not exists."));

    clear();

    //try
    {
        boost::this_thread::disable_interruption no_interruption;
        boost::lock_guard<boost::mutex> lock(_store_mutex);

        tptree root;
        if (format == Format::TXT)
        {
            std::ifstream ifs(filename, std::ios_base::in);
            boost::archive::text_iarchive tia(ifs);
            tia >> root >> *this;
            //printTree(root, tcerr);
        }
        else if (format == Format::BIN)
        {
            std::ifstream ifs(filename, std::ios_base::in | std::ios_base::binary);
            boost::archive::binary_iarchive bia(ifs);
            bia >> root >> *this;
        }
        else CERROR("Invalid store format");

        size_t inverse_sz = 0;
#if (!defined(NO_INVERSE_INDEX) || !defined(NO_MTREE))
        for (const auto &rec : _store)
        {
#ifndef NO_MTREE
            _mtree->add(rec);
#endif
#ifndef NO_INVERSE_INDEX
            for (const auto &p : rec.trajectory)
            {
                _inverse->append(p.spec(), rec);
                ++inverse_sz;
            }
#endif
        }
#endif //INVERSE_INDEX||MTREE
        CINFO("loaded '" << filename << "' store " << size() << " inverse " << inverse_sz);

        // --- header ---
        Factory<Robo::RoboI> frobo;
        auto pNewRobo = frobo.create(root);
        if (*pNewRobo != *pRobo)
        {
            CINFO("change robo from " << pRobo->getName() << " to " << pNewRobo->getName());
            pRobo = pNewRobo;
        }

        constructApprox(RoboPos::Approx::max_n_controls, filter);
    }
    //catch (const std::exception &e) { SHOW_CERROR(e.what()); }
}

//------------------------------------------------------------------------------
RoboPos::Approx* RoboMoves::Store::getApprox() { return _approx.get(); }

//------------------------------------------------------------------------------
RoboMoves::ApproxFilter RoboMoves::Store::getApproxNoFilterAllRecords() const { return RoboMoves::ANoFilter(*this); }

//------------------------------------------------------------------------------
void RoboMoves::Store::constructApprox(size_t max_n_controls, const RoboMoves::ApproxFilter *filter)
{
    if (!getApprox())
    {
        _approx = std::make_unique<RoboPos::Approx>(
            this->size(),
            max_n_controls,
            /*noize*/[](size_t) { return 0.00000000001; },
            /*sizing*/[]() { return 1.01; });
    }

    if (!getApprox()->constructed())
    {
        CINFO("Construct Approx...");
        getApprox()->constructXY((filter) ? *filter : getApproxNoFilterAllRecords());
    }
}
