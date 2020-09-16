#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboPosApprox.h"
//#include "RoboPosTour.h"
//#include "RoboPosTourEvo.h"
//#include "RoboLearnMoves.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;

namespace RoboMoves {
//------------------------------------------------------------------------------
//class AFilter : public RoboMoves::ApproxFilter //!< pick only 3 endPoint in each 'side'-vicinity of target
//{
//    const std::shared_ptr<const RoboMoves::Store> store;
//    const std::shared_ptr<const TargetI> target;
//    TargetI::vec_t::const_iterator tg_it{};
//    const TargetI::vec_t::const_iterator tg_end;
//#ifndef NO_MTREE
//    mt::mtree::query query;
//    mt::mtree::query::iterator query_iterator;
//#else //NO_MTREE
//    const Robo::distance_t side{};
//    RoboMoves::adjacency_ptrs_t range{};
//    size_t i_pt{};
//#endif //NO_MTREE
//    const size_t n_pt_at_tg{};
//#ifdef VISITED
//    RoboMoves::Store::VisitedHashes visited{};
//    RoboMoves::ControlHasher ch{};
//#endif //VISITED
//    //Point aim{};
//public:
//    AFilter(const RoboMoves::Store *store, const TargetI *target, Robo::distance_t side = 0.11, size_t pick_points = 3) :
//        store(store, [](const Store*) {}),
//        target(target, [](const TargetI*) {}),
//        tg_it(target->coords().begin()),
//        tg_end(target->coords().end()),
//#ifndef NO_MTREE
//        query(),
//        query_iterator(query.begin()),
//#else //NO_MTREE
//        side(side),
//#endif //NO_MTREE
//        n_pt_at_tg(pick_points)
//    {}
//    AFilter(AFilter&&) = default;
//    AFilter(const AFilter&) = default;
//    const Record* operator()() override;
//    void reset() override
//    {
//        tg_it = target->coords().begin();
//#ifndef NO_MTREE
//        query = {};
//        query_iterator = query.begin();
//#else //NO_MTREE
//        range.clear();
//#endif //NO_MTREE
//    }
//    size_t expect_size() const override { return (n_pt_at_tg * target->n_coords()); }
//};
} //end RoboMoves

//------------------------------------------------------------------------------

//const Record* RoboMoves::AFilter::operator()()
//{
//start:;
//#ifndef NO_MTREE
//    while (query_iterator == query.end())
//    {
//        query = store->_mtree->get_nearest_by_limit(*tg_it, n_pt_at_tg);
//        query_iterator = query.begin();
//        if (++tg_it == tg_end)
//            return nullptr; //finish
//    }
//#ifdef VISITED
//    for (; query_iterator == query.end(); ++query_iterator)
//    {
//        const Record *pRec = query_iterator->data;
//        auto hash = ch(pRec->controls);
//        if (visited.count(hash) == 0)
//        {
//            visited.insert(hash);
//            return pRec;
//        }
//    }
//    goto start;
//#endif //VISITED
//    return (query_iterator++)->data;
//#else //NO_MTREE
//    if (range.empty() || (i_pt % n_pt_at_tg) == 0)
//    {
//        range.clear();
//        while (range.empty())
//        {
//            aim = *tg_it;
//            store->adjacencyPoints(range, aim, side);
//            if (++tg_it == tg_end)
//                return nullptr; //finish
//        }
//        range.sort(ClosestPredicate(aim));
//        i_pt = 0;
//    }
//#ifdef VISITED
//    for (const Record *pRec = range.front(); range.size() > 0; )
//    {
//        auto hash = ch(pRec->controls);
//        if (visited.count(hash) == 0)
//        {
//            visited.insert(hash);
//            range.pop_front();
//            ++i_pt;
//            tcout << "AFilter::req::aim=" << aim << i_pt << std::endl;
//            return pRec;
//        }
//        range.pop_front();
//    }
//    tcout << "AFilter::range.empty aim=" << aim << std::endl;
//    goto start;
//#endif //VISITED
//    visited.insert(hash);
//    range.pop_front();
//    ++i_pt;
//    return pRec;
//#endif //NO_MTREE
//}
