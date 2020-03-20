#include "StdAfx.h"

#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboPosTour.h"
#include "RoboPosTourEvo.h"
#include "RoboPosApprox.h"
#include "RoboLearnMoves.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;

//#define TOUR_OLD
#if defined(TOUR_OLD)
#include "RoboPosTourNoRec.h"
#endif

//------------------------------------------------------------------------------
bool RoboPos::LearnMoves::less(Robo::distance_t distance, Robo::distance_t new_distance)
{
    CDEBUG(" d=" << distance << " > nd=" << new_distance << std::endl);
    return distance > new_distance && (distance - new_distance) > 0.000000001;
}

//------------------------------------------------------------------------------
bool RoboPos::LearnMoves::check_precision(Robo::distance_t distance)
{
    return (_target.precision() > distance);
}

//------------------------------------------------------------------------------
bool RoboPos::LearnMoves::check_precision(Robo::distance_t distance, const Point &aim)
{
    bool res = check_precision(distance);
    if (res)
        CINFO(aim << _T(" reached"));
    return res;
}

//------------------------------------------------------------------------------
struct RoboPos::LearnMoves::MidHitStat
{
    Robo::distance_t min_ = DBL_MAX, max_ = 0., avg_ = 0.;
    int count_ = 0;
    void append(Robo::distance_t d)
    {
        avg_ += d;
        if (min_ > d) min_ = d;
        if (max_ < d) max_ = d;
        ++count_;
    }
    void print() const
    {
        std::cout << "mid_hit_stat: {";
        if (count_)
            std::cout << "c=" << count_ << " min=" << min_ << " max=" << max_ << " avg=" << avg_ / count_;
        std::cout << "}" << std::endl;
    }
};

//------------------------------------------------------------------------------
void RoboPos::LearnMoves::append(MidHitStat *mhs, Robo::distance_t d) { mhs->append(d); }


//------------------------------------------------------------------------------
RoboPos::LearnMoves::LearnMoves(IN RoboMoves::Store &store, IN Robo::RoboI &robo,
                                IN const TargetI &target, IN const tstring &fn_config) :
    _store(store), _robo(robo), _target(target), _fn_config(fn_config)
{
    _robo.reset();
    _base_pos = _robo.position();
    read_config();
    mid_hit_stat  = new LearnMoves::MidHitStat();
    mid_hit_stat1 = new LearnMoves::MidHitStat();
}

//------------------------------------------------------------------------------
RoboPos::LearnMoves::~LearnMoves()
{
    delete mid_hit_stat;
    delete mid_hit_stat1;
}

//------------------------------------------------------------------------------
void RoboPos::LearnMoves::read_config()
{
    tfstream fin = Utils::utf8_stream(_fn_config, std::ios::in);
    if (!fin.is_open())
        throw std::runtime_error("LM: config is not open");
    pt::read_ini(fin, _config);
    load(_config);
}

//------------------------------------------------------------------------------
void RoboPos::LearnMoves::save(tptree &node) const
{
    tptree child;
    node.add_child(_T("LearnMoves"), child);
    CONF_PUT(child, _tries);
    //CONF_PUT(child, tries_break);
    CONF_PUT(child, _random_try);
    CONF_PUT(child, side3);
    CONF_PUT(child, side_decrease_step);
    CONF_PUT(child, use_weighted_mean);
}

//------------------------------------------------------------------------------
void RoboPos::LearnMoves::load(tptree &node)
{
    //assert(node.size() == 6);
    CONF_GET_OPT_VAL(node, _tries, 4);
    //CONF_GET_OPT(node, tries_break);
    CONF_GET_OPT_VAL(node, _random_try, 3);
    CONF_GET_OPT_VAL(node, side3, 0.1);
    CONF_GET_OPT_VAL(node, side_decrease_step, 0.01);
    CONF_GET_OPT_VAL(node, use_weighted_mean, true);
}

//------------------------------------------------------------------------------
std::shared_ptr<TourI> RoboPos::LearnMoves::makeTour(int stage)
{
    read_config();
    std::shared_ptr<TourI> pTour;
    if (stage == 1)
    {
        tstring type = _config.get<tstring>(_T("Tour.1"));
        if (type == _T("evo"))
            pTour = std::make_shared<TourEvo>(_store, _robo, _config, _target);
        else if (type == _T("evostep"))
            pTour = std::make_shared<TourEvoSteps>(_store, _robo, _config, _target);
        else if (type == _T("workspace"))
            pTour = std::make_shared<TourWorkSpace>(_store, _robo, _config);
        else
            CERROR("Invalid type");
    }
    //else if (stage == 2)
    //{
    //    tstring type = _config.get<tstring>(_T("Tour.2"));
    //    if (type == _T("target"))
    //        pTour = std::make_shared<TourTarget>(_store, _robo);
    //    else
    //        CERROR("Invalid type");
    //}
    else CERROR("Invalid stage");
    return pTour;
}

//------------------------------------------------------------------------------
class AFilter : public RoboMoves::ApproxFilter
{
    RoboMoves::Store &store;
    RoboMoves::adjacency_ptrs_t range{};
    TargetI::vec_t::const_iterator tg_it{};
    const TargetI::vec_t::const_iterator tg_end;
    const Robo::distance_t side{};
    const size_t n_pt_at_tg{3};
    size_t i_pt{};
public:
    AFilter(RoboMoves::Store &store, const TargetI &target, Robo::distance_t side) :
        store(store), 
        tg_it(target.coords().begin()), 
        tg_end(target.coords().end()),
        side(side)
    {}
    //AFilter(AFilter&&) = delete;
    //AFilter(const AFilter&) = delete;
    const Record* operator()(size_t index)
    {
        if ((i_pt % n_pt_at_tg) == 0)
        {
            do
            {
                store.adjacencyPoints(range, *tg_it, side);
                ++tg_it;
                if (tg_it == tg_end)
                    return nullptr; //finish
            } while (range.empty());
            range.sort(ClosestPredicate(*tg_it));
        }
        const Record *pRec = range.front();
        range.pop_front();
        ++i_pt;
        return pRec;
    }
};

//------------------------------------------------------------------------------
/// грубое покрытие всего рабочего пространства
void  RoboPos::LearnMoves::STAGE_1()
{
    try
    {
    load(_config);
    /* mm :
    *    (target.max - target.min) = 300 mm
    *    1
    *    0.84 <--> 300 mm
    *    x    <-->   1 mm   ==> 0.0028
    */

#if defined TOUR_OLD
    borders_t borders;
    defineRobotBorders(_robo, 70U /*25U*/, borders);
    Approx approx(1,1);
    std::shared_ptr<Tour> pTour{ new TourNoRecursion(_store, _robo, borders, _target, approx) };
    pTour->run(/* _b_distance */  true, // Stage 1
               /* _b_target   */ false,
               /* _b_braking  */  true,
               /* _b_predict  */ false,
               /* _b_checking */  true,
               borders,
               0.07 /*0.1*/, 5 /*1*/);
               //0.1, 10);
               //0.03, 3); // non-recursive
#else
    std::shared_ptr<TourI> pTour = makeTour(1);
    if (!pTour) return;
    pTour->run();
#endif
    }
    catch (boost::thread_interrupted&)
    { CINFO("WorkingThread interrupted"); }
    catch (const std::exception &e)
    { SHOW_CERROR(e.what()); }
}
/// Покрытие всей мишени не слишком плотно
void  RoboPos::LearnMoves::STAGE_2()
{
    try
    {
    load(_config);
    /*         ~ - - - - *
     *       /
     *      /  +----------------+
     *    x \  |                |
     *   x   \ |                |
     * x       -                |
     *         | \              |
     *         |  *             |
     *         |                |
     *         +----------------+
     */
    TourTarget::TargetContain target_contain = [&target=_target](const Point &p) {
        //return target.contain(p);
        double corr = 0.01;
        return (p.x >= (target.min().x - corr) && p.x <= (target.max().x + corr) &&
                p.y >= (target.min().y - corr) && p.y <= (target.max().y + corr));
    };
    
#if defined TOUR_OLD
    borders_t borders;
    defineTargetBorders(_target, _store, /* side */ 0.05, borders);
    std::shared_ptr<Tour> pTour{ new TourNoRecursion(_store, _robo, borders, _target, approx) };
    pTour->run(/* _b_distance */ false,
               /* _b_target   */ true, // Stage 2
               /* _b_braking  */ true,
               /* _b_predict  */ true,
               /* _b_checking */ true,
               borders,
               0.015 /*0.02*/, 2 /*3*/);
               //0.015, 2); // non-recursive
#else
    // _T("target")
    std::shared_ptr<TourTarget> pTour = std::make_shared<TourTarget>(_store, _robo, _config, _target, target_contain);
    if (!pTour) return;
    pTour->run();
#endif
    }
    catch (boost::thread_interrupted&)
    { CINFO("WorkingThread interrupted"); }
    catch (const std::exception &e)
    { SHOW_CERROR(e.what()); }
}
/// Попадание в оставшиеся непокрытыми точки мишени
void  RoboPos::LearnMoves::STAGE_3(OUT Trajectory &uncovered)
{
    size_t count_regular = 0, count_random = 0;

    try
    {
    if (!_store.size())
        CERROR("Empty database");
    
    load(_config);
    uncovered.clear();

    {
        CINFO("Construct Approx...");
        AFilter next(_store, _target, side3);
        _store.constructApprox(32, next);
    }

    _complexity = 0;
    _gradient_points_complexity = 0;
    _gradient_wmeans_complexity = 0;
    _rundown_alldirs_complexity = 0;
    _rundown_maindir_complexity = 0;
    _wmean_complexity = 0;
    // -----------------------------------------------------
    size_t current = 0;
    auto itp = _target.it_coords();
    for (auto it = itp.first; it != itp.second; ++it, ++current)
    {
        distance_t distance = bg::distance(_base_pos, *it);
        // ---------------------------------------------------
        CINFO(_T("current: ") << current << _T(" / ") << _target.coords().size());
        tcerr << _T("current: ") << current << _T(" / ") << _target.coords().size();
        bool is_aim;
        // ---------------------------------------------------        
        for (size_t tries = 0; (tries <= _tries) && !check_precision(distance, *it); ++tries)
        {
            Point aim;
            // -------------------------------------------------
            if (!(tries % _random_try))
            {
                ++count_regular;
                aim = *it;
                is_aim = true;
            }
            else
            {
                ++count_random;
                const distance_t prec = _target.precision();
                const auto rx = Utils::random(-prec * 10, prec * 10); // const_3 from??
                const auto ry = Utils::random(-prec * 10, prec * 10);

                //const auto min = prec * prec;
                //const auto max = prec * 2.;
                //auto rx = Utils::random(min, max);
                //auto ry = Utils::random(min, max);
                //rx = Utils::random(2) ? -rx : rx;
                //ry = Utils::random(2) ? -ry : ry;

                aim = { it->x + rx, it->y + ry };
                is_aim = false;
            }
            // -------------------------------------------------
            distance_t d = testStage3(aim);
            if (is_aim)
                distance = d;
            // -------------------------------------------------
            boost::this_thread::interruption_point();
            // -------------------------------------------------
        }
        if (!check_precision(distance, *it))
        {
            tcerr << _T("  failed") << std::endl;
            uncovered.push_back(*it);
        }
        else tcerr << _T("  reached") << std::endl;
    } // for
    } // try
    catch (boost::thread_interrupted&)
    { CINFO("WorkingThread interrupted"); }
    catch (const std::exception &e)
    { SHOW_CERROR(e.what()); }
    // -----------------------------------------------------
    CINFO(_T("\n") <<
          _T("\ngradient_points_complexity: ") << _gradient_points_complexity <<
          _T("\ngradient_wmeans_complexity: ") << _gradient_wmeans_complexity <<
          _T("\nrundown_alldirs_complexity: ") << _rundown_alldirs_complexity <<
          _T("\nrundown_maindir_complexity: ") << _rundown_maindir_complexity <<
          _T("\n  weighted_mean_complexity: ") << _wmean_complexity << 
          _T("\n") <<
          _T("\n   TOTAL Complexity: ") << complexity() <<
          _T("\n AVERAGE Complexity: ") << (static_cast<double>(complexity()) / (count_regular + count_random)) <<
          _T("\n            minutes: ") << (static_cast<double>(complexity()) / TourI::divToMinutes) <<
          _T("\n   Uncovered points: ") << uncovered.size() << _T(" / ") << _target.coords().size() <<
          _T("\n      regular tries: ") << count_regular <<
          _T("\n       random tries: ") << count_random << 
          _T("\n"));
    // -----------------------------------------------------
    mid_hit_stat->print();
    mid_hit_stat1->print();
    //std::exit(1);
}

//------------------------------------------------------------------------------
void  RoboPos::LearnMoves::uncover(OUT Trajectory &uncovered)
{
    uncovered.clear();
    auto itp = _target.it_coords();
    for (auto it = itp.first; it != itp.second; ++it)
    {
        auto p = _store.getClosestPoint(*it, side3);
        if (p.first && boost_distance(p.second.hit, *it) > _target.precision())
            uncovered.push_back(*it);
    }
    CINFO("uncovered: " << uncovered.size() << '/' << _target.coords().size());
}

//------------------------------------------------------------------------------
Robo::distance_t RoboPos::LearnMoves::actionRobo(IN const Point &aim, IN const Control &controls)
{
    // -----------------------------------------------
    boost::this_thread::interruption_point();
    // -----------------------------------------------
    Point hit;
    const Record *pRec = _store.exactRecordByControl(controls);
    if (pRec) // visited.find (h) != visited.end () )
    {
        // if ( pRec ) { 
        hit = pRec->hit;
        // } else { throw exception ("handAct: Not in Store"); }
    }
    else
    {
        _robo.reset();
        _robo.move(controls);
        ++_complexity;
        hit = _robo.position();
        //if (pRec)
        //{
        //    tcout << " d=" << bg::distance(pRec->hit, hit) << ", hit" << hit << ", old" << pRec->hit;
        //    tcout << std::endl;
        //    tcout << std::endl;
        //}
        _store.insert(Record{ aim, _base_pos, hit, controls, _robo.trajectory() });
    }
    return bg::distance(aim, hit);
}

//------------------------------------------------------------------------------
Point RoboPos::LearnMoves::predict(const Robo::Control &controls)
{ return _store.getApprox()->predict(controls); }
