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
RoboPos::LearnMoves::LearnMoves(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN const TargetI &target,
                                IN double precision_mm, IN const tstring &fn_config) :
    _store(store), _robo(robo), _target(target)//, _precision(precision_mm * TourI::divToMiliMeters)
{
    _robo.reset();
    _base_pos = _robo.position();
    
    tfstream fin = Utils::utf8_stream(fn_config, std::ios::in);
    if (!fin.is_open())
        throw std::runtime_error("LM: config is not open");
    pt::read_ini(fin, _config);
    load(_config);
}

//------------------------------------------------------------------------------
void RoboPos::LearnMoves::save(tptree &node) const
{
    tptree lm;
    node.add_child(_T("LearnMoves"), lm);
    lm.put(_T("tries"), unsigned(_tries));
    //lm.put(_T("LearnMoves.tries_break"), tries_break);
    //lm.put(_T("LearnMoves.random_try"), random_try);
    lm.put(_T("side"), side3);
    lm.put(_T("side_decrease_step"), side_decrease_step);
    lm.put(_T("use_weighted_mean"), use_weighted_mean);
}

//------------------------------------------------------------------------------
void RoboPos::LearnMoves::load(tptree &node)
{
    //assert(node.size() == 6);
    _tries = node.get_optional<unsigned>(_T("tries")).get_value_or(4);
    //tries_break = node.get_optional<unsigned>(_T("tries_break"));
    //random_try = node.get_optional<unsigned>(_T("random_try"));
    side3 = node.get<double>(_T("side"), 0.1);
    side_decrease_step = node.get<double>(_T("side_decrease_step"), 0.01);
    use_weighted_mean = node.get<bool>(_T("use_weighted_mean"), true);
}

//------------------------------------------------------------------------------
std::shared_ptr<TourI> RoboPos::LearnMoves::makeTour(int stage)
{
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
    load(_config);
    size_t count = 0;
    size_t count_random = 0;
    try
    {
    uncovered.clear();
    // -----------------------------------------------------
    auto itp = _target.it_coords();
    for (auto it = itp.first; it != itp.second; ++it)
    {
        CINFO(_T("current: ") << count << _T(" / ") << _target.coords().size());
        // ---------------------------------------------------
        auto prec = _target.precision();
        auto p = _store.getClosestPoint(*it, side3);
        for (size_t tries = 0; (tries <= _tries) && (!p.first || bg::distance(p.second.hit, *it) > prec); ++tries)
        {
            ++count;
            Point pt;
            // -------------------------------------------------
            if (!(tries % 3)) pt = *it;
            else
            {
                ++count_random;
                double min = prec * prec;
                double max = prec * 2.;

                double rx = Utils::random(min, max);
                double ry = Utils::random(min, max);

                rx = Utils::random(2) ? -rx : rx;
                ry = Utils::random(2) ? -ry : ry;

                pt = { it->x + rx, it->y + ry };
            }
            // -------------------------------------------------
            _complexity += testStage3(pt);
            p = _store.getClosestPoint(*it, side3);
            // -------------------------------------------------
            boost::this_thread::interruption_point();
        }
        // ---------------------------------------------------
        if (!p.first || boost_distance(p.second.hit, *it) > prec)
            uncovered.push_back(*it);
    }
    }
    catch (boost::thread_interrupted&)
    { CINFO("WorkingThread interrupted"); }
    catch (const std::exception &e)
    { SHOW_CERROR(e.what()); }
    // -----------------------------------------------------
    CINFO(_T("TOTAL Complexity: ") << complexity() << 
          _T(" minutes:") << (static_cast<double>(complexity()) / TourI::divToMinutes) << std::endl <<
          _T("AVERAGE Complexity: ") << complexity() / count << std::endl << 
          _T("Uncovered points: ") << uncovered.size() << "/" << _target.coords().size() << std::endl <<
          _T("tries: ") << count << _T(" tries-of-randoms: ") << count_random << std::endl);
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
bool RoboPos::LearnMoves::actionRobo(IN const Point &aim, IN const Control &controls, OUT Point &hit)
{
    bool res = false;
    //ControlHasher ch; // ???
    //size_t h = ch(controls);
    // -----------------------------------------------
    boost::this_thread::interruption_point();
    // -----------------------------------------------
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
        hit = _robo.position();
        Record rec(aim, _base_pos, hit, controls, _robo.trajectory());
        _store.insert(rec);
        res = true;
    }
    return res;
}
