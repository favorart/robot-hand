#include "StdAfx.h"

#include "Robo.h"
#include "RoboPos.h"
#include "RoboPosTour.h"
#include "RoboPosApprox.h"
#include "RoboLearnMoves.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
using namespace Robo::Mobile;
using namespace Robo::NewHand;

//#define TOUR_OLD
//#define TOUR_EVO
#if defined TOUR_OLD
#include "RoboPosTourNoRec.h"
#endif
//#elif defined TOUR_EVO
#include "RoboPosTourEvo.h"
//#endif

//------------------------------------------------------------------------------
RoboPos::LearnMoves::LearnMoves(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target) :
    _store(store), _robo(robo), _target(target), _precision(target.precision())
{
    _robo.reset();
    _base_pos = _robo.position();

    tptree root;
    tfstream fin("LearnMoves.txt", std::ios::in);
    if (!fin.is_open())
        return;
    pt::read_ini(fin, root);

    auto s = root.get<tstring>(_T("Tour1"));
    tcout << s << std::endl;
    Tour1_type = (s == _T("evo")) ? 1 : 2;
    Tour2_type = (root.get<tstring>(_T("Tour2")) == _T("target")) ? 1 : 0;
}
//------------------------------------------------------------------------------
RoboPos::LearnMoves::LearnMoves(IN RoboMoves::Store &store, IN Robo::RoboI &robo, IN RecTarget &target,
           IN double precision,
           IN const ConfigJSON::StageInput1 &stage1,
           IN const ConfigJSON::StageInput2 &stage2,
           IN const ConfigJSON::StageInput3 &stage3) :
    _store(store), _robo(robo), _target(target), _precision(precision),
    _stage1_params(stage1), _stage2_params(stage2), _stage3_params(stage3)
{
    _robo.reset();
    _base_pos = _robo.position();

    tptree root;
    tfstream fin("LearnMoves.txt", std::ios::in);
    if (!fin.is_open())
        return;
    pt::read_ini(fin, root);

    Tour1_type = root.get<tstring>(_T("Tour1"));
    Tour2_type = root.get<tstring>(_T("Tour2"));
}
//------------------------------------------------------------------------------
/// грубое покрытие всего рабочего пространства
void  RoboPos::LearnMoves::STAGE_1()
{
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
//#elif defined TOUR_EVO
    std::shared_ptr<TourI> pTour;
    if (Tour1_type == _T("evo"))
        pTour = std::make_shared<TourEvo>(_store, _robo, _target);
    else if (Tour1_type == _T("evoS"))
        pTour = std::make_shared<TourEvoSteps>(_store, _robo, _target);
    else
        pTour = std::make_shared<TourWorkSpace>(_store, _robo);
    //pTour->setPrecision(0.1, 7);
    //pTour->setBrakings(false);
    pTour->run();
#endif
}
/// Покрытие всей мишени не слишком плотно
void  RoboPos::LearnMoves::STAGE_2()
{
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

    auto noize = [](size_t) { return 0.00000000001; };
    auto sizing = []() { return 1.01; };

    Approx approx(_store.size(), _robo.musclesCount(), noize, sizing);
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
    std::shared_ptr<TourTarget> pTour = std::make_shared<TourTarget>(_store, _robo, approx, _target, target_contain);
    //pTour->setPrecision(0.011, 3);
    //pTour->setChecking(predict);
    //pTour->setPredict(predict);
    pTour->run();
#endif
}

int RoboPos::LearnMoves::tries = 33;

/// Попадание в оставшиеся непокрытыми точки мишени
void  RoboPos::LearnMoves::STAGE_3(OUT Trajectory &uncovered)
{
    size_t count = 0;
    uncovered.clear();
    // -----------------------------------------------------
    auto itp = _target.it_coords();
    for (auto it = itp.first; it != itp.second; ++it)
    {
        ++count;
        // ---------------------------------------------------
        CINFO(_T("current: ") << count << _T(" / ") << _target.coords().size());
        // ---------------------------------------------------
        auto prec = _target.precision();
        auto p = _store.getClosestPoint(*it, _stage3_params.side);
        for (auto tries = 0; (tries <= LearnMoves::tries /*33%3*/) && (!p.first || boost_distance(p.second.hit, *it) > prec); ++tries)
        {
            Point pt;
            // -------------------------------------------------
            if (!(tries % 3)) pt = *it;
            else
            {
                double min = prec * prec;
                double max = prec * 2.;

                double rx = Utils::random(min, max);
                double ry = Utils::random(min, max);

                rx = Utils::random(2) ? -rx : rx;
                ry = Utils::random(2) ? -ry : ry;
                // -------------------------------------------------
                pt = { it->x + rx, it->y + ry };
            }
            // -------------------------------------------------
            _complexity += gradientMethod_admixture(pt); /* !!! СЛОМАНО !!! */
            // -------------------------------------------------
            p = _store.getClosestPoint(*it, _stage3_params.side);
            // -------------------------------------------------
            boost::this_thread::interruption_point();
        }
        // ---------------------------------------------------
        if (!p.first || boost_distance(p.second.hit, *it) > prec)
            uncovered.push_back(*it);
    }
    // -----------------------------------------------------
    tcout << _T("TOTAL Complexity: ") << complexity()
          << _T(" ") << double(complexity()) / 60. << _T(" minutes.") << std::endl;
    tcout << _T("AVERAGE Complexity: ") << complexity() / count << std::endl;
    tcout << _T("Uncovered: ") << uncovered.size() << std::endl << std::endl;
}

//------------------------------------------------------------------------------
void  RoboPos::LearnMoves::uncover(OUT Trajectory &uncovered)
{
    uncovered.clear();
    //auto vec = _target.coords(); int i = 0;
    auto itp = _target.it_coords();
    //Point c = {}; int n = 0;
    for (auto it = itp.first; it != itp.second; ++it)
    {
        //tcout << n << ' ' << *it << ' ' << vec[n] << std::endl;
        //c += *it;
        //n++;
        auto p = _store.getClosestPoint(*it, _stage3_params.side);
        if (p.first && boost_distance(p.second.hit, *it) > _target.precision())
            uncovered.push_back(*it);
    }
    //tcout << "sum =" << c / n << std::endl;
}

//------------------------------------------------------------------------------
bool RoboPos::LearnMoves::actionRobo(IN const Point &aim, IN const Control &controls, OUT Point &hit)
{
    bool res = false;
    ControlHasher ch;
    size_t h = ch(controls);
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


//------------------------------------------------------------------------------
void  draftDistance(IN  Point  &robo_pos_prev,
                    IN  Point  &robo_pos,
                    OUT frames_t  &lasts_step,
                    IN  const frames_t  &last,
                    IN  const frames_t   lasts_max,
                    IN  const frames_t   lasts_incr_value,
                    IN double distance)
{
    if (boost_distance(robo_pos_prev, robo_pos) > distance)
    {
        if (lasts_step > 2 * lasts_incr_value)
            lasts_step -= lasts_incr_value;
        // else if ( lasts_step > lasts_incr_value )
        //   lasts_step -= lasts_incr_value * 0.5;
        // else if ( lasts_step > 2 )
        //   --lasts_step;

        // else if ( lasts_step == 1 && lasts_incr_value == 1U )
        // { }
    }
    else if (boost_distance(robo_pos_prev, robo_pos) < distance)
    {
        if ((last + lasts_incr_value) < lasts_max)
            lasts_step += lasts_incr_value;
        // else if ( (last + 0.5 * lasts_incr_value) < last_max )
        //   lasts_step += 0.5 * lasts_incr_value;
        // else
        //   lasts_step = lasts_max - last;
    }

}

//------------------------------------------------------------------------------
void  RoboPos::LearnMoves::testStage1()
{
    _complexity = 0;
    _robo.reset();
    Point  robo_pos_base = _robo.position();
    Point  robo_pos_prev_i = robo_pos_base;
    Point  robo_pos_prev_j = robo_pos_base;

    /* Возьмём первый мускул наугад */
    for (muscle_t muscle_i : { RoboI::muscleByJoint(0, false),
                               RoboI::muscleByJoint(0, true) })
        for (muscle_t muscle_j : { RoboI::muscleByJoint(1, false),
                                   RoboI::muscleByJoint(1, true) })
        // if ( (muscle_i != muscle_j) && musclesValidUnion (muscle_i | muscle_j) )
        {
            Point robo_pos_i;
            /* Нужен уменьшающийся шаг в зависимости от пройдённой дистанции */
            frames_t  lasts_step = _stage1_params.lasts_incr_value;

            auto last_i_max = _robo.muscleMaxLasts(muscle_i),
                 last_j_max = _robo.muscleMaxLasts(muscle_j);
            /* Попробуем его варьировать по длительности */
            for (frames_t last_i = _stage1_params.lasts_init_value; last_i < last_i_max; last_i += lasts_step)
            {
                for (frames_t last_j = _stage1_params.lasts_init_value; last_j < last_j_max; last_j += lasts_step)
                {
                    Control control;
                    control.append({ muscle_i, 0U, last_i });
                    control.append({ muscle_j, 0U /* start_j */, last_j });

                    _robo.reset();
                    _robo.move(control);

                    Point robo_pos_j = _robo.position();
                    if (last_j == _stage1_params.lasts_init_value)
                        robo_pos_i = robo_pos_j;

                    Record rec{ robo_pos_j, robo_pos_base, robo_pos_j, control, _robo.trajectory() };
                    _store.insert(rec);

                    draftDistance(robo_pos_prev_j, robo_pos_j, lasts_step, last_j, last_j_max,
                                  _stage1_params.lasts_incr_value, _stage1_params.draft_distance);
                    robo_pos_prev_j = robo_pos_j;

                    ++_complexity;

                    try
                    { boost::this_thread::interruption_point(); }
                    catch (boost::thread_interrupted&)
                    { 
                        CINFO("thread interrupted");
                        return;
                    }
                } // end for

                draftDistance(robo_pos_prev_i, robo_pos_i, lasts_step, last_i, last_i_max,
                              _stage1_params.lasts_incr_value, _stage1_params.draft_distance);
                robo_pos_prev_i = robo_pos_i;
            } // end for 
        }
    // ----------------------------------------------------
    tcout << _T("\nStage 1 complexity=") << complexity() << std::endl;
    // ----------------------------------------------------
}

void  RoboPos::LearnMoves::testStage2()
{
#ifdef STAGE_OLD
    _complexity = 0;

    borders_t borders;
    defineTargetBorders(_target, _store, _stage2_params.draft_distance, borders);
    MainDirections md = MainDirectionsFactory(_robo);

    CINFO("borders: " << borders.size());

    _robo.reset();
    Point  robo_pos_base = _robo.position();

    Point  robo_pos_prev_i = robo_pos_base;
    Point  robo_pos_prev_j = robo_pos_base;

    Counters stats;
    frames_t  last_step = 1U;

    for (auto muscle_i : { RoboI::muscleByJoint(0, false),
                           RoboI::muscleByJoint(0, true) })
        for (auto muscle_j : { RoboI::muscleByJoint(1, false),
                               RoboI::muscleByJoint(1, true) })
        // if ( (muscle_i != muscle_j) && musclesValidUnion (muscle_i | muscle_j) )
        {
            Point  robo_pos_i;
            /* Нужен уменьшающийся шаг в зависимости от пройдённой дистанции */
            frames_t  last_step = _stage2_params.lasts_incr_value;

            auto  last_i_max = _robo.muscleMaxLast(muscle_i),
                 last_j_max = _robo.muscleMaxLast(muscle_j);

            bool target_contain = true;
            for (frames_t last_i = borders[muscle_i].min_lasts;
                 last_i > 0 && /* target_contain || */ last_i <= borders[muscle_i].max_lasts;
                 last_i += last_step)
            {
                for (frames_t last_j = borders[muscle_j].min_lasts /*- 50*/;
                     last_j > 0 && /* target_contain || */ last_j <= borders[muscle_j].max_lasts /* + 50 */;
                     last_j += last_step)
                {
                    Control controls;
                    controls.append({ muscle_i, 0U, last_i });
                    controls.append({ muscle_j, 0U /* start_j */, last_j });

                    Point predEnd = robo_pos_base;
                    predEnd += md.predict(muscle_i, last_i);
                    predEnd += md.predict(muscle_j, last_j);

                    //stats.fill(_robo, _target, controls, predEnd);
                    {
                        Trajectory  trajectory;
                        _robo.reset();
                        _robo.move(controls, trajectory);
                        //++_complexity;
                        bool model = _target.contain(predEnd);
                        bool real = _target.contain(_robo.position());
                        // ==================================================
                        stats.fill(model, real, _robo.position(), predEnd);
                        // ==================================================
                        _robo.reset();
                    }

                    /* типо на мишени */
                    if (_target.contain(predEnd))
                    {
                        Trajectory  trajectory;
                        _robo.reset();
                        _robo.move(controls, trajectory);

                        Point  robo_pos_j = _robo.position();
                        if (last_j == borders[muscle_j].min_lasts)
                            robo_pos_i = robo_pos_j;
                        //----------------------------------------------
                        target_contain = _target.contain(robo_pos_j);
                        // tcout << tstring(robo_pos_j) << std::endl;
                        //----------------------------------------------
                        Record  rec(robo_pos_j, robo_pos_base, robo_pos_j, controls, trajectory);
                        _store.insert(rec);

                        draftDistance(robo_pos_prev_j, robo_pos_j, last_step, last_j, last_j_max,
                                      _stage2_params.lasts_incr_value, _stage2_params.draft_distance);
                        robo_pos_prev_j = robo_pos_j;

                        ++_complexity;

                        try
                        { boost::this_thread::interruption_point(); }
                        catch (boost::thread_interrupted&)
                        {
                            CINFO("thread interrupted");
                            return;
                        }
                    }
                    else
                    { target_contain = false; }
                } // end for

                draftDistance(robo_pos_prev_i, robo_pos_i, last_step, last_i, last_i_max,
                              _stage2_params.lasts_incr_value, _stage2_params.draft_distance);
                robo_pos_prev_i = robo_pos_i;
            } // end for
        } // end if

    stats.print();
    // ----------------------------------------------------
    tcout << _T("\nStage 2 Complexity ") << complexity() << std::endl;
    // ----------------------------------------------------
    _robo.reset();
#endif // STAGE_OLD
}

void  RoboPos::LearnMoves::testStage3(OUT Robo::Trajectory &uncovered)
{
    // std::list<Point> uncovered;
    // _store.uncoveredTargetPoints (_target, uncovered);
    for (auto &pt : _target.coords())
    {
        adjacency_ptrs_t range;
        _store.adjacencyPoints(range, pt, 2. * _target.thickness());

        if (range.empty())
        {
            /* UNCOVERED */
            uncovered.push_back(pt);
        }
        else
        {
            double precision = _target.precision();
            if (ba::none_of(range, [&pt, precision](const Record *rec) {
                return  (boost_distance(pt, rec->hit) <= precision); }))
            { uncovered.push_back(pt); }
        } // end else
    } // end for
}

//------------------------------------------------------------------------------
#define HAND_TEST_BREAK 0

void  RoboPos::LearnMoves::testCoverTarget(IN Store &_store, IN RoboI &_robo, IN RecTarget &_target)
{
    /*    Покрыть всё в одно движение
     *    Начать двигать какой-то мышцой (варьировать длительность),
     *    К её движению сложить все комбинации остальных мышц.
     */
    _robo.reset();
    Point robo_base = _robo.position(), robo_prev = robo_base;

    try
    {
        CDEBUG(_T("robo.muscles_count = ") << int(_robo.musclesCount()));
        /* Возьмём первый мускул наугад */
        for (muscle_t muscle_i = 0; muscle_i < _robo.musclesCount(); ++muscle_i)
        {
            CDEBUG(muscle_i << _T("  ") << (int)_robo.muscleMaxLasts(muscle_i));
            for (muscle_t muscle_j = 0; muscle_j < _robo.musclesCount(); ++muscle_j)
            {
                if ((muscle_i != muscle_j) /*&& !musclesValidUnion(muscle_i, muscle_j)*/)
                    continue;

                CDEBUG(muscle_j << _T("  ") << (int)_robo.muscleMaxLasts(muscle_j));
                /* Попробуем его варьировать по длительности */
                for (frames_t last_i : boost::irange<frames_t>(5U, _robo.muscleMaxLasts(muscle_i) /*,2*/))
                {
                    Control control;
                    control.append({ muscle_i, 0U, last_i });
                    for (frames_t last_j : boost::irange<frames_t>(3U, _robo.muscleMaxLasts(muscle_j) /*,2*/))
                    {
                        for (frames_t start_j : boost::irange<frames_t>(5U, last_i))
                        {
                            boost::this_thread::interruption_point();

                            Trajectory  trajectory;
                            control.append({ muscle_j, start_j, last_j });

                            _robo.reset();
                            _robo.move(control);

                            // if ( _robo.position().x > x_leftBorder && _robo.position().x < x_rightBorder
                            //   && _robo.position().y < y_topBorder  && _robo.position().y > y_bottomBorder )

                            const Point&  robo_pos = _robo.position();
                            if (_target.contain(robo_pos))
                            {
                                _store.insert(Record(robo_pos, robo_base, robo_pos,
                                                    { muscle_i, muscle_j }, { 0, start_j }, { last_i, last_j }, 2U,
                                                    _robo.trajectory()));
                                // trajectories.push_back (make_shared<Trajectory> (trajectory));

                                if (_store.size() == 200000)
                                {
                                    _store.dump_off(_T("NewRoboI_200000_moves_tightly_save.bin"));
                                    CDEBUG("NewRoboI_200000_moves_tightly_saved");
                                }
                                else if (_store.size() == 400000)
                                {
                                    _store.dump_off(_T("NewRoboI_400000_moves_tightly_save.bin"));
                                    CDEBUG("NewRoboI_400000_moves_tightly_saved");
                                }
                                else if (_store.size() == 600000)
                                {
                                    _store.dump_off(_T("NewRoboI_600000_moves_tightly_save.bin"));
                                    CDEBUG("NewRoboI_600000_moves_tightly_saved");
                                }
                                else if (_store.size() == 1000000)
                                {
                                    _store.dump_off(_T("NewRoboI_1000000_moves_tightly_save.bin"));
                                    CDEBUG("NewRoboI_1000000_moves_tightly_saved");
                                    CDEBUG(muscle_i << ' ' << last_i);
                                    CDEBUG(muscle_j << ' ' << start_j << ' ' << last_j);
                                }
                                else if (_store.size() == 1500000)
                                {
                                    _store.dump_off(_T("NewRoboI_1500000_moves_tightly_save.bin"));
                                    CDEBUG("NewRoboI_1500000_moves_tightly_saved");
                                    CDEBUG(muscle_i << ' ' << last_i);
                                    CDEBUG(muscle_j << ' ' << start_j << ' ' << last_j );
                                }
                                else if (_store.size() == 2000000)
                                {
                                    _store.dump_off(_T("NewRoboI_2000000_moves_tightly_save.bin"));
                                    CDEBUG("NewRoboI_2000000_moves_tightly_saved");
                                    CDEBUG(muscle_i << ' ' << last_i);
                                    CDEBUG(muscle_j << ' ' << start_j << ' ' << last_j);
                                }
                            }
                            if (HAND_TEST_BREAK && boost_distance(robo_pos, _target.center()) > boost_distance(robo_prev, _target.center()))
                            {
                                CDEBUG("break");
                                break;
                            }
                            robo_prev = robo_pos;

                            try
                            { boost::this_thread::interruption_point(); }
                            catch (boost::thread_interrupted&)
                            {
                                CINFO("WorkingThread interrupted");
                                CDEBUG(muscle_i << ' ' << last_i);
                                CDEBUG(muscle_j << ' ' << start_j << ' ' << last_j);
                                return;
                            } // end catch
                        } // end for
                    } // end for
                } // end for
            } // end for
        } // end for
    } // end try
    catch (const std::exception &e)
    {
        CERROR(e.what());
        return;
    }
}

