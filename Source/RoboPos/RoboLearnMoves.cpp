#include "StdAfx.h"

#include "Robo.h"
#include "RoboPos.h"
#include "RoboPosTour.h"
#include "RoboLearnMoves.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
using namespace Robo::Mobile;
using namespace Robo::NewHand;
//------------------------------------------------------------------------------
/// грубое покрытие всего рабочего пространства
void  RoboPos::LearnMoves::STAGE_1(IN bool verbose)
{
    std::shared_ptr<Tour> pTour{ new TourWorkSpace(store, robo, target) };
    pTour->STAGE_1(verbose);
}
/// Покрытие всей мишени не слишком плотно
void  RoboPos::LearnMoves::STAGE_2(IN bool verbose)
{
    std::shared_ptr<Tour> pTour{ new TourWorkSpace(store, robo, target) };
    pTour->STAGE_2(verbose);
}
/// Попадание в оставшиеся непокрытыми точки мишени
void  RoboPos::LearnMoves::STAGE_3(OUT Trajectory &uncovered, OUT size_t &complexity, IN bool verbose)
{
    size_t count = 0U;
    uncovered.clear();
    // -----------------------------------------------------
    for (const auto &pt : target.coords())
    {
        ++count;
        // ---------------------------------------------------
        tcout << _T("current: ") << count << _T(" / ")
              << target.coords().size();// << _T (" \r");
        
        // ---------------------------------------------------
        int tries = 32;
        Point p{ pt };
        // ---------------------------------------------------
        auto rec = std::ref(store.ClothestPoint(pt, stage3.side));
        while (tries >= 0 && boost_distance(rec.get().hit, pt) > target.precision())
        {
            complexity += gradientMethod_admixture(p, verbose);
            // -------------------------------------------------
            rec = std::ref(store.ClothestPoint(pt, stage3.side));
            // -------------------------------------------------
            double  rx = 0., ry = 0.;
            if ((tries % 3))
            {
                double min = target.precision() * target.precision();
                double max = target.precision() * 2.;

                rx = random(min, max);
                ry = random(min, max);

                rx = random(2) ? -rx : rx;
                ry = random(2) ? -ry : ry;
            }
            // -------------------------------------------------
            p = Point{ pt.x + rx, pt.y + ry };
            // -------------------------------------------------
            --tries;
            // ++tries;
            // if ( tries > 100 ) { break; }
        }

        // if ( tries > 11 )
        // { tcout << _T ("tries: ") << tries << _T (" \r"); }
        // else
        { tcout << _T(" \r"); }
        // ---------------------------------------------------
        {
            const Record &rec = store.ClothestPoint(pt, stage3.side);
            if (boost_distance(rec.hit, pt) > target.precision())
            { uncovered.push_back(pt); }
        }
    } // end for
    
    // -----------------------------------------------------
    tcout << _T("TOTAL Complexity: ") << complexity << "  minutes:" << double(complexity) / 60. << std::endl;
    tcout << _T("AVERAGE Complexity: ") << complexity / count << std::endl;
    tcout << _T("Uncovered: ") << uncovered.size() << std::endl << std::endl;
}

//------------------------------------------------------------------------------
void  RoboPos::LearnMoves::uncover(OUT Trajectory &uncovered)
{
    uncovered.clear();
    for (const auto &pt : target.coords())
    {
        const Record &rec = store.ClothestPoint(pt, stage3.side);
        // -------------------------------------------------------
        if (boost_distance(rec.hit, pt) > target.precision())
        { uncovered.push_back(pt); }
    }
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
    const Record *pRec = store.exactRecordByControl(controls);
    if (pRec) // visited.find (h) != visited.end () )
    {
        // if ( pRec ) { 
        hit = pRec->hit;
        // } else { throw exception ("handAct: Not in Store"); }
    }
    else
    {
        Trajectory trajectory;
        robo.reset();
        robo.move(controls, trajectory);
        hit = robo.position();
        //robo.reset();
        // -----------------------------------------------
        Record rec(aim, base_pos, hit, controls, trajectory);
        store.insert(rec);
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
    size_t complexity = 0U;

    robo.reset();
    Point  robo_pos_base = robo.position();

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
            frames_t  lasts_step = stage1.lasts_incr_value;

            auto last_i_max = robo.muscleMaxLast(muscle_i),
                last_j_max = robo.muscleMaxLast(muscle_j);
            /* Попробуем его варьировать по длительности */
            for (frames_t last_i = stage1.lasts_init_value; last_i < last_i_max; last_i += lasts_step)
            {
                for (frames_t last_j = stage1.lasts_init_value; last_j < last_j_max; last_j += lasts_step)
                {
                    Control control;
                    control.append({ muscle_i, 0U, last_i });
                    control.append({ muscle_j, 0U /* start_j */, last_j });

                    Trajectory  trajectory;
                    robo.reset();
                    robo.move(control, trajectory);

                    Point robo_pos_j = robo.position();
                    if (last_j == stage1.lasts_init_value)
                        robo_pos_i = robo_pos_j;

                    Record rec(robo_pos_j, robo_pos_base, robo_pos_j, control, trajectory);
                    store.insert(rec);

                    draftDistance(robo_pos_prev_j, robo_pos_j,
                                  lasts_step, last_j, last_j_max,
                                  stage1.lasts_incr_value, stage1.draft_distance);
                    robo_pos_prev_j = robo_pos_j;

                    ++complexity;

                    try
                    { boost::this_thread::interruption_point(); }
                    catch (boost::thread_interrupted&)
                    { return; } // end catch

                } // end for

                draftDistance(robo_pos_prev_i, robo_pos_i,
                              lasts_step, last_i, last_i_max,
                              stage1.lasts_incr_value, stage1.draft_distance);
                robo_pos_prev_i = robo_pos_i;

            } // end for 
        }
    // ----------------------------------------------------
    tcout << _T("\nStage 1 Complexity ") << complexity << std::endl;
    // ----------------------------------------------------
}

void  RoboPos::LearnMoves::testStage2()
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
    size_t complexity = 0U;

    borders_t borders;
    defineTargetBorders(target, store, stage1.draft_distance, borders);
    NewHand::MainDirections MDs = NewHand::MainDirectionsFactory(robo);

    tcout << _T("borders: ") << borders.size() << std::endl;

    robo.reset();
    Point  robo_pos_base = robo.position();

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
            frames_t  last_step = stage2.lasts_incr_value;

            auto  last_i_max = robo.muscleMaxLast(muscle_i),
                last_j_max = robo.muscleMaxLast(muscle_j);

            bool target_contain = true;
            for (frames_t last_i = borders[muscle_i].first;
                 last_i > 0 && /* target_contain || */ last_i <= borders[muscle_i].second;
                 last_i += last_step)
            {
                for (frames_t last_j = borders[muscle_j].first /*- 50*/;
                     last_j > 0 && /* target_contain || */ last_j <= borders[muscle_j].second /* + 50 */;
                     last_j += last_step)
                {
                    Control control;
                    control.append({ muscle_i, 0U, last_i });
                    control.append({ muscle_j, 0U /* start_j */, last_j });

                    Point predEnd = robo_pos_base;
                    predEnd += MDs.predict(muscle_i, last_i);
                    predEnd += MDs.predict(muscle_j, last_j);

                    stats.fill(robo, target, control, predEnd);

                    /* типо на мишени */
                    if (target.contain(predEnd))
                    {
                        Trajectory  trajectory;
                        robo.reset();
                        robo.move(control, trajectory);

                        Point  robo_pos_j = robo.position();
                        if (last_j == borders[muscle_j].first)
                            robo_pos_i = robo_pos_j;
                        //----------------------------------------------
                        target_contain = target.contain(robo_pos_j);
                        // tcout << tstring(robo_pos_j) << std::endl;
                        //----------------------------------------------
                        Record  rec(robo_pos_j, robo_pos_base, robo_pos_j, control, trajectory);
                        store.insert(rec);

                        draftDistance(robo_pos_prev_j, robo_pos_j, last_step,
                                      last_j, last_j_max, stage2.lasts_incr_value,
                                      stage2.draft_distance);
                        robo_pos_prev_j = robo_pos_j;

                        ++complexity;

                        try
                        { boost::this_thread::interruption_point(); }
                        catch (boost::thread_interrupted&)
                        { return; } // end catch

                    }
                    else
                    { target_contain = false; }
                } // end for

                draftDistance(robo_pos_prev_i, robo_pos_i, last_step, last_i, last_i_max, stage2.lasts_incr_value, stage2.draft_distance);
                robo_pos_prev_i = robo_pos_i;
            } // end for
        } // end if

    stats.print();
    // ----------------------------------------------------
    tcout << _T("\nStage 2 Complexity ") << complexity << std::endl;
    // ----------------------------------------------------
    robo.reset();
}

void  RoboPos::LearnMoves::testStage3(OUT std::list<Point> &uncovered)
{
    // std::list<Point> uncovered;
    // store.uncoveredTargetPoints (target, uncovered);
    for (auto &pt : target.coords())
    {
        adjacency_ptrs_t  range;
        store.adjacencyPoints(range, pt, 2. * target.thickness());

        if (range.empty())
        {
            /* UNCOVERED */
            uncovered.push_back(pt);
        }
        else
        {
            double precision = target.precision();
            if (boost::algorithm::none_of(range, [&pt, precision](const Record *rec)
                                          // (const std::shared_ptr<Record> &rec)
            { return  (boost_distance(pt, rec->hit) <= precision); }))
            {
                /* UNCOVERED */
                uncovered.push_back(pt);

            } // end if

        } // end else
    } // end for
}

//------------------------------------------------------------------------------
void  RoboPos::LearnMoves::testCoverTarget(IN Store &store, IN RoboI &robo, IN RecTarget &target)
{
    /*    Покрыть всё в одно движение
    *    Начать двигать какой-то мышцой (варьировать длительность),
    *    К её движению сложить все комбинации остальных мышц.
    */
    robo.reset();
    Point robo_base = robo.position();
    Point robo_prev = robo.position();

    try
    {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
        tcout << _T("robo.muscles_.size = ") << (int)robo.muscles_.size() << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
        /* Возьмём первый мускул наугад */
        for (muscle_t muscle_i = 0; muscle_i < robo.musclesCount(); ++muscle_i)
        {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
            tcout << muscle_i << _T("  ") << (int)robo.muscleMaxLast(muscle_i) << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
            for (muscle_t muscle_j = 0; muscle_j < robo.musclesCount(); ++muscle_j)
            {
                if ((muscle_i != muscle_j) /*&& !musclesValidUnion(muscle_i, muscle_j)*/)
                    continue;
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                tcout << muscle_j << _T("  ") << (int)robo.muscleMaxLast(muscle_j) << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                /* Попробуем его варьировать по длительности */
                for (frames_t last_i : boost::irange<frames_t>(5U, robo.muscleMaxLast(muscle_i) /*,2*/))
                {
                    Control control;
                    control.append({ muscle_i, 0U, last_i });
                    for (frames_t last_j : boost::irange<frames_t>(3U, robo.muscleMaxLast(muscle_j) /*,2*/))
                    {
                        // for ( frames_t start_j : boost::irange<frames_t> (5U, last_i, 2) ) // ?? + inertial_lasts
                        for (frames_t start_j : boost::irange<frames_t>(5U, last_i))
                        {
                            boost::this_thread::interruption_point();

                            Trajectory  trajectory;
                            control.append({ muscle_j, start_j, last_j });

                            robo.reset();
                            robo.move(control, trajectory);

                            // if ( robo.position().x > x_leftBorder && robo.position().x < x_rightBorder
                            //   && robo.position().y < y_topBorder && robo.position().y > y_bottomBorder )

                            const Point&  robo_pos = robo.position();
                            if (target.contain(robo_pos))
                            {
                                store.insert(Record(robo_pos, robo_base, robo_pos,
                                                    { muscle_i, muscle_j }, { 0, start_j }, { last_i, last_j }, 2U,
                                                    trajectory));
                                // trajectories.push_back (make_shared<Trajectory> (trajectory));

                                if (store.size() == 200000)
                                {
                                    store.save(_T("NewRoboI_200000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                                    tcout << _T("NewRoboI_200000_moves_tightly_saved") << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                                }
                                else if (store.size() == 400000)
                                {
                                    store.save(_T("NewRoboI_400000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                                    tcout << _T("NewRoboI_400000_moves_tightly_saved") << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                                }
                                else if (store.size() == 600000)
                                {
                                    store.save(_T("NewRoboI_600000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                                    tcout << _T("NewRoboI_600000_moves_tightly_saved") << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                                }
                                else if (store.size() == 1000000)
                                {
                                    store.save(_T("NewRoboI_1000000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                                    tcout << _T("NewRoboI_1000000_moves_tightly_saved") << std::endl;
                                    tcout << muscle_i << ' ' << last_i << std::endl;
                                    tcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                                }
                                else if (store.size() == 1500000)
                                {
                                    store.save(_T("NewRoboI_1500000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                                    tcout << _T("NewRoboI_1500000_moves_tightly_saved") << std::endl;
                                    tcout << muscle_i << ' ' << last_i << std::endl;
                                    tcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                                }
                                else if (store.size() == 2000000)
                                {
                                    store.save(_T("NewRoboI_2000000_moves_tightly_save.bin"));
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                                    tcout << _T("NewRoboI_2000000_moves_tightly_saved") << std::endl;
                                    tcout << muscle_i << ' ' << last_i << std::endl;
                                    tcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                                }
                            }

#define   _HAND_TEST_BREAK
#ifdef    _HAND_TEST_BREAK
                            if (boost_distance(robo_pos, target.center()) > boost_distance(robo_prev, target.center()))
                            {
                                // #ifdef    _HAND_TEST_CONSOLE_PRINTF
                                // tcout << _T ("break") << std::endl;
                                // #endif // _HAND_TEST_CONSOLE_PRINTF
                                break;
                            }
                            robo_prev = robo_pos;
#endif // _HAND_TEST_BREAK
                            try
                            { boost::this_thread::interruption_point(); }
                            catch (boost::thread_interrupted&)
                            {
#ifdef    _HAND_TEST_CONSOLE_PRINTF
                                tcout << _T("WorkingThread interrupted") << std::endl;
                                tcout << muscle_i << ' ' << last_i << std::endl;
                                tcout << muscle_j << ' ' << start_j << ' ' << last_j << std::endl;
#endif // _HAND_TEST_CONSOLE_PRINTF
                                return;
                            } // end catch
                        } // end for
                    } // end for
                } // end for
            } // end for
        } // end for
    } // end try
#ifdef    _HAND_TEST_CONSOLE_PRINTF
    catch (std::exception &ex)
    {
        tcout << ex.what() << std::endl;
#else
    catch (...)
    {
#endif // _HAND_TEST_CONSOLE_PRINTF
        return;
    }
    }

