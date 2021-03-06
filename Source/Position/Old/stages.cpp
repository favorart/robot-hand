

//    case 'j':
//    {
//        wd.pLM->testStage3(wd.canvas.uncoveredPointsList);  /* OLD */
//        //========================================
//        wd.canvas.hDynamicBitmapChanged = true;
//        InvalidateRect(hWnd, &myRect, TRUE);
//        break;
//    }
//
//    case 'k':
//    {
//        //========================================
//        WorkerThreadRunTask(wd, _T(" *** Stage 2 test ***  "),
//                            [](LearnMoves &lm) { lm.testStage2(); /* OLD */ },
//                            std::ref(*wd.pLM));
//        if (WorkerThreadTryJoin(wd))
//            InvalidateRect(hWnd, &myRect, TRUE);
//        //========================================
//        break;
//    }



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

