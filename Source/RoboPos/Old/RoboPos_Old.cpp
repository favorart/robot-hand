#include "StdAfx.h"
#include "RoboPos.h"
#include "RoboPosMDirections.h"


using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!! Борьба с КОЛИЧЕСТВОМ, которая перетягивает начальную точку !!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void  RoboPos::LearnMoves::Mean(IN const Point &aim,
                                OUT Trajectory   *trajectory,
                                OUT Trajectories *trajectories)
{
    complexity = 0U;
    // -----------------------------------------------    
    robo.reset();
    Point robo_position = robo.position(), robo_pos_base = robo_position;
    // -----------------------------------------------
    double  distance = boost_distance(robo_position, aim);
    double  next_distance = distance;
    // -----------------------------------------------
    do
    {
        if (next_distance < distance)
        { distance = next_distance; }
        // -----------------------------------------------
        // Point  min (aim.x - side, aim.y - side),
        //        max (aim.x + side, aim.y + side);
        // -----------------------------------------------
#ifdef _DEBUG_PRINT
        tcout << std::endl;
        tcout << std::endl << _T("RoboPos::LearnMoves::Mean") << std::endl;
        tcout << _T("min=") << min << _T(" max=") << max << std::endl;
#endif // _DEBUG_PRINT
        // -----------------------------------------------
        adjacency_ptrs_t  range;
        // size_t  count = store.adjacencyPoints (range, aim, side);
        // size_t  count = store.adjacencyRectPoints<adjacency_refs_t, ByP> (range, min, max);
        size_t  count = store.adjacencyByPBorders(range, aim, side);
        if (count == 0U)
        {
#ifdef _DEBUG_PRINT
            // tcout << _T ("ERROR: empty adjacency") << std::endl;
#endif // _DEBUG_PRINT
        // -------------
            return;
        }
        // -----------------------------------------------
        if (trajectories)
        {
            for (auto &pRec : range)
            { trajectories->push_back(pRec->trajectory); }
        }
        // -----------------------------------------------
        // ClosestPredicate  cp (aim);
        // range.sort (cp);
        // -----------------------------------------------
        double all_distance = 0.;
        if (weighted_mean)
        {
            for (auto &pRec : range)
            {
                all_distance += boost_distance(aim, pRec->hit);
                // -----------------------------------------------
#ifdef _DEBUG_PRINT
          // tcout << pRec->controls () << std::endl;
#endif // _DEBUG_PRINT
            }
#ifdef _DEBUG_PRINT
            tcout << std::endl;
#endif // _DEBUG_PRINT
        }
        // -----------------------------------------------
        Control  controls;
        for (auto m : robo.muscles_)
        {
            Control  control(m, 0U, 0U);
            // -----------------------------------------------
            frames_t  n_control_summands = 0U;
            for (auto &pRec : range)
                for (auto &c : pRec->controls)
                {
                    if (c.muscle == m)
                    {
                        if (weighted_mean)
                        {
                            /* взвешенное cреднее арифметическое */
                            double weight = boost_distance(aim, pRec->hit) / all_distance;
                            control.start += weight * c.start;
                            control.last += weight * c.last;
                        }
                        else
                        {
                            /* обычное cреднее арифметическое */
                            control.start += c.start;
                            control.last += c.last;
                            ++n_control_summands;
                        } // end else
                    } // end if
                } // end for-for
                  // -----------------------------------------------
            if (!weighted_mean)
            {
                if (n_control_summands) control.last /= n_control_summands;
                if (n_control_summands) control.start /= n_control_summands;
            }
            // -----------------------------------------------
            if (control.last)
            { controls.push_back(control); }
        }
        // -----------------------------------------------
        for (auto j : robo.joints_)
        {
            auto mo = RoboI::muscleByJoint(j, true);
            auto mc = RoboI::muscleByJoint(j, false);

            auto it_o = std::find(controls.begin(), controls.end(), mo);
            auto it_c = std::find(controls.begin(), controls.end(), mc);

            if (it_o != controls.end() && it_c != controls.end())
                if (it_o->last > it_c->last)
                { it_c->start = it_o->last + 1U; }
                else
                { it_o->start = it_c->last + 1U; }
        }
        controls.sort();
        // -----------------------------------------------
        {
            Trajectory trajectory_;
            robo.move(controls.begin(), controls.end(), &trajectory_);
            robo.reset();
            robo_position = robo.position();

            ++complexity;
            // -----------------------------------------------
            Record  rec(aim, robo_pos_base,
                                   robo_position, controls,
                                   trajectory_);
            store.insert(rec);
            // -----------------------------------------------
            if (trajectory)
            { *trajectory = trajectory_; }
        }
        // -----------------------------------------------
        next_distance = boost_distance(robo_position, aim);
        if (next_distance < distance)
        { distance = next_distance; }

        if (precision >= distance)
        { break; }

#ifdef _DEBUG_PRINT
        tcout << _T("Prec: ") << next_distance << std::endl;
#endif // _DEBUG_PRINT
        // -----------------------------------------------
        if (distance > precision)
        { rundownFull(aim, robo_position); }

        next_distance = boost_distance(robo_position, aim);
        if (next_distance < distance)
        { distance = next_distance; }

        if (complexity > 2000) // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        { break; }

        if (precision >= distance)
        { break; }
        // -----------------------------------------------
        side -= side_decrease_step;
        // -----------------------------------------------
        controls.clear();
        range.clear();
        // -----------------------------------------------
    } while (precision < distance || distance != next_distance);
    // -----------------------------------------------
#ifdef _DEBUG_PRINT_RES
    tcout << _T("ResPrec: ") << distance << std::endl;
    tcout << _T("Complexity: ") << complexity << std::endl << std::endl;
#endif // _DEBUG_PRINT_RES
}

void  RoboPos::LearnMoves::Close(IN const Point &aim,
                                 OUT Trajectory    *trajectory,
                                 OUT Trajectories  *trajectories)
{
    complexity = 0U;
    // -----------------------------------------------    
    robo.reset();
    Point  robo_position = robo.position(),
           robo_pos_base = robo.position();
    // -----------------------------------------------
    double  distance;
    // -----------------------------------------------
    Point  min(aim.x - side, aim.y - side),
           max(aim.x + side, aim.y + side);
    // -----------------------------------------------
#ifdef _DEBUG_PRINT
    tcout << std::endl;
    tcout << std::endl << _T("RoboPos::LearnMoves::Close") << std::endl;
    tcout << _T(" min=") << min << _T("  max=") << max << std::endl;
#endif // _DEBUG_PRINT
    // -----------------------------------------------
    adjacency_ptrs_t  range;
    // auto count = store.adjacencyPoints (range, aim, side);
    auto  count = store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    if (count == 0U)
    {
#ifdef _DEBUG_PRINT
        // tcout << _T ("ERROR: empty adjacency") << std::endl;
#endif // _DEBUG_PRINT
      // -------------
        return;
    }
    // -----------------------------------------------
    if (trajectories)
    {
        for (auto &pRec : range)
        { trajectories->push_back(pRec->trajectory); }
    }
    // -----------------------------------------------
    ClosestPredicate  cp(aim);
    auto it_min = boost::range::min_element(range, cp);
    // -----------------------------------------------
    robo_position = range.front()->hit;

    distance = boost_distance(robo_position, aim);
    if (distance > precision)
    { rundownFull(aim, robo_position); }

    // -----------------------------------------------
#ifdef _DEBUG_PRINT_RES
    tcout << _T("ResPrec: ") << distance << std::endl;
    tcout << _T("Complexity: ") << complexity << std::endl << std::endl;
#endif // _DEBUG_PRINT_RES
}

//------------------------------------------------------------------------------
void  RoboPos::LearnMoves::rundownMethod_old(IN const Point &aim,
                                             IN const Control &init_controls,
                                             IN Point &robo_position)
{
    robo.reset();
    Point robo_pos_base = robo.position();
    Point robo_pos = robo_position;
    // -----------------------------------------------
    Control controls;
    auto iter = controls.begin();
    for (joint_t j = 0; j < robo.jointsCount(); ++j)
    {
        auto mo = RoboI::muscleByJoint(j, true);
        auto mc = RoboI::muscleByJoint(j, false);

        auto it_o = std::find(init_controls.begin(), init_controls.end(), mo);
        auto it_c = std::find(init_controls.begin(), init_controls.end(), mc);

        if (it_o != init_controls.end() && it_c != init_controls.end())
        {
            controls.append(iter, *it_o);
            controls.append(iter, *it_c);
        }
        else if (it_o != init_controls.end())
        {
            controls.append(iter, *it_o);
            controls.append(iter, Control(mc, it_o->last + 1, 50));
        }
        else if (it_c != init_controls.end())
        {
            controls.append(iter, Control(mo, it_c->last + 1, 50));
            controls.append(iter, *it_c);
        }
        else
        {
            controls.append(iter, Control(mo, 0, 51));
            controls.append(iter, Control(mc, 52, 50));
        }
    }
    // -----------------------------------------------
    controls.sort();
    // -----------------------------------------------
    std::list<std::shared_ptr<Control>>  ordered_controls;
    for (auto &c : controls)
    { ordered_controls.push_back(std::make_shared<Control>(c)); }
    ordered_controls.sort([](const std::shared_ptr<Control> &sca,
                             const std::shared_ptr<Control> &scb) {
        return sca->muscle < scb->muscle;
    });
    // -----------------------------------------------
    frames_t lasts_step = 1U;
    double distance = boost_distance(robo_position, aim);
    // -----------------------------------------------
    while (precision < distance)
    {
        // -----------------------------------------------
        auto it_straight = ordered_controls.begin();
        auto it_opposite = std::next(it_straight);

        if ((**it_straight).last < (**it_opposite).last)
        { std::swap(it_straight, it_opposite); }
        // -----------------------------------------------
        double best_distance = distance;
        for (auto j : robo.joints_)
        {
            frames_t last_straight = (**it_straight).last;
            frames_t last_opposite = (**it_opposite).last;

            frames_t best_last_straight = 0U;
            frames_t best_last_opposite = 0U;

            double next_distance = distance;
            double prev_distance = distance;
            // ---------------------------------------------------------
            do
            {
                prev_distance = next_distance;
                (**it_straight).last += lasts_step;
                (**it_opposite).start += lasts_step;

#ifdef _DEBUG_PRINT
                for (auto &c : controls)
                { tcout << c << std::endl; }
#endif // _DEBUG_PRINT

                roboAct(aim, controls, robo_pos);
                next_distance = boost_distance(robo_pos, aim);

                if (next_distance >= prev_distance)
                {
                    (**it_straight).last -= lasts_step;
                    (**it_opposite).start -= lasts_step;

                    do
                    {
                        prev_distance = next_distance;
                        (**it_opposite).last -= lasts_step;
                        roboAct(aim, controls, robo_pos);
                        next_distance = boost_distance(robo_position, aim);
                    } while (next_distance < prev_distance);
                }

            } while (next_distance < prev_distance);

            if (prev_distance < best_distance)
            {
                best_last_straight = (**it_straight).last;
                best_last_opposite = (**it_opposite).last;

                best_distance = prev_distance;
            }
            // ---------------------------------------------------------
            (**it_straight).last = last_straight;
            (**it_opposite).last = last_opposite;

            do
            {
                prev_distance = next_distance;
                (**it_straight).last -= lasts_step;
                (**it_opposite).start -= lasts_step;

                roboAct(aim, controls, robo_pos);
                next_distance = boost_distance(robo_pos, aim);

                if (next_distance >= prev_distance)
                {
                    (**it_straight).last += lasts_step;
                    (**it_opposite).start += lasts_step;
                    do
                    {
                        prev_distance = next_distance;
                        (**it_opposite).last += lasts_step;

                        roboAct(aim, controls, robo_pos);
                        next_distance = boost_distance(robo_pos, aim);

                    } while (next_distance < prev_distance);
                }

            } while (next_distance < prev_distance);

            if (prev_distance < best_distance)
            {
                best_last_straight = (**it_straight).last;
                best_last_opposite = (**it_opposite).last;

                best_distance = prev_distance;
            }
            // ---------------------------------------------------------
            (**it_straight).last = best_last_straight;
            (**it_opposite).last = best_last_opposite;

            distance = best_distance;

#ifdef _DEBUG_PRINT
            tcout << _T("prec: ") << best_distance << std::endl;
#endif // _DEBUG_PRINT
            ++it_straight;
            ++it_opposite;
        }
    }
    // -----------------------------------------------
#ifdef _DEBUG_PRINT_RES
    tcout << _T("prec: ") << distance << std::endl;
#endif // _DEBUG_PRINT_RES
}
//------------------------------------------------------------------------------
void  testCover(IN  Store &store, IN RoboI &robo, OUT Trajectories &trajectories)
{
    /*    Покрыть всё в одно движение
     *    Начать двигать какой-то мышцой (варьировать длительность),
     *    К её движению сложить все комбинации остальных мышц.
     */
    robo.reset();
    Point robo_base = robo.position();
    /* Возьмём первый мускул наугад */
    for (muscle_t muscle_i = 0; muscle_i < robo.musclesCount(); ++muscle_i)
    {
        Trajectory  trajectory;
        /* Попробуем его варьировать по длительности */
        for (frames_t last_i : boost::irange<frames_t>(1U, robo.muscleMaxLast(muscle_i) / 2, 10))
        {
            Control control({ muscle_i, 0U, last_i });
            for (muscle_t muscle_j = 0; muscle_j < robo.musclesCount(); ++muscle_j)
            {
                if ((muscle_i == muscle_j) /* && !musclesValidUnion(muscle_i, muscle_j) */)
                    continue;

                for (frames_t last_j : boost::irange<frames_t>(1U, robo.muscleMaxLast(muscle_j) / 2, 10))
                {
                    robo.move(control, trajectory);
                    control.append({ muscle_j, last_i + 1, last_j });

                    // auto& index = store.get<Record::ByC> ();
                    // auto& index = store.get<Record::ByP> ();
                    // index.equal_range (robo.position());

                    store.insert(Record(robo.position(), robo_base, robo.position(), control, trajectory));
                    // trajectories.push_back (make_shared<Trajectory> (trajectory));
                }
            }
        }
        // for ( auto muscle_j : robo.muscles_ )
        // {
        //   if ( !(muscle_j & muscles) && musclesValidUnion (muscle_i | muscles) )
        //   {
        //     for ( frames_t last_i : boost::irange<frames_t> (1U, 126U, 10) ) //robo.muscleMaxLast (muscle_i)) )
        //     {
        //       robo.move (muscle_i | muscle_j, last_i, trajectory);
        //       store.insert (Record (robo.position(), robo_base, robo.position(),
        //                    { muscle_i }, { 0 }, { last_i, 126U - last_i }, 1U,
        //                    trajectory)
        //                    );
        //     }
        //   } // end if
        // } // end for
        // 
        //   // trajectories.push_back (make_shared<Trajectory> (trajectory));
        //   /*  Теперь в каждый такт длительности первого мускула,
        //    *  включим второй и будет варьировать его длительность
        //    */
        //   testCover (store, robo, muscles | muscle_i, trajectories);
        // 
        // } // end for
        //} // end if
    } // end for
}


//------------------------------------------------------------------------------
struct Direction
{
    double   angle = 0.; // with Ox
    Point    direction{};
    muscle_t muscle = Robo::MInvalid;
    Point    norm_dir{};

    struct Ellipse
    {};

    Direction() = default;
    Direction(Point direction, muscle_t muscle) :
        direction(direction),
        muscle(muscle),
        angle(atan2(direction.y, direction.x))
    {}

    double  cos_sim(const Direction &d) const
    {
        auto  ad = direction;
        auto  bd = d.direction;
        /* cosine similarity */
        return  (ad.x * bd.x + ad.y * bd.y)
         / (sqrt(ad.x * ad.x + ad.y * ad.y)
         *  sqrt(bd.x * bd.x + bd.y * bd.y));
    }

    void draw(HDC hdc)
    {
        if (!norm_dir.x && !norm_dir.y)
            norm_dir = Point(0.1 * cos(angle), 0.1 * sin(angle));

        MoveToEx(hdc, Tx(0.), Ty(0.), NULL);
        LineTo(hdc, Tx(norm_dir.x), Ty(norm_dir.y));
    }

};
void RoboPos::LearnMoves::getTargetCenter(IN Store &store, IN RoboI &robo, OUT Point &center)
{
    robo.reset();
    Point start = robo.position();

    std::vector<Direction> directions{};
    directions.reserve(robo.musclesCount());

    //robo.selectControl ()
    // for ( auto muscle_i : robo.muscles_ )
    for (auto iCtrl = 0U; iCtrl < robo.controlsCount; ++iCtrl)
    {
        auto muscle_i = robo.selectControl(iCtrl);
    
        robo.move(muscle_i, 1U);
        // robo.step (muscle_i, 1U); robo.step ();
    
        directions.push_back(Direction(Point(robo.position().x - start.x,
                                             robo.position().y - start.y), muscle_i));
    }
    // --------------------
    Direction   direct = Direction(Point(center.x - start.x,
                                         center.y - start.y), EmptyMov);
    
    Direction  &d = *std::min_element(directions.begin(), directions.end(),
                                      [&direct](const Direction &a, const Direction &b) { return  direct.cos_sim(a) < direct.cos_sim(b); });
    // { return  boost_distance (direct.direction, a.direction)
    //         > boost_distance (direct.direction, b.direction); });
    // --------------------
    
    auto max_last = robo.muscleMaxLast(d.control);
    robo.move(d.control, max_last);
    
    double center_distance = boost_distance(center, start);
    double maxdir_distance = boost_distance(robo.position(), start);
    
    frames_t  last = static_cast<frames_t> ((center_distance * max_last) / maxdir_distance);
    robo.move(d.control, last);
}
//------------------------------------------------------------------------------
bool  RoboPos::LearnMoves::tryToHitTheAim(IN Store &store, IN RoboI &robo, IN const Point &aim)
{
    const size_t hit_tries = 20;
    const double hit_precision = 2.5;

    size_t complexity = 0U;

    // borders (store, target);
    NewHand::MainDirections MDs = NewHand::MainDirectionsFactory(robo);

    robo.reset();
    bool result = 0;
    std::list<std::shared_ptr<Record>>  exact_range;
    /* While there is no exact trajectory */
    for (size_t n_try = 0U; (result = exact_range.empty()) || n_try < hit_tries; ++n_try)
    {
        store.adjacencyPoints(exact_range, aim, hit_precision);
        /* ?????? */
    }
    return !result;
}
//------------------------------------------------------------------------------
static void  testCoverTarget1(IN Store &store, IN RoboI &robo, IN RecTarget &target)
{
    robo.reset();
    /* Fixate the aim point */
    auto aim = target.coords()[45];
    double prev_distance = boost_distance(robo.position(), aim);

    double    best_distance = prev_distance;
    muscle_t  best_muscle = Robo::MInvalid;
    frames_t  best_last = 0;

    // while ( aim.y () < robo.position().y )
    {
        /* Try each muscle */
        for (auto i : boost::irange<size_t>(1U, robo.controlsCount))
        {
            muscle_t muscle = Robo::MInvalid;
            auto muscle = robo.selectControl(i);
            auto last = robo.muscleMaxLast(muscle);

            robo.step(muscle);
            robo.step();
            /* Find the best muscle by distance */
            double cur_distance = boost_distance(robo.position(), aim);
            if (cur_distance < best_distance)
            {
                best_muscle = muscle;
                best_last = last;
                best_distance = cur_distance;
            }
            robo.reset();
        }

        robo.step(best_muscle);
        prev_distance = best_distance;
        double cur_distance = best_distance;
        auto last_ = 1U;

        do
        {
            prev_distance = cur_distance;
            //-----------
            robo.step();
            //-----------
            cur_distance = boost_distance(robo.position(), aim);
            ++last_;
        } while ((prev_distance >= cur_distance) && (last_ <= best_last));

        if (!robo.moveEnd())
            robo.step(best_muscle);
        
        while (!robo.moveEnd())
            robo.step();
    }
    //=========================================================================
    // // const to target
    // const boost_point2_t  tlu (target.Min ().x, target.Max ().y),
    //                       tld (target.Min ().x, target.Min ().y),
    //                       trd (target.Max ().x, target.Min ().y),
    //                       tru (target.Max ().x, target.Max ().y);
    // 
    // const array<boost_point2_t, 4> target_corners = { tlu, tld, trd, tru };
    // const boost_point2_t *t_closest;
    // double distance = 0.;
    //// Create the tree of possible passes
    //for ( MusclesEnum muscle_i : muscles )
    //{
    //  // auto h = robo.position();
    //  for ( uint_t last_i = 1U; last_i < robo.muscleMaxLast (muscle_i); ++last_i )
    //  {
    //    std::list<Point> trajectory;
    //    
    //    
    //    const boost_point2_t pos = robo.position();
    //    t_closest = &target_corners[0];
    //    distance = boost::geometry::distance (t_closest, pos);
    //    for ( size_t i = 1U; i < target_corners.size (); ++i )
    //    {
    //      double d = boost::geometry::distance (target_corners[i], pos);
    //      if ( distance > d )
    //      {
    //        distance = d;
    //        t_closest = &target_corners[i];
    //      }
    //    }
    //    // robo.move (muscle_i, last_i, trajectory);
    //    robo.step (muscle_i);
    //    // while ( last-- )
    //    for ( auto last_ = 1U; last_ <= last_i; ++last_ )
    //    {
    //      robo.step ();
    //      const boost_point2_t pos = robo.position();
    //      double d = boost::geometry::distance (pos, t_closest);
    //    }
    //    if ( !robo.isMoveEnd () )
    //      robo.step (muscle_i);
    //    while ( !robo.isMoveEnd () )
    //      robo.step ();
    //{
    //  const Point &aim = robo.position();
    //  Record rec (aim, aim,
    //              { muscle_i }, { 0 }, { last_i },
    //              1U, trajectory);
    //  store.insert (rec);
    //}
    //=========================================================================
    //// tcout << muscle_i << ' ' << hi << ' ' << last_i << std::endl;
    //insert (trajectories, trajectory);
    //if ( nesting > 1U )
    //  for ( MusclesEnum muscle_j : muscles )
    //  {
    //    if ( (muscle_i == muscle_j) || !musclesValidUnion (muscle_i | muscle_j) )
    //      // if ( j == i )
    //      continue;
    //    Point cur1 = robo.position();
    //    for ( uint_t last_j = 1U; last_j < robo.muscleMaxLast (muscle_j) / 2; ++last_j )
    //    {
    //      std::list<Point>::iterator tail_j = trajectory.end (); --tail_j;
    //      robo.move (muscle_j, last_j, trajectory);
    //      {
    //        const Point &aim = robo.position();
    //        store.insert (Record (aim, aim,
    //        { muscle_i, muscle_j },
    //        { 0, last_i },
    //        { last_i, last_j },
    //                      2U,
    //                      trajectory)
    //                      );
    //      }
    //      // tcout << muscles[j] << ' ' << hj << ' ' << last_j << std::endl;
    //      ++tail_j;
    //      // std::list::splice () allows you to concatenate two std::list's in constant time.
    //      insert (trajectories, trajectory);
    //      //=================================================
    //      if ( nesting > 2U )
    //        for ( MusclesEnum muscle_k : muscles )
    //        {
    //          if ( (muscle_i == muscle_k || muscle_k == muscle_j)
    //              || !musclesValidUnion (muscle_i | muscle_j | muscle_k) )
    //            // if ( k == i || k == j )
    //            continue;
    //          for ( auto last_k = 1U; last_k < robo.muscleMaxLast (muscle_k) / 2; ++last_k )
    //          {
    //            std::list<Point>::iterator tail_k = trajectory.end (); --tail_k;
    //            robo.move (muscle_k, last_k, trajectory);
    //            {
    //              const Point &aim = robo.position();
    //              store.insert (Record (aim, aim,
    //              { muscle_i, muscle_j, muscle_k },
    //              { 0, last_i, last_i + last_j },
    //              { last_i, last_j, last_k },
    //                            3U,
    //                            trajectory)
    //                            );
    //            }
    //            // tcout << muscles[k] << ' ' << hk << ' ' << last_k << std::endl;
    //            ++tail_k;
    //            insert (trajectories, trajectory);
    //            trajectory.erase (tail_k, trajectory.end ());
    //            robo.set (jointByMuscle (muscle_k), { (jointByMuscle (muscle_k) == Elbow) ? 70. : 0. });
    //          }
    //        }
    //      //=================================================
    //      trajectory.erase (tail_j, trajectory.end ());
    //      //a.insert (a.end (), b.begin (), b.end ());
    //      robo.set (jointByMuscle (muscle_j), { (jointByMuscle (muscle_j) == Elbow) ? 70. : 0. });
    //    }
    //  }
    //     robo.set (jointByMuscle (muscle_i), { (jointByMuscle (muscle_i) == Elbow) ? 70. : 0. });
    //   }
    // }
    //robo.reset();
}
//------------------------------------------------------------------------------

//void  /**/testCover (Store &store, RoboI &robo,
//                            MusclesEnum muscles /* recursive */,
//                            int recursive,
//                            std::list<int> step,
//                            std::list<Point> trajectory1)
//{
//  // robo.reset ();
//  robo.reset();
//
//  Point robo_base = robo.position();
//
//  for ( int i = step[recursive]; i < robo.muscles_.size (); ++i )
//  {
//    auto muscle_i = robo.muscles_[i];
//    if ( !(muscle_i & muscles) )
//    {
//      Trajectory  trajectory;
//
//      /* Попробуем его варьировать по длительности */
//      for ( frames_t last_i : boost::irange (1U, robo.muscleMaxLast (muscle_i)) )
//      {
//        robo.move (muscle_i, last_i, trajectory);
//        store.insert (Record (robo.position(), robo_base, robo.position(),
//                             { muscle_i }, { 0 }, { last_i }, 1U,
//                             trajectory)
//                     );
//
//        std::copy (trajectory.begin (), trajectory.end (), trajectory1.begin ());
//        if ( step.size < recursive + 1 )
//        {}
//
//        testCover (store, robo, muscles & muscle_i, recursive + 1, step, trajectory1);
//
//      } // end for
//    } // end if
//  } // end for
//}

//------------------------------------------------------------------------------
