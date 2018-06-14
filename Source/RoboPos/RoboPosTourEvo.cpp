#include "RoboPosTourEvo.h"

using namespace Robo;
using namespace RoboPos;

namespace {
//bool areClockwise(const Point &v1, const Point &v2)
//{ return (-v1.x*v2.y + v1.y*v2.x) > 0; }
//
//bool withinRadius(const Point &v, double radiusSquared)
//{ return (v.x*v.x + v.y*v.y) <= radiusSquared; }
//
//bool insideSector(const Point &pt, const Point &beg, sectorStart, end, radiusSquared)
//{
//    Point rel = { pt.x - center.x, pt.y - center.y };
//    return !areClockwise(sectorStart, relPoint) 
//         && areClockwise(sectorEnd, relPoint)
//         && withinRadius(relPoint, radiusSquared);
//}
//
//bool insideAngle(const Point &pt, const Point &center, const Point &beg, const Point &end)
//{
//    Point rel = { pt.x - center.x, pt.y - center.y };
//    return !areClockwise(beg, rel) && areClockwise(end, rel);
//}
}

namespace RoboPos
{
/// Center of mass of uncovered target points
class Goal
{
    const size_t max_stages = 3;
    size_t stage = 0;
    const TargetI &_t;
    std::vector<std::pair<Point, size_t>> _goals{};

    bool covered(const Point &aim, double side, const RoboMoves::Store &store)
    {
        auto cpp = store.getClosestPoint(aim, side);
        return (cpp.first && boost_distance(cpp.second.hit, aim) < side);
    }
    /// index of a sector of circle the point is belonged
    int belongs(const Point &aim, const Point &center, int n_angles)
    {
        int res = n_angles;
        Point base = { center.x + 1, center.y };
        const auto ag = M_PI / n_angles;

        double a = angle_radians(base, center, aim);
        for (int i = 0; i < n_angles; ++i)
            if (i * ag >= a && a < (i + 1) * ag)
            {
                res = i;
                break;
            }
        return res;
    }

public:
    Goal(const TargetI &t) : _t(t), _goals(1, { Point{ 0, 0 }, 0 })
    {
        auto tp = _t.it_coords();
        _goals.front().first += std::accumulate(tp.first, tp.second, Point{}) / _t.n_coords();
        _goals.front().second = _t.n_coords();
        stage = 1;
    }
    const Point& biggest() const { return _goals.back().first; }
    bool recalc(const RoboMoves::Store &store, double side)
    {
        _goals.clear();
        --stage;
        return next_stage(store, side);
    }
    bool next_stage(const RoboMoves::Store &store, double side)
    {
        if (!_goals.empty())
            _goals.pop_back();
        if (_goals.empty())
        {
            if (stage >= max_stages)
                // enumerate all goals from target
                return false;

            const auto n_angles = (stage) ? (4 * stage) : 1;
            _goals.resize(n_angles);

            auto tp = _t.it_coords();
            for (auto it = tp.first; it != tp.second; ++it)
                if (!covered(*it, side, store))
                {
                    auto i = (n_angles > 1) ? belongs(*it, _t.center(), n_angles) : 0;
                    _goals[i].first += *it;
                    _goals[i].second++;
                }

            for (size_t n = 0; n < n_angles; ++n)
                _goals[n].first /= _goals[n].second;

            std::sort(_goals.begin(), _goals.end(),
                      [](const auto &a, const auto &b) { return a.second < b.second; });

            ++stage;
        }
        return true;
    }
};
}

Robo::frames_t RoboPos::TourEvo::minLasts()
{
    frames_t ll = 0;
    Point pos = _robo.position(), prev_pos = pos;
    for (muscle_t m = 0; m < _robo.musclesCount(); ++m)
    {
        frames_t lasts = 1;
        while (fabs(boost_distance(pos, prev_pos)) <= RoboI::minFrameMove)
        {
            runNestedMove({ { m, 0, lasts } }, pos);
            lasts++;
        }
        if (ll < lasts)
            ll = lasts;
    }
    return ll;
}

bool RoboPos::TourEvo::runNestedForMuscle(joint_t, Control&, Point&)
{
    _robo.reset();
    _base_pos = _robo.position();
    Goal goal(_t);
    distance_t best_dist = boost_distance(goal.biggest(), _robo.position());
    
    //std::vector<frames_t> lasts_step(_robo.musclesCount(), _lasts_step_increment_init);
    //std::vector<Actuator> as(_robo.musclesCount(), { MInvalid, 0,0 });
    Control controls, controls_prev;

    auto lasts_max = musclesMaxLasts(_robo);
    lasts_max = (lasts_max > TourI::too_long) ? TourI::too_long : lasts_max;

    const muscle_t n_muscles = _robo.musclesCount();
    const joint_t n_acts = joint_t(std::pow(2, n_muscles));

    frames_t frames = 0;
    const frames_t lasts_step = 1;
    const frames_t lasts_init = minLasts();
    tcout << lasts_init << std::endl << std::endl;

    bool done = false;
    CINFO("Evo: next_stage goal=" << goal.biggest());
    while (!done)
    {
        joint_t best_acts = 0;
        //Control best_c;
        distance_t best_d = best_dist;
        frames_t best_lasts = 0;

        Point prev_pos{};
        distance_t prev_dist = best_dist;
        Trajectory trajectory{};

        for (joint_t acts = 1; acts < (n_acts - 1) && !done; ++acts)
        {
            Point pos{};
            //Control c{};
            //for (muscle_t i = 0; i < n_acts; ++i)
            //    if (acts & (1 << i))
            //        c.append({ i, frames, lasts_init });
            {
                Control c = controls;
                for (muscle_t m = 0; m < n_acts; ++m)
                    if (acts & (1 << m))
                        c.append({ m, frames, lasts_init });
                bool exists = (_store.exactRecordByControl(c) != NULL);
                if (exists)
                    continue;
            }

            // ============
            // repeat trajectory after reset -- pre-move
            if (frames)
                _robo.move(controls, frames);
            // ============
            runNestedStep(Robo::bitset_t{ acts }, lasts_max, pos);
            // ============
            _robo.move(lasts_init);
            // ============
            prev_pos = pos = _robo.position();

            bool first_move = true;
            frames_t lasts = 0;
            for (lasts = lasts_init; (lasts < lasts_max) && (!done); lasts += lasts_step)
            {
                // ============
                //runNestedMove(controls + c, pos);
                // ============

                distance_t d = boost_distance(goal.biggest(), pos);
                //CDEBUG(controls + as << " d=" << d <<" | " << goal.biggest() << " <<< " << pos);

                if (!first_move)
                {
                    if (fabs(boost_distance(pos, prev_pos)) <= RoboI::minFrameMove)
                    {
                        //tcout << "constPos last=" << lasts << std::endl;
                        runNestedStop(/*frames + lasts,*/ acts, true);
                        runNestedReset(controls, pos);
                        break;
                    }
                    if (d > (prev_dist + RoboI::minFrameMove))
                    {
                        /* This muscle is FAIL now */
                        Control c(bitset_t{ acts }, frames, lasts);
                        //tcout << "muscle FAIL d=" << prev_dist << " { " << c << " }" << " pos=" << pos << std::endl;
                        tcout << " d=" << prev_dist << std::endl;
                        runNestedStop(/*frames + lasts,*/ acts, true);
                        runNestedReset(controls, pos);
                        break;
                    }
                }
                prev_dist = d;
                prev_pos = pos;
                first_move = false;

                if (prev_dist < _reached_dist)
                {
                    CINFO("Evo: next_stage goal=" << goal.biggest());
                    done = goal.next_stage(_store, side);

                    for (muscle_t m = 0; m < n_acts; ++m)
                        if (acts & (1 << m))
                            controls.append({ m, frames, lasts });

                    runNestedStop(/*frames + lasts,*/ acts, true);
                    runNestedReset(controls, pos);
                    //controls += c;
                    //c.clear();
                    //for (muscle_t i = 0; i < n_acts; ++i)
                    //    if (acts & (1 << i))
                    //        c.append({ i, frames, lasts_init });
                    continue;
                }

                for (frames_t l = 0; l < lasts_step; ++l)
                    _robo.step(/*frames + lasts + l, trajectory*/);
                pos = _robo.position();
                //for (auto &a : c) a.lasts += lasts_step;
            }
            if (lasts >= lasts_max)
            {
                Control c(bitset_t{ acts }, frames, lasts_max);
                tcout << " d=" << prev_dist << " { " << c << " }" << " pos=" << pos << " (lasts == lasts_max)" << std::endl;
                runNestedStop(/*(frames + lasts_max - lasts_step),*/ acts, false);
            }

            if (lasts > lasts_init /*10*/ && best_d > prev_dist)
            {
                best_acts = acts;
                //best_c = c;
                best_d = prev_dist;
                best_lasts = lasts;
            }
            //if (max_lasts < lasts) max_lasts = lasts;
        }

        for (muscle_t m = 0; m < n_acts; ++m)
            if (best_acts & (1 << m))
                controls.append({ m, frames, best_lasts });
        //controls += best_c;
        frames += best_lasts;
        tcout << "---acts=" << controls << std::endl << std::endl;

        if (_t.contain(prev_pos))
        {
            CINFO("Evo: recalc goal");
            goal.recalc(_store, side);
        }

        if (best_d >= best_dist)
        {
            CINFO("Evo: regress " << controls);
            for (auto i = controls.size(); i > 0; --i)
            {
                if (controls[i - 1].start != controls[controls.size() - 1].start)
                    break;

                controls[i - 1].lasts -= lasts_step;
                if (controls[i - 1].lasts <= lasts_init)
                    controls[i - 1] = { MInvalid, 0, 0 };
            }
            CINFO(" become " << controls);
        }
        else
            best_dist = best_d;

        if (controls_prev == controls)
        {
            /* FAIL */
            CINFO("Evo: FAIL");
            return false;
        }
        controls_prev = controls;
    }
    CINFO("Evo: DONE");
    return true;
}

bool RoboPos::TourEvo::runNestedReset(IN const Control &controls, IN OUT Point &hit)
{
    bool exist = (_store.exactRecordByControl(controls) != NULL);
    if (!exist)
    {
        _complexity++;
        RoboMoves::Record rec{ hit, _base_pos, hit, controls, _robo.trajectory() };
        _store.insert(rec);
    }
    _robo.reset();
    hit = { 0,0 };

    return exist;
}

bool RoboPos::TourEvo::runNestedStop(IN const Control &controls, IN bool stop)
{
    if (stop)
        _robo.move(controls /*+ _breakings_controls*/);
    else
        _robo.move();
    return stop;
}

bool RoboPos::TourEvo::runNestedStop(IN const Robo::bitset_t &muscles, IN bool stop)
{
    if (stop)
        _robo.move(muscles, 1); // ??? breakings
    else
        _robo.move();
    return stop;
}

bool RoboPos::TourEvo::runNestedStep(IN const Control &controls, OUT Point &robo_hit)
{
    _robo.step(controls /*+ _breakings_controls*/);
    robo_hit = _robo.position();
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return true;
}


bool RoboPos::TourEvo::runNestedStep(IN const Robo::bitset_t &muscles, IN Robo::frames_t lasts, OUT Point &robo_hit)
{
    _robo.step(muscles, lasts); // ??? breakings
    robo_hit = _robo.position();
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    return true;
}


//namespace Robo
//{
//bool runNestedForMuscle(IN joint_t joint, IN Control &controls, OUT Point &robo_pos_high)
//{
//    CDEBUG("runNested " << joint/* << " last_i " << last_i*/);
//    --_max_nested;
//    // стартруем разными сочленениями последовательно
//    //const frames_t start_i = controls.size() ? (controls[-1].lasts + controls[-1].start) : 0;
//    // стартруем разными сочленениями одновременно
//    const frames_t start_i = 0;
//    //------------------------------------------
//    Point curr_pos = _base_pos, prev_pos = _base_pos;
//    //------------------------------------------
//    // заметание пространства проходит рядами,
//    // сдвигаясь всё дальше от начальной точки
//    int high = 0; // число положений захвата робота в вышестоящем ряду
//    Point max_robo_pos_high{ 0., 0. }; // сумма положений робота в вышестоящем ряду
//
//    Actuator control_i;
//    //------------------------------------------
//    for (auto muscle_i : { RoboI::muscleByJoint(joint, true),
//         RoboI::muscleByJoint(joint, false) })
//    {
//        control_i.muscle = muscle_i;
//        //------------------------------------------
//        auto &board = _borders[muscle_i];
//        if (board.min_lasts == 0 && board.min_lasts >= board.max_lasts)
//        {
//            CWARN("Empty borders of muscle=" << muscle_i);
//            return false;
//        }
//
//
//        //------------------------------------------
//        // адаптивный шаг длительности
//        frames_t lasts_step = _lasts_step_increment_init;
//        //frames_t lasts_i_max = _robo.muscleMaxLast(muscle_i);
//        //------------------------------------------
//        for (frames_t last_i = board.min_lasts; last_i < board.max_lasts/*lasts_i_max*/; last_i += lasts_step)
//        {
//            control_i.lasts = last_i;
//            //------------------------------------------
//            if (_max_nested)
//            {
//                auto nj = _next_joint(_robo.jointsCount(), joint, false);
//                //===============================================================
//                if (!runNestedForMuscle(nj, controls + control_i, curr_pos))
//                    return false;
//                //===============================================================
//            }
//            else
//            {
//                exactsBreakings(joint, controls + control_i);
//                //===============================================================
//                runNestedMove(controls + control_i, curr_pos);
//                //===============================================================
//            }
//            //------------------------------------------
//            if (high)
//            {
//                double d = boost_distance(prev_pos, curr_pos);
//                //------------------------------------------
//                if (d > _step_distance)
//                {
//                    // адаптивный шаг изменения длительности
//                    if (lasts_step > _lasts_step_increment)
//                        lasts_step -= _lasts_step_increment;
//                    else if (_lasts_step_increment > 1 && lasts_step > _lasts_step_increment / 2)
//                        lasts_step -= _lasts_step_increment / 2;
//                    else if (_b_braking) // ?? && _lasts_step_increment == 1)
//                    {
//                        // если нельзя сохранить одинаковый промежуток уменьшением длительности
//                        // подключаем торможения противоположным двигателем
//                        appendBreakings(joint, control_i);
//                    }
//                }
//                else if (d < _step_distance)
//                {
//                    if (_b_braking && _breakings_controls_actives > 0 &&
//                        _breakings_controls[joint].lasts > 0 /* !!! */)
//                    {
//                        // сначала по возможности отключаем торможения
//                        removeBreakings(joint);
//                    }
//                    else
//                    {
//                        // затем увеличиваем длительность
//                        lasts_step += _lasts_step_increment;
//                    }
//                }
//            }
//            //------------------------------------------
//            {
//                // сохраняем полученное положение робота
//                // в сумму для получения среднего, которым пользуется
//                // функция выше по реккурсии
//                max_robo_pos_high.x += curr_pos.x;
//                max_robo_pos_high.y += curr_pos.y;
//                ++high;
//                //CDEBUG("curr_pos " << curr_pos);
//            }
//            //------------------------------------------
//            prev_pos = curr_pos;
//            //------------------------------------------
//        } // end for (lasts)
//
//          //-----------------------------
//        cleanBreakings(joint);
//        //-----------------------------
//    } // end for (muscle)
//      //------------------------------------------
//    if (high)
//    {
//        robo_pos_high = Point{ max_robo_pos_high.x / high,
//            max_robo_pos_high.y / high };
//    }
//    //------------------------------------------
//    ++_max_nested;
//    return true;
//}
//}

