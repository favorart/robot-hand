#include "RoboPosTourEvo.h"

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

bool RoboPos::TourEvo::runNestedForMuscle(Robo::joint_t, Robo::Control&, Point&)
{
    _robo.reset();
    Point pos = _robo.position(), prev_pos = pos;

    Goal goal(_t);
    double dist = boost_distance(goal.biggest(), pos), prev_dist = dist;
    CDEBUG("init=" << dist << ' ' << pos << ' ' << goal.biggest());
    Robo::Control controls;
    
    std::vector<Robo::frames_t> lasts_step(_robo.musclesCount(), _lasts_step_increment_init);

    bool done = false;
    bool first_move = true;
    std::vector<Robo::Actuator> as(_robo.musclesCount(), { Robo::MInvalid, 0,0 });
    while (!done)
    {
        for (Robo::muscle_t m = 0; m < _robo.musclesCount() && !done; ++m)
        {
            as[m] = { m, 0, min_lasts };
            // // Joint : muscle + opposite (oppo_penalty)
            // //
            // for (Robo::joint_t j = 0; j < robo.jointsCount(); ++j)
            // Robo::muscle_t main = robo.muscleByJoint(j, true);
            // Robo::muscle_t oppo = robo.muscleByJoint(j, false);
            // if (none of increasing do not gives closing to goal)
            do
            {
                //a.lasts += lasts_step[m];
                runNestedMove(controls + as, pos);
                double d = boost_distance(goal.biggest(), pos);
                CDEBUG(d << ' ' << pos << ' ' << goal.biggest() << controls + as);
                if (d >= reached_less(dist)) /* LESS */
                    break;

                if (!first_move)
                {
                    auto dd = boost_distance(pos, prev_pos);
                    /// ??? TODO:: Increment params !!
                    if (d < _step_distance)
                    {
                        lasts_step[m] += _lasts_step_increment;
                        // or -= oppo ?? 
                    }
                    else if (d > _step_distance)
                    {
                        if (lasts_step[m] > _lasts_step_increment)
                            lasts_step[m] -= _lasts_step_increment;
                        // or += oppo ??
                    }
                }
                prev_pos = pos;
                first_move = false;

                dist = d;
                if (dist < _reached_dist)
                {
                    CINFO("Evo: next_stage goal");
                    done = goal.next_stage(_store, side);
                }

                as[m].lasts += lasts_step[m];
            } while (as[m].lasts < _robo.muscleMaxLast(m) && !done);

            if (as[m].lasts >= big_lasts/*robo.muscleMaxLast(m)*/)
            {
                CINFO("Evo: m=" << m << " reset lasts=" << as[m].lasts);
                //a.lasts = min_lasts; // random(min_lasts) ??
                as[m].lasts = Utils::random(min_lasts, big_lasts);
                first_move = true;
            }
            as[m].lasts -= std::min(min_lasts, as[m].lasts); // _lasts_step_increment
            if (as[m].lasts > min_lasts)
                as[m].lasts -= _lasts_step_increment;
            else
                as[m].lasts = 0;
            //if (a.lasts) controls.append(a);
        }

        if (_t.contain(pos))
        {
            CINFO("Evo: recalc goal");
            goal.recalc(_store, side);
        }
        if (dist >= prev_dist)
        {
            /* FAIL */
            CINFO("Evo: FAIL");
            return false;
        }
    }
    CINFO("Evo: DONE");
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

