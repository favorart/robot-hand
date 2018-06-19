#include <stack>
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

#include "WindowData.h" /// !!! RM
std::list<Point> MyWindowData::goals;

namespace RoboPos
{
/// Center of mass of uncovered target points
class Goal
{
    const TargetI &_t;
    const size_t _max_stages{};
    size_t _stage{};
    std::vector<std::pair<Point, size_t>> _goals{};
    TargetI::vec_t::const_iterator _current{};

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
    Goal(const TargetI &t) :
        _t(t), _goals(1, { Point{ 0, 0 }, 0 }), _current(_t.it_coords().first), _max_stages(5)
    {
        auto tp = _t.it_coords();
        _goals.front().first += std::accumulate(tp.first, tp.second, Point{}) / _t.n_coords();
        _goals.front().second = _t.n_coords();
        _stage = 0;
    }
    const Point& biggest() const
    {
        return (_stage == _max_stages) ? (*_current) : (_goals.back().first);
    }
    bool recalc(const RoboMoves::Store &store, double side)
    {
        if (_stage == _max_stages)
            return true;
        _goals.clear();
        if (_stage) _stage--;
        return next_stage(store, side);
    }
    bool next_stage(const RoboMoves::Store &store, double side)
    {
        if (!_goals.empty())
            _goals.pop_back();
        if (_goals.empty())
        {
            if (_stage == _max_stages)
            {
                /* enumerate finished */
                if (_current == _t.it_coords().second)
                {
                    _stage++;
                    _current = _t.it_coords().first;
                    return false;
                }
                /* enumerate all goals from the Target */
                while (covered(*_current, side, store))
                    _current++;
                return true;
            }
            if (_stage > _max_stages)
                return false;

            const auto n_angles = (_stage) ? (4 * _stage) : 1;
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

            _stage++;
        }

        MyWindowData::goals.clear();
        for (auto &goal : _goals)
            MyWindowData::goals.push_back(goal.first);
        return true;
    }
    size_t stage() const { return _stage; }
};
}

frames_t RoboPos::TourEvo::minLasts()
{
    frames_t ll = 0;
    Point pos = _robo.position();
    for (muscle_t m = 0; m < _robo.musclesCount(); ++m)
    {
        frames_t lasts = 1;
        while (boost_distance(pos, _base_pos) <= RoboI::minFrameMove)
        {
            runNestedMove({ Actuator{ m, 0, lasts } }, pos);
            lasts++;
        }
        if (ll < lasts)
            ll = lasts;
    }
    return ll;
}

bool RoboPos::TourEvo::containStore(const Control &controls, joint_t muscles, frames_t frame, frames_t lasts)
{
    Control c = controls;
    for (muscle_t m = 0; m < _robo.musclesCount(); ++m)
        if (muscles & (1 << m))
            c.append({ m, frame, lasts });
    tcout << "-storeC=" << c << std::endl;
    bool exists = (_store.exactRecordByControl(c) != NULL);
    if (exists)
        tcout << "-exists=" << c << std::endl;
    return exists;
}


RoboPos::TourEvo::TourEvo(RoboMoves::Store &store, Robo::RoboI &robo, const TargetI &target) :
    TourI(store, robo), _t(target),
    _reached_dist(0.00001),
    _oppo_penalty(1.),
    _max_ncontrols(10),
    _step_back(4)
{
    tptree root;
    tfstream fin("TourEvo.txt", std::ios::in);
    if (!fin.is_open())
        return;
    pt::read_ini(fin, root);

    _reached_dist = root.get<double>(_T("reached_dist"));
    _oppo_penalty = root.get<double>(_T("oppo_penalty"));
    _prev_dist_add = root.get<double>(_T("prev_dist_add"));
    _max_ncontrols = root.get<frames_t>(_T("max_ncontrols"));
    _step_back = root.get<frames_t>(_T("step_back"));
}

bool RoboPos::TourEvo::stepBack(Control &controls, Control &controls_prev, distance_t curr_best_dist)
{
    //if (controls_prev == controls)
    //{
    //    //CINFO("Evo: regress " << controls);
    //    //tcout << "Evo: regress " << controls << std::endl;
    //    auto last = (controls.size() - 1);
    //    for (auto i = controls.size(); i > 0; --i)
    //    {
    //        if (controls[i - 1].start != controls[last].start)
    //            break;
    //
    //        controls[i - 1].lasts -= std::min(_step_back, controls[i - 1].lasts);
    //
    //        if (controls[i - 1].lasts > too_long)
    //            throw std::logic_error("too_long");
    //
    //        if (controls[i - 1].lasts <= std::max(_step_back, 1u))
    //        {
    //            controls.pop(i - 1);
    //            frames = controls[controls.size() - 1].start;
    //
    //            if (distances.size())
    //                distances.pop();
    //        }
    //        if (distances.size())
    //            curr_best_dist = distances.top();
    //        else
    //            curr_best_dist = boost_distance(goal.biggest(), _base_pos);
    //    }
    //    //CINFO(" become " << controls);
    //    //tcout << "      become " << controls << std::endl;
    //}
    //if (_max_ncontrols <= controls.size())
    //{
    //    //tcout << "Evo: maxsize " << controls << std::endl;
    //    while (_max_ncontrols <= controls.size() && controls.size())
    //    {
    //        auto start = controls[controls.size() - 1].start;
    //        while (start == controls[controls.size() - 1].start && controls.size())
    //            controls.pop_back();
    //
    //        if (distances.size())
    //            distances.pop();
    //        if (distances.size())
    //            curr_best_dist = distances.top();
    //        else
    //            curr_best_dist = boost_distance(goal.biggest(), _base_pos);
    //        frames = controls[controls.size() - 1].start;
    //    }
    //    if (!controls.size())
    //    {
    //        frames = 0;
    //        curr_best_dist = boost_distance(goal.biggest(), _base_pos);
    //    }
    //    //tcout << "      become " << controls << std::endl;
    //    tcout << "Evo: maxsize " << controls << std::endl;
    //}
    //
    //if (!controls.size())
    //{
    //    tcout << "--from scratch " << std::endl;
    //}
    return false;
}


bool RoboPos::TourEvo::runNestedForMuscle(joint_t, Control&, Point&)
{
    _robo.reset();
    _base_pos = _robo.position();
    
    Goal goal(_t);

    distance_t the_best_dist = boost_distance(goal.biggest(), _base_pos), curr_best_dist = the_best_dist;
    Control controls, controls_prev;
    std::stack<distance_t> distances{};

    const muscle_t n_muscles = _robo.musclesCount();
    const joint_t n_acts = joint_t(std::pow(2, n_muscles));

    const frames_t lasts_max = std::min(musclesMaxLasts(_robo), TourI::too_long);
    const frames_t lasts_step = 1;
    const frames_t lasts_init = minLasts();
    tcout << lasts_init << std::endl << std::endl;
    frames_t frames = 0;

    bool one_watched = false;
    bool done = false;
    //CINFO("Evo: next_stage goal=" << goal.biggest());
    tcout << "Evo: next_stage goal=" << goal.biggest() << std::endl;
    while (!done)
    {
        joint_t best_acts = 0;
        distance_t best_d = curr_best_dist;
        frames_t best_lasts = 0;

        Point prev_pos{};
        distance_t prev_dist = curr_best_dist;

        for (joint_t acts = 1; acts < (n_acts - 1) && !done; ++acts)
        {
            Point pos{};
            if (containStore(controls, acts, frames, lasts_init))
            {
                tcout << "-exists=" << controls << std::endl;
                continue;
            }
            one_watched = true;
            // ============
            /* repeat trajectory after reset -- pre-move */
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
                distance_t d = boost_distance(goal.biggest(), pos);
                // ============
                if (!first_move)
                {
                    if (fabs(boost_distance(pos, prev_pos)) <= RoboI::minFrameMove)
                    {
                        //tcout << "constPos last=" << lasts << std::endl;
                        runNestedStop(acts, true);
                        runNestedReset(controls, pos);
                        //lasts = lasts_init;
                        break;
                    }
                    if (d > (prev_dist + _prev_dist_add))
                    {
                        /* This muscle is FAIL now */
                        Control c(bitset_t{ acts }, frames, lasts);
                        //tcout << "muscle FAIL d=" << prev_dist << " { " << c << " }" << " pos=" << pos << std::endl;
                        tcout << " FAIL pos=" << pos << " " << controls + c << std::endl;
                        runNestedStop(acts, true);
                        runNestedReset(controls, pos);
                        //if (lasts <= 2 * lasts_init) lasts = lasts_init;
                        break;
                    }
                }

                prev_dist = d;
                prev_pos = pos;
                first_move = false;

                if (prev_dist < _reached_dist)
                {
                    //CINFO("Evo: next_stage goal=" << goal.biggest());
                    tcout << "Evo: next_stage goal=" << goal.biggest() << std::endl;
                    done = !goal.next_stage(_store, side);

                    for (muscle_t m = 0; m < n_muscles; ++m)
                        if (acts & (1 << m))
                            controls.append({ m, frames, lasts });

                    runNestedStop(acts, true);
                    runNestedReset(controls, pos);
                    continue;
                }

                // ============
                for (frames_t l = 0; l < lasts_step; ++l)
                    _robo.step();
                // ============
                pos = _robo.position();
            }

            if (lasts >= lasts_max)
            {
                Control c(bitset_t{ acts }, frames, lasts_max);
                tcout << " d=" << prev_dist << " { " << c << " }" 
                    << " pos=" << pos 
                    << " (lasts == lasts_max)"
                    << std::endl;
                runNestedStop(acts, false);
                runNestedReset(controls, Point{});
            }

            if (lasts > lasts_init && best_d > prev_dist)
            {
                tcout << " d=" << prev_dist << std::endl;
                best_acts = acts;
                best_d = prev_dist;
                best_lasts = lasts;
            }
        }

        if (!one_watched)
        {
            /* FAIL */
            //CINFO("Evo: FAIL");
            tcout << "Evo: FAIL " << the_best_dist << " stage: " << goal.stage() << std::endl;
            return false;
        }

        if (best_acts) /* if best_acts selected */
        {
            {
                Control c = controls;
                for (muscle_t m = 0; m < n_muscles; ++m)
                    if (best_acts & (1 << m))
                        c.append({ m, frames, lasts_init });

                _robo.move(c);
                auto hit = _robo.position();
                RoboMoves::Record rec{ hit, _base_pos, hit, c, _robo.trajectory() };
                _store.insert(rec);
                /* for exact by controls */
                tcout << "---prev=" << c << std::endl;

                _robo.reset();
            }

            for (muscle_t m = 0; m < n_muscles; ++m)
                if (best_acts & (1 << m))
                    controls.append({ m, frames, best_lasts });

            frames += best_lasts;
            distances.push(best_d);
            tcout << "---acts=" << controls << std::endl;
        }

        if (_t.contain(prev_pos))
        {
            //CINFO("Evo: recalc goal");
            //tcout << "Evo: recalc goal" << std::endl;
            goal.recalc(_store, side);
        }

        if (curr_best_dist > best_d)
            curr_best_dist = best_d;

        if (the_best_dist > curr_best_dist)
            the_best_dist = curr_best_dist;

        ///stepBack(controls, controls_prev, curr_best_dist);
        // ==========================================================================
        if (controls_prev == controls)
        {
            //CINFO("Evo: regress " << controls);
            //tcout << "Evo: regress " << controls << std::endl;
            auto last = (controls.size() - 1);
            auto start = controls[last].start;

            bool popped = false;
            for (auto i = controls.size(); i > 0; --i)
            {
                if (controls[i - 1].start != start)
                    break;

                controls[i - 1].lasts -= std::min(_step_back, controls[i - 1].lasts);

                if (controls[i - 1].lasts > too_long)
                    throw std::logic_error("too_long");

                if (controls[i - 1].lasts <= std::max(_step_back, 1u))
                {
                    controls.pop(i - 1);
                    popped = true;
                }
            }
            frames = controls[controls.size() - 1].start;

            if (distances.size() && popped)
                distances.pop();
            curr_best_dist = (distances.size()) ? distances.top() : boost_distance(goal.biggest(), _base_pos);

            //CINFO(" become " << controls);
            ///tcout << "      become " << controls << std::endl;
            ///if (controls.size() == 2 && controls[0].start == 0 && controls[1].start == 0 && controls[0].lasts < 500)
            ///{
            ///    tcout << "this " << controls << std::endl;
            ///}
        }
        if (_max_ncontrols <= controls.size())
        {
            //tcout << "Evo: maxsize " << controls << std::endl;
            while (_max_ncontrols <= controls.size() && controls.size())
            {
                auto start = controls[controls.size() - 1].start;
                while (start == controls[controls.size() - 1].start && controls.size())
                    controls.pop_back();

                if (distances.size())
                    distances.pop();
                if (distances.size())
                    curr_best_dist = distances.top();
                else
                    curr_best_dist = boost_distance(goal.biggest(), _base_pos);
                frames = controls[controls.size() - 1].start;
            }
            if (!controls.size())
            {
                frames = 0;
                curr_best_dist = boost_distance(goal.biggest(), _base_pos);
            }
            //tcout << "      become " << controls << std::endl;
            tcout << "Evo: maxsize " << controls << std::endl;
        }

        if (!controls.size())
        {
            tcout << "--from scratch " << std::endl;
            return false;
        }
        // ==========================================================================
        controls_prev = controls;
    }
    //CINFO("Evo: DONE");
    tcout << "Evo: DONE" << std::endl;
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
    boost::this_thread::interruption_point();
    //----------------------------------------------
    //const auto *pRec = _store.exactRecordByControl(controls);
    //if (pRec)
    //{
    //    robo_hit = pRec->hit;
    //    return true;
    //}
    //----------------------------------------------
    _robo.step(controls /*+ _breakings_controls*/);
    robo_hit = _robo.position();
    //----------------------------------------------
    return true;
}

bool RoboPos::TourEvo::runNestedStep(IN const Robo::bitset_t &muscles, IN Robo::frames_t lasts, OUT Point &robo_hit)
{
    boost::this_thread::interruption_point();
    //----------------------------------------------
    // ??? exactRecordByControl
    //----------------------------------------------
    _robo.step(muscles, lasts); // ??? breakings
    robo_hit = _robo.position();
    //----------------------------------------------
    return true;
}


RoboPos::TourEvoSteps::TourEvoSteps(RoboMoves::Store &store, Robo::RoboI &robo, const TargetI &target) :
    TourEvo(store, robo, target)
    //,
    //_reached_dist(0.00001),
    //_oppo_penalty(1.),
    //_max_ncontrols(10),
    //_step_back(4)
{
    tptree root;
    tfstream fin("TourEvo.txt", std::ios::in);
    if (!fin.is_open())
        return;
    pt::read_ini(fin, root);

    _reached_dist = root.get<double>(_T("reached_dist"));
    _oppo_penalty = root.get<double>(_T("oppo_penalty"));
    _prev_dist_add = root.get<double>(_T("prev_dist_add"));
    _max_ncontrols = root.get<frames_t>(_T("max_ncontrols"));
    _step_back = root.get<frames_t>(_T("step_back"));
}

bool RoboPos::TourEvoSteps::runNestedForStep(IN const Robo::RoboI::bitwise &muscles, OUT Point &hit)
{
    boost::this_thread::interruption_point();
    //----------------------------------------------
    _robo.step(muscles);
    hit = _robo.position();
    //----------------------------------------------
    return true;
}

bool RoboPos::TourEvoSteps::runNestedForMuscle(joint_t joint, Control &controls, Point &robo_hit)
{
    _robo.reset();
    _base_pos = _robo.position();

    Goal goal(_t);

    const frames_t lasts_max = std::min(musclesMaxLasts(_robo), TourI::too_long);
    const frames_t lasts_step = 1;

    const muscle_t n_muscles = _robo.musclesCount();
    const joint_t  n_acts = joint_t(std::pow(2, n_muscles));

    //std::vector<muscle_t> ctrls;

    Point best_hit{};
    joint_t best_acts{};
    distance_t best_dist = boost_distance(goal.biggest(), _base_pos);
    
    std::stack<distance_t> distances{};

    bool done = false;
    tcout << "Evo: next_stage goal=" << goal.biggest() << std::endl;
    for (frames_t lasts = 0; (lasts < lasts_max) && (!done); lasts += lasts_step)
    {
        bool Act = false;

        for (muscle_t acts = 1; acts < (n_acts - 1) && !done; ++acts)
        {
            if (containStore(controls, acts, lasts, 1))
                continue;

            if (controls.size()) /* pre-move */
                _robo.move(controls, lasts);
            //_robo.move(ctrls);

            Act = true;
            Point hit{};
            // ============
            runNestedForStep(RoboI::bitwise{ acts }, hit);
            // ============
            distance_t goal_dist = boost_distance(hit, goal.biggest());
            if (goal_dist < best_dist)
            {
                best_dist = goal_dist;
                best_acts = acts;
                best_hit = hit;
                //----------------------------------------------
                tcout << _robo.frame() << " |";
                for (muscle_t m = 0; m < _robo.musclesCount(); ++m)
                    tcout << " " << _robo.muscleStatus(m) << "-" << _robo.lastsStatusT(m) << _robo.lastsStatus(m);
                tcout << std::endl;
            }

            runNestedStop(acts, true);
            runNestedReset(controls, hit);
        } // end for acts

        if (!best_acts)
        {
            if (!Act)
            {
                tcout << "Evo: FAIL " << best_dist << std::endl;
                return false;
            }

            //ctrls.pop_back();
            {
                //tcout << "Evo: regress " << controls << std::endl;
                auto last = (controls.size() - 1);
                auto start = controls[last].start;
                bool popped = false;
                for (auto i = controls.size(); i > 0; --i)
                {
                    if (controls[i - 1].start != start)
                        break;

                    controls[i - 1].lasts -= std::min(lasts_step, controls[i - 1].lasts);
                    if (!controls[i - 1].lasts)
                    {
                        controls.pop(i - 1);
                        popped = true;
                    }
                    else if (controls[i - 1].lasts > too_long)
                        throw std::logic_error("too_long");
                }
                lasts = (controls[controls.size() - 1].start + controls[controls.size() - 1].lasts);
                if (distances.size() && popped)
                    distances.pop();
                best_dist = (distances.size()) ? distances.top() : boost_distance(goal.biggest(), _base_pos);
                //tcout << "      become " << controls << std::endl;
            }
        }

        //ctrls.push_back(best_acts);
        {
            /* if best_acts selected */
            const auto last_start = controls[controls.size() - 1].start;
            for (size_t i = controls.size(); i > 0; --i)
            {
                auto m = controls[i - 1].muscle;
                if (controls[i - 1].start == last_start && best_acts & (1 << m))
                {
                    controls[i - 1].lasts++;
                    best_acts ^= (1 << m);
                }
            }
            if (best_acts)
            {
                for (muscle_t m = 0; m < n_muscles; ++m)
                    if (best_acts & (1 << m))
                        controls.append({ m, lasts, 1 });
            }
            tcout << "---acts=" << controls << std::endl;
        }
        
        if (_t.contain(best_hit))
        {
            //tcout << "Evo: recalc goal" << std::endl;
            goal.recalc(_store, side);
        }

        if (best_dist < _reached_dist)
        {
            /* it has walked throught a goal, want to stop at a goal */
            compansateOverHit(controls, goal.biggest());

            tcout << "Evo: next_stage goal=" << goal.biggest() << std::endl;
            done = !goal.next_stage(_store, side);
        }
    } // end for lasts
    
    if (!done)
    {
        tcout << "Evo: FAIL " << std::endl;
        tcout << " d=" << best_dist << " { " << controls << " }"
            << " hit=" << best_hit
            << " (lasts == lasts_max)"
            << std::endl;
        return false;
    }

    tcout << "Evo: DONE" << std::endl;
    return true;
}

bool RoboPos::TourEvoSteps::compansateOverHit(Robo::Control &controls, const Point &goal)
{
    const frames_t lasts = _robo.getVisitedRarity();

    _robo.move(controls);
    Point hit = _robo.position();
    auto &visited = _robo.trajectory();
    runNestedReset(controls, Point{});

    while (boost_distance(hit, goal) > _reached_dist)
    {
        distance_t best_path = boost_distance(hit, goal);
        size_t best_n = 0, n = 0;
        auto best_it = visited.end();

        const Point goal_dir = (hit - goal);
        for (auto curr = visited.begin(); curr != visited.end(); ++curr, ++n)
        {
            auto next = std::next(curr);
            if (next == visited.end())
                break;
            Point curr_dir =( *next - *curr);

            distance_t path = boost_distance(curr_dir, goal_dir);
            if (best_path > path)
            {
                best_path = path;
                best_it = curr;
                best_n = n;
            }
        }

        if (best_it == visited.end())
            return false;

        const size_t prev_to_last = (controls.size() - 2);
        const frames_t start = (best_n == prev_to_last) ? (best_n) : (best_n * _robo.getVisitedRarity());
        for (size_t i = 0; i < controls.size(); )
        {
            auto &c = controls[i];
            if (c.start >= start)
            {
                if ((c.start + c.lasts) <= (start + lasts))
                    controls.pop(i);
                else
                {
                    c.start = (start + lasts + 1);
                    ++i;
                }
            }
            else // if (c.start < start)
            {
                if ((c.start + c.lasts) <= (start + lasts))
                    c.lasts = (start - c.start);
                else
                {
                    controls.append({ c.muscle, (start + lasts + 1), (c.lasts - lasts) });
                    c.lasts = (start - c.start);
                }
                ++i;
            }
        }

        _robo.move(controls);
        hit = _robo.position();
        visited = _robo.trajectory();
        runNestedReset(controls, Point{});
    }

    {
        /* check-out */
        _robo.move(controls);
        hit = _robo.position();
        visited = _robo.trajectory();
        _robo.reset();
        if (boost_distance(hit, goal) > _reached_dist)
        {
            tcout << "overhit fail " << boost_distance(hit, goal) << std::endl;
        }
    }

    controls.clear(); // ???
    return true;
}


class Mutation
{
    static void longer(const Robo::Actuator&) {}
    static void shorter(const Robo::Actuator&) {}
    static void erlier(const Robo::Actuator&) {}
    static void later(const Robo::Actuator&) {}
    static void replace_muscle(const Robo::Actuator&, Robo::muscle_t n_muscles) {}
    static void split(const Robo::Control&) {}
    static void join(const Robo::Control&) {}
};

