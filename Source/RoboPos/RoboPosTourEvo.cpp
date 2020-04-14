#include "RoboPosTourEvo.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;

#ifdef DEBUG_SHOW_CONTRADICTION
#include "WindowData.h"
std::list<Point> MyWindowData::goals;
#endif

namespace RoboPos
{
/// Center of mass of uncovered target points
class Goal
{
    struct AvgGoals
    {
        Point avg_goals{};
        unsigned n_goals = 0;
        AvgGoals() = default;
        AvgGoals(const Point &p, unsigned n) :
            avg_goals(p), n_goals(n)
        {}
    };
    const double _side{ 0.01 };
    const double _reached{};
    const unsigned _max_stages{ 5 };
    const unsigned _angles_factor{ 4 };
    const TargetI &_target;
    unsigned _stage{ 0 };
    std::vector<AvgGoals> _goals{};
    TargetI::vec_t::const_iterator _current{};

    size_t belongs(const Point &aim, const int n_angles) const
    {
        auto OA = (aim - _target.center());
        double a = atan2(OA.y, OA.x); /* [-pi, +pi] */
        a = (a + M_PI) / (2 * M_PI / n_angles);
        return static_cast<size_t>(floor(a));
    }
    bool covered(const Point &aim, const RoboMoves::Store &store) const
    {
        auto cpp = store.getClosestPoint(aim, _side);
        return (cpp.first && boost_distance(cpp.second.hit, aim) <= _reached);
    }
    void stage0()
    {
        auto its_pair = _target.it_coords();
        auto n_coords = static_cast<unsigned>(_target.n_coords());
        _goals.emplace_back(std::accumulate(its_pair.first, its_pair.second, Point{}) / n_coords, n_coords);
    }

public:
    Goal(const TargetI &t, double reached) :
        _reached(reached),
        _target(t),
        _current(t.it_coords().first)
    {
        _goals.reserve(_max_stages * _angles_factor);
    }
    const Point& biggest() const
    {
        return (_stage == _max_stages) ? (*_current) : (_goals.back().avg_goals);
    }
    bool recalc(const RoboMoves::Store &store)
    {
        _goals.clear();
        if (_stage > 0)
        {
            if (_stage < _max_stages)
                --_stage;
            return next_stage(store);
        }
        else
        {
            stage0();
            if (covered(_goals.front().avg_goals, store))
            {
                _goals.pop_back();
                return next_stage(store);
            }
            return true;
        }
    }
    bool next_stage(const RoboMoves::Store &store)
    {
        if (!_goals.empty())
            _goals.pop_back();

        while (_goals.empty())
        {
            if (_stage > _max_stages)
                return false;
            else if (_stage == _max_stages)
            {
                 /* enumerate all goals from the Target */
                while (_current != _target.it_coords().second)
                {
                    if (!covered(*_current, store))
                        return true;
                    _current++;
                }
                 /* enumerate finished */
                ++_stage;
                _current = _target.it_coords().first;
                return false;
            }
            else
            {
                ++_stage;
                const auto n_angles = (_angles_factor * _stage);
                _goals.resize(n_angles);

                const auto its_pair = _target.it_coords();
                for (auto it = its_pair.first; it != its_pair.second; ++it)
                    if (!covered(*it, store))
                    {
                        size_t i = belongs(*it, n_angles);
                        _goals[i].avg_goals += *it;
                        _goals[i].n_goals++;
                    }

                for (auto it = _goals.begin(); it != _goals.end();)
                {
                    it->avg_goals /= it->n_goals;
                    //auto cpp = store.getClosestPoint(it->avg_goals, _side);
                    //double d = (cpp.first) ? boost_distance(cpp.second.hit, it->avg_goals) : _reached + 1;
                    //auto hit = ((cpp.first) ? cpp.second.hit : Point{ 0,0 });
                    //tcout << cpp.first << " avg=" << it->avg_goals << " hit=" << hit << ' ' << d << std::endl;
                    //if ((it->n_goals > 0) && (d > _reached))
                    if ((it->n_goals > 0) && !covered(it->avg_goals, store))
                        ++it;
                    else
                        it = _goals.erase(it);
                }

                auto pred_n_uncovered = [](const auto &a, const auto &b) { return a.n_goals > b.n_goals; };
                std::sort(_goals.begin(), _goals.end(), pred_n_uncovered);
            }
        }
        return true;
    }

#ifdef DEBUG_SHOW_CONTRADICTION
    size_t size() const { return _goals.size(); }
    void show() const
    {
        MyWindowData::goals.clear();
        for (auto &goal : _goals)
            MyWindowData::goals.push_back(goal.avg_goals);
    }
#endif
    size_t stage() const { return _stage; }
    /// index of a sector of the target the point is belonged
    static size_t point_belongs_to_sector(const Point &aim, const Point &center, const int n_angles)
    {
        Point base = { center.x + 1., center.y };
        double a = angle_radians(base, center, aim); /* [-pi, +pi] */
        a = (a + M_PI) / (2 * M_PI / n_angles);
        return static_cast<size_t>(floor(a));
    }
};
}

frames_t RoboPos::TourEvo::minLasts()
{
    frames_t ll = 0;
    Point pos = _robo->position();
    for (muscle_t m = 0; m < _robo->musclesCount(); ++m)
    {
        frames_t lasts = 1;
        while (boost_distance(pos, _base_pos.spec()) <= RoboI::minFrameMove)
        {
            runNestedMove({ Actuator{ m, 0, lasts } }, pos);
            lasts++;
        }
        if (ll < lasts)
            ll = lasts;
    }
    return ll;
}

const RoboMoves::Record* RoboPos::TourEvo::appendMarker(const Control &controls, const Control &new_controls, const Point &aim)
{
    if (!new_controls.size() && !controls.size()) return NULL;
    //================
    auto pReq = _store->exactRecordByControl(controls);
    //if (!pReq && controls.size())
    //    //throw std::runtime_error("not found");
    //    tcout << "NoApp: " << controls << std::endl;
    //================
    const Point &pos = (pReq) ? pReq->hit : _base_pos.spec();
    const auto &traj = (pReq) ? pReq->getVisited() : StateTrajectory{ _base_pos };
    _store->insert(RoboMoves::Record{ aim, _base_pos.spec(), pos, new_controls, traj }); // <<< insert marker
    //================
    return _store->exactRecordByControl(new_controls);
}

bool RoboPos::TourEvo::containMarker(const Control &controls, muscle_t muscles, frames_t frame, const Point &aim)
{
    //if (!muscles) CERROR("Invalid muscles");
    //tcout << "-storeC=" << c << std::endl;
    // -----------------------------------
    Robo::Control marker{ bitset_t{ muscles }, frame, _lasts_init };
    auto new_controls = controls + marker;
    // -----------------------------------
    auto pReq = _store->exactRecordByControl(new_controls);
    bool exists = (pReq != NULL);
    // -----------------------------------
    if (pReq && pReq->aim != aim)
    {
        //tcout << "aim= " << pReq->aim << ' ' << aim << ' ' << new_controls << std::endl;
        CDEBUG("aim= " << pReq->aim << ' ' << aim << ' ' << new_controls);
        _store->replace(new_controls, [&aim](RoboMoves::Record &rec) { rec.updateTimeAim(aim); });
        exists = false;
    }
    // -----------------------------------
    if (!exists) appendMarker(controls, new_controls, aim);
    //else tcout << "-exists=" << c << std::endl;
    // -----------------------------------
    return exists;
}


RoboPos::TourEvo::TourEvo(Store *store, RoboI *robo, const tptree *config, const TargetI &target)
    : TourI(store, robo, config),
    _target(target),
    _reached_dist(0.000005),
    _oppo_penalty(0.5),
    _prev_dist_add(Robo::RoboI::minFrameMove),
    _base_pos(_robo->currState()),
    _n_muscles(_robo->musclesCount()),
    _n_acts(static_cast<muscle_t>(std::pow(2, _n_muscles))),
    _lasts_max(std::min(musclesMaxLasts(*_robo), TourI::too_long)),
    _lasts_init(minLasts() + 5),
    _max_ncontrols(8),
    _lasts_step(10),
    _step_back(55)
{
    _reached_dist = _config.get<double>(_T("evo.reached_dist"));
    _oppo_penalty = _config.get<double>(_T("evo.oppo_penalty"));
    _prev_dist_add = _config.get<double>(_T("evo.prev_dist_add"));
    _max_ncontrols = _config.get<frames_t>(_T("evo.max_ncontrols"));
    _lasts_step = _config.get<frames_t>(_T("evo.lasts_step"));
    _step_back = _config.get<frames_t>(_T("evo.step_back"));
}

//------------------------------------------------------------------------------
void RoboPos::TourEvo::printParameters() const
{
    CINFO("\nTourEvo:\n\nreached_dist=" << _reached_dist <<
          "\noppo_penalty=" << _oppo_penalty <<
          "\nprev_dist_add=" << _prev_dist_add <<
          "\nmax_ncontrols=" << _max_ncontrols <<
          "\nlasts_step=" << _lasts_step <<
          "\nstep_back=" << _step_back << std::endl);
}

bool RoboPos::TourEvo::runNestedPreMove(const Control &controls, muscle_t muscles, frames_t frame, const Point &aim, Point &hit)
{
    bool exists = containMarker(controls, muscles, frame, aim);
    //if (!muscles) CERROR("Empty muscles controls " << controls);
    if (!exists)
    {
        if (controls.size())
            _robo->move(controls, frame); // ??? breakings
        if (muscles)
            _robo->move(Robo::bitset_t{ muscles }, _lasts_max, _lasts_init);
        hit = _robo->position();
    }
    return exists;
}

bool RoboPos::TourEvo::runNestedForMuscle(joint_t, Control&, Point&)
{
    _robo->reset();
    _base_pos = _robo->currState();
    
    Goal goal(_target, _reached_dist * 2);
    goal.recalc(*_store);
    goal.show();
    /// ===== TEST ============================
    //while (goal.stage() < 5)
    //{
    //    goal.next_stage(_store, side);
    //    tcout << goal.biggest() << std::endl;
    //    //goal.show();
    //    MyWindowData::goals.push_back(goal.biggest());
    //}
    //return false;
    /// =======================================
    
    frames_t frames = 0; // максимальное число задействованных фрэймов
    Control controls /* лучшее управление */, controls_prev /* управление на предыдущем уровне иерархии */;
    auto p = _store->getClosestPoint(goal.biggest(), 0.5); // ближайшая точка из БД
    if (p.first) // если такая есть
    {
        /* not from scratch */
        controls = p.second.controls;

        auto pred = [](const auto &a, const auto &b) {return (a.start + a.lasts) < (b.start + b.lasts); };
        auto it = br::max_element(controls, pred);
        frames = it->start + it->lasts + 1;
    }

    distance_t the_best_dist = boost_distance(goal.biggest(), _base_pos.spec());
    std::stack<distance_t> distances{};

    //tcout << _lasts_init << std::endl << std::endl;
    CDEBUG(_lasts_init << std::endl);
    distance_t best_d = the_best_dist;

    RoboMoves::Record best_prev{};
    size_t step_back_count = 0;
    
    bool new_stage = false;
    
    CDEBUG("Evo: start goal=" << goal.biggest() << " pos=" << _robo->position());
    
    // TODO: Moves marked as USED
    // But not used for new point_targets
    
    bool done = false;
    while (!done)
    {
        joint_t best_acts = 0;
        frames_t best_lasts = 0;

        Point prev_pos{};
        distance_t prev_dist = best_d;
        
        bool one_watched = false;
        for (muscle_t acts = 1; (acts < (_n_acts - 1)) && (!done); ++acts)
        {
            Point pos{};
            // ============
            //tcout << "controls=" << controls << " acts=" << acts << std::endl;
            if (runNestedPreMove(controls, acts, frames, goal.biggest(), pos))
            {
                //tcout << "skip" << std::endl;
                continue;
            }
            // ============
            one_watched = true;
            prev_pos = pos;
            // ============
            bool first_move = true;
            frames_t lasts = 0;
            for (lasts = _lasts_init; (lasts < _lasts_max) && (!done); lasts += _lasts_step)
            {
                // ============
                distance_t d = boost_distance(goal.biggest(), pos);
                // ============
                if (!first_move)
                {
                    if (boost_distance(pos, prev_pos) <= RoboI::minFrameMove && lasts > 50)
                    {
                        //tcout << "last=" << lasts << " stop " << d << " p=" << pos << std::endl;
                        runNestedStop(acts, true); pos = _robo->position();
                        //tcout << "Evo: stop " << d << " p=" << pos << std::endl;
                        runNestedReset(controls, acts, frames, lasts, goal.biggest(), pos);
                        break;
                    }
                    if (d > (prev_dist + _prev_dist_add))
                    {
                        /* This muscle is FAIL now */
                        //Control c1(bitset_t{ acts }, frames, lasts);
                        //tcout << " FAIL d=" << d << ">" << prev_dist << " { " << c1 << " }" /*<< " pos=" << pos*/ << std::endl;
                        ////tcout << " FAIL pos=" << pos << " " << controls + c1 << std::endl;
                        runNestedStop(acts, true); pos = _robo->position();
                        ////tcout << "Evo: stop " << d << " p=" << pos << std::endl;
                        runNestedReset(controls, acts, frames, lasts, goal.biggest(), pos);

                        frames_t ll = lasts - std::min(lasts, 2 * _lasts_step);
                        if (ll > _lasts_init)
                        {
                            Control c = (controls + Control(bitset_t{ acts }, frames, ll));
                            Point useless{};
                            runNestedMove(c, useless);
                            _robo->reset();
                        }
                        break;
                    }
                }

                if (prev_dist > d || first_move)
                {
                    prev_dist = d;
                    prev_pos = pos;
                }
                first_move = false;

                if (prev_dist < _reached_dist)
                {
                    runNestedStop(acts, true); pos = _robo->position();
                    runNestedReset(controls, acts, frames, lasts, goal.biggest(), pos);

                    //tcout << "goals=" << goal.size() << std::endl;
                    done = !goal.next_stage(*_store);

                    //controls.clear(); // !!!
                    //std::stack<distance_t>().swap(distances); // clear
                    //best_d = the_best_dist = boost_distance(goal.biggest(), _base_pos);
                    //best_prev = RoboMoves::Record{};

                    Control c(bitset_t{ acts }, frames, lasts);
                    CINFO("Evo: next_stage goal=" << goal.biggest() << " controls= " << controls + c << " stage: " << goal.stage() << " goals=" << goal.size());
                    goal.show(); //RM

                    if (goal.stage() == 5)
                    {
                        CINFO(std::endl <<
                              _T("step: ") << (_step_distance / TourI::divToMiliMeters) << _T(" mm.") << std::endl <<
                              _T("Complexity: ") << complexity() <<
                              _T("  minutes:") << static_cast<double>(complexity()) / TourI::divToMinutes);
                    }
                    new_stage = true;
                    //runNestedPreMove(controls, 0, frames, goal.biggest(), pos);
                    //first_move==true??
                    continue;
                }

                // ============
                for (frames_t l = 0; l < _lasts_step; ++l)
                    _robo->step();
                // ============
                pos = _robo->position();
            }

            if (lasts >= _lasts_max)
            {
                Control c({ acts, frames, _lasts_max });
                CDEBUG("Evo: (lasts == lasts_max) " << " d=" << prev_dist << ' ' << c << " pos=" << pos);
                runNestedStop(acts, false); pos = _robo->position();
                runNestedReset(controls, acts, frames, _lasts_max, goal.biggest(), pos);
            }

            if (lasts > _lasts_init && best_d > prev_dist)
            {
                //tcout << " d=" << prev_dist << std::endl;
                best_acts = acts;
                best_d = prev_dist;
                best_lasts = lasts;
            }
        } // for muscle_t acts

        if (best_acts) /* if best_acts selected */
        {
            for (muscle_t m = 0; m < _n_muscles; ++m)
                if (best_acts & (1 << m))
                    controls.append({ m, frames, best_lasts });

            /* for exact by controls */
            best_prev = (*appendMarker(controls, controls, goal.biggest()));
            ////tcout << "---prev=" << controls << std::endl;

            frames += best_lasts + 1;
            distances.push(best_d);
            CDEBUG("Evo: --best-acts=" << controls);
        }

        if (_target.contain(prev_pos))
        {
            //CDEBUG("Evo: recalc goal");
            //goal.recalc(_store); /// <---- TODO !!!!!!!!!!!!!!!!!
            //goal.show();
        }

        if (the_best_dist > best_d)
            the_best_dist = best_d;

        ///stepBack(controls, controls_prev, best_d);
        //----------------------------------------------
        if (controls.size() && controls_prev == controls && !one_watched)
        {
            CDEBUG("Evo: regress " << controls);
            //tcout << "Evo: regress " << controls << std::endl;
            auto last = (controls.size() - 1);
            auto start = controls[last].start;

            // TEMP
            auto pReq = _store->exactRecordByControl(controls);
            if (pReq && !best_prev.trajectory.size())
                best_prev = *pReq;

            bool popped = false;
            for (auto i = controls.size(); i > 0; --i)
            {
                if (controls[i - 1].start != start)
                    break;

                if (controls[i - 1].lasts <= std::max<frames_t>(_step_back, 1))
                {
                    controls.pop(i - 1);
                    popped = true;
                }
                else
                {
                    controls[i - 1].lasts -= std::min(_step_back, controls[i - 1].lasts);
                    step_back_count++;

                    if (controls[i - 1].lasts > TourI::too_long)
                        throw std::logic_error("too_long");                
                }
            }

            distance_t d;
            if (distances.size() && popped)
            {
                distances.pop();
                step_back_count = 0;
                d = (distances.size()) ? distances.top() : boost_distance(goal.biggest(), _base_pos.spec());
                if (distances.size())
                {
                    auto pNewReq = _store->exactRecordByControl(controls);
                    if (pNewReq)
                        best_prev = *pNewReq;
                    else
                        throw std::runtime_error("best_prev");
                }
                else
                    best_prev = {};
            }
            else
            {
                //size_t i = (step_back_count * _step_back) / _robo->getVisitedRarity();
                size_t i = (step_back_count * _step_back);
                //if ((step_back_count * _step_back) % _robo->getVisitedRarity() || !i)
                //    i++;
                i = (best_prev.trajectory.size() - std::min(best_prev.trajectory.size(), i));
                d = boost_distance(goal.biggest(), best_prev.trajectory[i].spec());
            }
            best_d = d;

            if (controls.size())
            {
                auto &c = controls[controls.size() - 1];
                frames = controls.size() ? (c.start + c.lasts + 1) : 0;

                Point useless{};
                runNestedMove(controls, useless);
                _robo->reset();
            }
            else frames = 0;

            //CDEBUG(" become " << controls);
            ////tcout << "      become " << controls << std::endl;
            one_watched = true;
        }
        if (controls.size() >= _max_ncontrols)
        {
            //tcout << "Evo: maxsize " << controls << std::endl;
            bool popped = false;
            while (_max_ncontrols <= controls.size() && controls.size())
            {
                auto start = controls[controls.size() - 1].start;
                while (start == controls[controls.size() - 1].start && controls.size())
                {
                    controls.pop_back();
                    popped = true;
                }
            }
            auto &c = controls[controls.size() - 1];

            if (distances.size() && popped)
                distances.pop();
            best_d = (distances.size()) ? distances.top() : boost_distance(goal.biggest(), _base_pos.spec());
            frames = controls.size() ? (c.start + c.lasts + 1) : 0;
            ////tcout << "      become " << controls << std::endl;
            //tcout << "Evo: maxsize " << controls << std::endl;
            CDEBUG("Evo: maxsize " << controls)
            one_watched = true;
        }
        if (controls.size() == 0)
        {
            if (!one_watched)
            {
                if (!new_stage)
                {
                    CINFO("Evo: FAIL " << the_best_dist << " stage: " << goal.stage());
                    return false;
                }

                controls.clear(); // !!!
                std::stack<distance_t>().swap(distances); // clear
                best_d = the_best_dist = boost_distance(goal.biggest(), _base_pos.spec());
                best_prev = RoboMoves::Record{};
                new_stage = false;
            }
        }
        //----------------------------------------------
        boost::this_thread::interruption_point();
        //----------------------------------------------
        controls_prev = controls;
    } // while !done
    CINFO("Evo: DONE");
    return true;
}

bool RoboPos::TourEvo::runNestedReset(IN const Control &controls, muscle_t muscles, frames_t frame, frames_t lasts, const Point &aim, IN OUT Point &hit)
{
    bool exist = (_store->exactRecordByControl(controls) != NULL);
    if (!exist)
    {
        _complexity++;
        // ----------------------------        
        //if (muscles)
        Robo::Control marker{ bitset_t{ muscles }, frame, _lasts_init };
        appendMarker(controls, controls + marker, aim);
        // ----------------------------
        //if (muscles)
        Robo::Control act{ bitset_t{ muscles }, frame, lasts };
        _store->insert(RoboMoves::Record{ hit, _base_pos.spec(), hit, controls + act, _robo->trajectory() }); // the performed move
        // ----------------------------
    }
    _robo->reset();
    hit = Point{};
    return exist;
}

bool RoboPos::TourEvo::runNestedStop(IN const Robo::bitset_t &muscles, IN bool stop)
{
    if (stop)
        _robo->move(muscles, 1); // ??? breakings
    else
        _robo->move();
    return stop;
}

//bool RoboPos::TourEvo::runNestedStop(IN const Control &controls, IN bool stop)
//{
//    if (stop)
//        _robo->move(controls /*+ _breakings_controls*/);
//    else
//        _robo->move();
//    return stop;
//}
//
//bool RoboPos::TourEvo::runNestedStep(IN const Control &controls, OUT Point &robo_hit)
//{
//    boost::this_thread::interruption_point();
//    //----------------------------------------------
//    //const auto *pRec = _store->exactRecordByControl(controls);
//    //if (pRec)
//    //{
//    //    robo_hit = pRec->hit;
//    //    return true;
//    //}
//    //----------------------------------------------
//    _robo->step(controls /*+ _breakings_controls*/);
//    robo_hit = _robo->position();
//    //----------------------------------------------
//    return true;
//}
//
//bool RoboPos::TourEvo::runNestedStep(IN const Robo::bitset_t &muscles, IN Robo::frames_t lasts, OUT Point &robo_hit)
//{
//    boost::this_thread::interruption_point();
//    //----------------------------------------------
//    // ??? exactRecordByControl
//    //----------------------------------------------
//    _robo->step(muscles, lasts); // ??? breakings
//    robo_hit = _robo->position();
//    //----------------------------------------------
//    return true;
//}


//------------------------------------------------------------------------------
RoboPos::TourEvoSteps::TourEvoSteps(Store *store, RoboI *robo, const tptree *config, const TargetI &target) :
    TourEvo(store, robo, config, target)
    //,
    //_reached_dist(0.00001),
    //_oppo_penalty(1.),
    //_max_ncontrols(10),
    //_step_back(4)
{
    //_reached_dist = _config.get<double>(_T("evostep.reached_dist"));
    //_oppo_penalty = _config.get<double>(_T("evostep.oppo_penalty"));
    //_prev_dist_add = _config.get<double>(_T("evostep.prev_dist_add"));
    //_max_ncontrols = _config.get<frames_t>(_T("evostep.max_ncontrols"));
    //_step_back = _config.get<frames_t>(_T("evostep.step_back"));
}

//------------------------------------------------------------------------------
void RoboPos::TourEvoSteps::printParameters() const
{
    //CINFO(std::endl <<
    //      "reached_dist=" << _reached_dist << std::endl
    //      "oppo_penalty=" << _oppo_penalty << std::endl
    //      "prev_dist_add=" << _prev_dist_add << std::endl
    //      "max_ncontrols=" << _max_ncontrols << std::endl
    //      "lasts_step=" << _lasts_step << std::endl
    //      "step_back=" << _step_back << std::endl);
}

bool RoboPos::TourEvoSteps::runNestedForStep(IN const Robo::RoboI::bitwise &muscles, OUT Point &hit)
{
    boost::this_thread::interruption_point();
    //----------------------------------------------
    _robo->step(muscles);
    hit = _robo->position();
    //----------------------------------------------
    return true;
}

bool RoboPos::TourEvoSteps::runNestedForMuscle(joint_t /*joint !!!*/, Control &controls, Point &/*robo_hit !!!*/)
{
    _robo->reset();
    _base_pos = _robo->currState();

    Goal goal(_target, _reached_dist);
    goal.recalc(*_store);
    goal.show();

    const frames_t lasts_max = std::min(musclesMaxLasts(*_robo), TourI::too_long);
    const frames_t lasts_step = 1;

    const muscle_t n_muscles = _robo->musclesCount();
    const joint_t  n_acts = joint_t(std::pow(2, n_muscles));

    Point best_hit{};
    joint_t best_acts{};
    distance_t best_dist = boost_distance(goal.biggest(), _base_pos.spec());
    
    std::stack<distance_t> distances{};

    tcout << "Evo: start goal=" << goal.biggest() << " pos=" << _base_pos.spec() << std::endl;

    bool done = false;
    for (frames_t lasts = 0; (lasts < lasts_max) && (!done); lasts += lasts_step)
    {
        bool Act = false;

        for (muscle_t acts = 1; acts < (n_acts - 1) && !done; ++acts)
        {
            if (containMarker(controls, acts, lasts /* !!! 1 !!!*/, goal.biggest()))
                continue;

            if (controls.size()) /* pre-move */
                _robo->move(controls, lasts);
            //_robo->move(ctrls);

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
#ifdef DEBUG_RM
                tcout << _robo->frame() << " |";
                for (muscle_t m = 0; m < _robo->musclesCount(); ++m)
                    tcout << " " << _robo->muscleStatus(m) << "-" << _robo->lastsStatusT(m) << _robo->lastsStatus(m);
                tcout << std::endl;
#endif
            }

            runNestedStop(acts, true); Point pos = _robo->position();
            //tcout << "Evo: stop " << d << " p=" << pos << std::endl;
            runNestedReset(controls, acts, _robo->frame() /* ??? */, lasts, goal.biggest(), pos);
        } // end for acts

        if (!best_acts)
        {
            if (!Act)
            {
                tcout << "Evo: FAIL " << best_dist << std::endl;
                return false;
            }
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
                    else if (controls[i - 1].lasts > TourI::too_long)
                        throw std::logic_error("too_long");
                }
                lasts = (controls[controls.size() - 1].start + controls[controls.size() - 1].lasts);
                if (distances.size() && popped)
                    distances.pop();
                best_dist = (distances.size()) ? distances.top() : boost_distance(goal.biggest(), _base_pos.spec());
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
        
        if (_target.contain(best_hit))
        {
            //tcout << "Evo: recalc goal" << std::endl;
            goal.recalc(*_store);
            goal.show();
        }

        if (best_dist < _reached_dist)
        {
            /* it has walked throught a goal, want to stop at a goal */
            compansateOverHit(controls, goal.biggest());

            tcout << "Evo: next_stage goal=" << goal.biggest() << std::endl;
            done = !goal.next_stage(*_store);
            goal.show();
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
    const frames_t lasts = 1;// _robo->getVisitedRarity();

    _robo->move(controls);
    Point hit = _robo->position(), pt = hit;
    runNestedReset(controls, 0, 0, 0, goal/*.biggest()*/, pt);

    while (boost_distance(hit, goal) > _reached_dist)
    {
        const auto &visited = _robo->trajectory();
        distance_t best_path = boost_distance(hit, goal);
        size_t best_n = 0, n = 0;
        auto best_it = visited.end();

        const Point goal_dir = (hit - goal);
        for (auto curr = visited.begin(); curr != visited.end(); ++curr, ++n)
        {
            auto next = std::next(curr);
            if (next == visited.end())
                break;
            Point curr_dir = (next->spec() - curr->spec());

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
        const frames_t start = (best_n == prev_to_last) ? (best_n) : (best_n * 1/*_robo->getVisitedRarity()*/);
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

        _robo->move(controls);
        hit = _robo->position(), pt = hit;
        runNestedReset(controls, 0, 0, 0, goal/*.biggest()*/, pt);
    }

    {
        /* check-out */
        _robo->move(controls);
        hit = _robo->position();
        //const auto &visited = _robo->trajectory();
        _robo->reset();
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
    static void replace_muscle(const Robo::Actuator&, Robo::muscle_t /*n_muscles*/) {}
    static void split(const Robo::Control&) {}
    static void join(const Robo::Control&) {}
};

